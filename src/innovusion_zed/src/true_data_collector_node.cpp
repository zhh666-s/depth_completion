#include <iomanip>
#include <cmath>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <boost/filesystem.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

#include "innovusion_zed/completion_model.hpp"

class TrueDataCollector {
public:
    TrueDataCollector()
        : private_nh_("~"),
          completion_module_(getParam<std::string>("param_path",
                             ros::package::getPath("innovusion_zed") + "/config/camera_left.json")) {
        private_nh_.param("depth_scale", depth_scale_, 1000.0);
        private_nh_.param("preview_window", preview_window_, true);

        const std::string default_output_dir =
            ros::package::getPath("innovusion_zed") + "/true_data";
        private_nh_.param("output_dir", output_dir_, default_output_dir);
        private_nh_.param("lidar_topic", lidar_topic_, std::string("/iv_points"));
        private_nh_.param("cam_topic", cam_topic_, std::string("/camera/left/image_raw"));

        boost::filesystem::create_directories(output_dir_ + "/left");
        boost::filesystem::create_directories(output_dir_ + "/depth");
        boost::filesystem::create_directories(output_dir_ + "/pcd");

        if (preview_window_) {
            cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
        }

        lidar_sub_ = nh_.subscribe(lidar_topic_, 10, &TrueDataCollector::lidarCallback, this);
        if (isCompressedImageTopic(cam_topic_)) {
            compressed_image_sub_ =
                nh_.subscribe(cam_topic_, 60, &TrueDataCollector::compressedImageCallback, this);
        } else {
            raw_image_sub_ =
                nh_.subscribe(cam_topic_, 10, &TrueDataCollector::rawImageCallback, this);
        }

        ROS_INFO_STREAM("true_data_collector lidar_topic: " << lidar_topic_);
        ROS_INFO_STREAM("true_data_collector cam_topic: " << cam_topic_);
        ROS_INFO_STREAM("true_data_collector output_dir: " << output_dir_);
        ROS_INFO("true_data_collector manual mode: press 's' or Space in the image window to save, 'q' or Esc to quit.");
    }

    ~TrueDataCollector() {
        if (preview_window_) {
            cv::destroyWindow(window_name_);
        }
    }

private:
    template <typename T>
    T getParam(const std::string& name, const T& default_value) {
        T value;
        private_nh_.param(name, value, default_value);
        return value;
    }

    static bool isCompressedImageTopic(const std::string& topic) {
        const std::string suffix = "/compressed";
        return topic.size() >= suffix.size() &&
               topic.compare(topic.size() - suffix.size(), suffix.size(), suffix) == 0;
    }

    void rawImageCallback(const sensor_msgs::ImageConstPtr& image_msg) {
        cv_bridge::CvImageConstPtr image_ptr;
        try {
            image_ptr = cv_bridge::toCvShare(image_msg,
                                             sensor_msgs::image_encodings::BGR8);
        } catch (const cv_bridge::Exception& e) {
            ROS_ERROR_THROTTLE(2.0, "Failed to convert left image: %s", e.what());
            return;
        }

        cv::Mat preview_image = image_ptr->image.clone();
        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            latest_image_ = preview_image;
            latest_image_stamp_ = image_msg->header.stamp;
            has_latest_image_ = !latest_image_.empty();
        }

        showPreviewAndHandleKey(preview_image);
    }

    void compressedImageCallback(const sensor_msgs::CompressedImageConstPtr& image_msg) {
        cv::Mat decoded = cv::imdecode(
            cv::Mat(1, image_msg->data.size(), CV_8UC1,
                    const_cast<uint8_t*>(image_msg->data.data())),
            cv::IMREAD_COLOR);
        if (decoded.empty()) {
            ROS_ERROR_THROTTLE(2.0, "Failed to decode compressed image topic %s",
                               cam_topic_.c_str());
            return;
        }

        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            latest_image_ = decoded.clone();
            latest_image_stamp_ = image_msg->header.stamp;
            has_latest_image_ = true;
        }

        showPreviewAndHandleKey(decoded);
    }

    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
        pcl::PointCloud<pcl::PointXYZI> raw_cloud;
        pcl::fromROSMsg(*cloud_msg, raw_cloud);
        if (raw_cloud.empty()) {
            ROS_WARN_THROTTLE(2.0, "Received empty point cloud on topic %s",
                              lidar_topic_.c_str());
            return;
        }

        std::lock_guard<std::mutex> lock(cloud_mutex_);
        latest_cloud_ = raw_cloud;
        latest_cloud_stamp_ = cloud_msg->header.stamp;
        has_latest_cloud_ = true;
    }

    void showPreviewAndHandleKey(const cv::Mat& image) {
        if (!preview_window_ || image.empty()) {
            return;
        }

        cv::imshow(window_name_, image);
        const int key = cv::waitKey(1) & 0xff;
        if (key == 's' || key == 'S' || key == 32) {
            saveLatestSample();
        } else if (key == 'q' || key == 'Q' || key == 27) {
            ROS_INFO("true_data_collector quitting by keyboard request.");
            ros::shutdown();
        }
    }

    void saveLatestSample() {
        cv::Mat image;
        ros::Time image_stamp;
        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            if (!has_latest_image_ || latest_image_.empty()) {
                ROS_WARN("No left image cached yet, skip saving.");
                return;
            }
            image = latest_image_.clone();
            image_stamp = latest_image_stamp_;
        }

        pcl::PointCloud<pcl::PointXYZI> raw_cloud;
        ros::Time cloud_stamp;
        {
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            if (!has_latest_cloud_ || latest_cloud_.empty()) {
                ROS_WARN("No lidar cloud cached yet, skip saving.");
                return;
            }
            raw_cloud = latest_cloud_;
            cloud_stamp = latest_cloud_stamp_;
        }

        Eigen::MatrixXf points = pointCloudToEigen(raw_cloud);
        if (points.cols() == 0) {
            ROS_WARN_THROTTLE(2.0, "Received point cloud with no valid XYZ points.");
            return;
        }

        cv::Mat sparse_depth_m = completion_module_.projectSparseDepth(points);
        cv::Mat sparse_depth_u16;
        sparse_depth_m.convertTo(sparse_depth_u16, CV_16UC1, depth_scale_);

        std::stringstream stem;
        stem << "sample_" << std::setw(4) << std::setfill('0') << saved_samples_
             << "_" << std::fixed << std::setprecision(6)
             << cloud_stamp.toSec();

        const std::string left_path = output_dir_ + "/left/" + stem.str() + ".png";
        const std::string depth_path = output_dir_ + "/depth/" + stem.str() + ".png";
        const std::string pcd_path = output_dir_ + "/pcd/" + stem.str() + ".pcd";

        if (!cv::imwrite(left_path, image)) {
            ROS_ERROR_STREAM("Failed to save left image: " << left_path);
            return;
        }
        if (pcl::io::savePCDFileBinary(pcd_path, raw_cloud) != 0) {
            ROS_ERROR_STREAM("Failed to save raw point cloud: " << pcd_path);
            return;
        }
        if (!cv::imwrite(depth_path, sparse_depth_u16)) {
            ROS_ERROR_STREAM("Failed to save depth image: " << depth_path);
            return;
        }

        ++saved_samples_;
        ROS_INFO_STREAM("Saved true_data sample " << saved_samples_ << ": "
                        << stem.str() << ", image_stamp=" << image_stamp
                        << ", cloud_stamp=" << cloud_stamp);
    }

    Eigen::MatrixXf pointCloudToEigen(const pcl::PointCloud<pcl::PointXYZI>& cloud) const {
        std::vector<size_t> valid_indices;
        valid_indices.reserve(cloud.points.size());
        for (size_t i = 0; i < cloud.points.size(); ++i) {
            const pcl::PointXYZI& pt = cloud.points[i];
            if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
                valid_indices.push_back(i);
            }
        }

        Eigen::MatrixXf points(3, valid_indices.size());
        for (size_t i = 0; i < valid_indices.size(); ++i) {
            const pcl::PointXYZI& pt = cloud.points[valid_indices[i]];
            points.col(i) << pt.x, pt.y, pt.z;
        }
        return points;
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    CompletionModule completion_module_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber raw_image_sub_;
    ros::Subscriber compressed_image_sub_;

    std::string lidar_topic_;
    std::string cam_topic_;
    std::string output_dir_;
    int saved_samples_ = 0;
    double depth_scale_ = 1000.0;
    bool preview_window_ = true;
    const std::string window_name_ = "true_data_collector_left";
    cv::Mat latest_image_;
    ros::Time latest_image_stamp_;
    bool has_latest_image_ = false;
    std::mutex image_mutex_;
    pcl::PointCloud<pcl::PointXYZI> latest_cloud_;
    ros::Time latest_cloud_stamp_;
    bool has_latest_cloud_ = false;
    std::mutex cloud_mutex_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "true_data_collector_node");
    TrueDataCollector collector;
    ros::spin();
    return 0;
}
