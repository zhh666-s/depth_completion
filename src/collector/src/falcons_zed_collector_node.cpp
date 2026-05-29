#include <iomanip>
#include <cmath>
#include <cstring>
#include <limits>
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

#include "collector/completion_model.hpp"

class FalconsZedCollector {
public:
    FalconsZedCollector()
        : private_nh_("~"),
          completion_module_(getParam<std::string>("param_path",
                             ros::package::getPath("innovusion_zed") + "/config/camera_left.json")) {
        private_nh_.param("depth_scale", depth_scale_, 1000.0);
        private_nh_.param("preview_window", preview_window_, true);

        const std::string default_output_dir =
            ros::package::getPath("collector") + "/data/falcons_zed";
        private_nh_.param("output_dir", output_dir_, default_output_dir);
        private_nh_.param("lidar_topic", lidar_topic_, std::string("/iv_points"));
        private_nh_.param("cam_topic", cam_topic_, std::string("/camera/left/image_raw"));

        boost::filesystem::create_directories(output_dir_ + "/left");
        boost::filesystem::create_directories(output_dir_ + "/depth");
        boost::filesystem::create_directories(output_dir_ + "/pcd");

        if (preview_window_) {
            cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
        }

        lidar_sub_ = nh_.subscribe(lidar_topic_, 10, &FalconsZedCollector::lidarCallback, this);
        if (isCompressedImageTopic(cam_topic_)) {
            compressed_image_sub_ =
                nh_.subscribe(cam_topic_, 60, &FalconsZedCollector::compressedImageCallback, this);
        } else {
            raw_image_sub_ =
                nh_.subscribe(cam_topic_, 10, &FalconsZedCollector::rawImageCallback, this);
        }

        ROS_INFO_STREAM("falcons_zed_collector lidar_topic: " << lidar_topic_);
        ROS_INFO_STREAM("falcons_zed_collector cam_topic: " << cam_topic_);
        ROS_INFO_STREAM("falcons_zed_collector output_dir: " << output_dir_);
        ROS_INFO("falcons_zed_collector manual mode: press 's' or Space in the image window to save, 'q' or Esc to quit.");
    }

    ~FalconsZedCollector() {
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
        if (!logged_cloud_fields_) {
            ROS_INFO_STREAM("falcons_zed_collector PointCloud2 fields: "
                            << describeCloudFields(*cloud_msg));
            logged_cloud_fields_ = true;
        }

        pcl::PointCloud<pcl::PointXYZI> raw_cloud = pointCloud2ToXYZI(*cloud_msg);
        if (raw_cloud.empty()) {
            ROS_WARN_THROTTLE(2.0, "Received empty point cloud on topic %s",
                              lidar_topic_.c_str());
            return;
        }
        logIntensityStats(raw_cloud);

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
            ROS_INFO("falcons_zed_collector quitting by keyboard request.");
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
        ROS_INFO_STREAM("Saved falcons_zed sample " << saved_samples_ << ": "
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

    void logIntensityStats(const pcl::PointCloud<pcl::PointXYZI>& cloud) {
        if (logged_intensity_stats_) {
            return;
        }

        size_t finite_count = 0;
        size_t nonzero_count = 0;
        float min_intensity = std::numeric_limits<float>::max();
        float max_intensity = std::numeric_limits<float>::lowest();

        for (const pcl::PointXYZI& point : cloud.points) {
            if (!std::isfinite(point.intensity)) {
                continue;
            }
            ++finite_count;
            if (point.intensity != 0.0f) {
                ++nonzero_count;
            }
            min_intensity = std::min(min_intensity, point.intensity);
            max_intensity = std::max(max_intensity, point.intensity);
        }

        logged_intensity_stats_ = true;
        if (finite_count == 0) {
            ROS_WARN_STREAM("falcons_zed_collector intensity stats: no finite intensity values.");
            return;
        }

        ROS_INFO_STREAM("falcons_zed_collector intensity stats: min=" << min_intensity
                        << ", max=" << max_intensity
                        << ", nonzero=" << nonzero_count
                        << "/" << finite_count);
        if (nonzero_count == 0) {
            ROS_WARN_STREAM("falcons_zed_collector parsed intensity is all zero. "
                            "If the field list above contains no intensity/reflectivity-like "
                            "field, check the lidar driver output topic.");
        }
    }

    static std::string describeCloudFields(const sensor_msgs::PointCloud2& cloud_msg) {
        std::ostringstream out;
        for (size_t i = 0; i < cloud_msg.fields.size(); ++i) {
            const sensor_msgs::PointField& field = cloud_msg.fields[i];
            if (i > 0) {
                out << ", ";
            }
            out << field.name << "(offset=" << field.offset
                << ", datatype=" << static_cast<int>(field.datatype)
                << ", count=" << field.count << ")";
        }
        return out.str();
    }

    static const sensor_msgs::PointField* findField(
        const sensor_msgs::PointCloud2& cloud_msg,
        const std::vector<std::string>& names) {
        for (const std::string& name : names) {
            for (const sensor_msgs::PointField& field : cloud_msg.fields) {
                if (field.name == name) {
                    return &field;
                }
            }
        }
        return nullptr;
    }

    template <typename T>
    static T readScalar(const uint8_t* data) {
        T value;
        std::memcpy(&value, data, sizeof(T));
        return value;
    }

    static bool readFieldAsFloat(const uint8_t* point_data,
                                 const sensor_msgs::PointField& field,
                                 float& value) {
        const uint8_t* data = point_data + field.offset;
        switch (field.datatype) {
        case sensor_msgs::PointField::INT8:
            value = static_cast<float>(readScalar<int8_t>(data));
            return true;
        case sensor_msgs::PointField::UINT8:
            value = static_cast<float>(readScalar<uint8_t>(data));
            return true;
        case sensor_msgs::PointField::INT16:
            value = static_cast<float>(readScalar<int16_t>(data));
            return true;
        case sensor_msgs::PointField::UINT16:
            value = static_cast<float>(readScalar<uint16_t>(data));
            return true;
        case sensor_msgs::PointField::INT32:
            value = static_cast<float>(readScalar<int32_t>(data));
            return true;
        case sensor_msgs::PointField::UINT32:
            value = static_cast<float>(readScalar<uint32_t>(data));
            return true;
        case sensor_msgs::PointField::FLOAT32:
            value = readScalar<float>(data);
            return true;
        case sensor_msgs::PointField::FLOAT64:
            value = static_cast<float>(readScalar<double>(data));
            return true;
        default:
            return false;
        }
    }

    pcl::PointCloud<pcl::PointXYZI> pointCloud2ToXYZI(
        const sensor_msgs::PointCloud2& cloud_msg) const {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        cloud.header = pcl_conversions::toPCL(cloud_msg.header);
        cloud.width = cloud_msg.width * cloud_msg.height;
        cloud.height = 1;
        cloud.is_dense = cloud_msg.is_dense;
        cloud.points.resize(cloud.width);

        const sensor_msgs::PointField* x_field = findField(cloud_msg, {"x"});
        const sensor_msgs::PointField* y_field = findField(cloud_msg, {"y"});
        const sensor_msgs::PointField* z_field = findField(cloud_msg, {"z"});
        const sensor_msgs::PointField* intensity_field =
            findField(cloud_msg, {"intensity", "reflectivity", "reflectance",
                                  "intensities", "i"});

        if (!x_field || !y_field || !z_field) {
            ROS_ERROR_THROTTLE(2.0, "PointCloud2 on %s is missing x/y/z fields.",
                               lidar_topic_.c_str());
            cloud.clear();
            return cloud;
        }
        if (!intensity_field) {
            ROS_WARN_THROTTLE(5.0,
                              "PointCloud2 on %s has no intensity-like field; "
                              "saved PCD intensity will be 0.",
                              lidar_topic_.c_str());
        }

        const size_t point_count = static_cast<size_t>(cloud_msg.width) * cloud_msg.height;
        for (size_t i = 0; i < point_count; ++i) {
            const uint8_t* point_data = &cloud_msg.data[i * cloud_msg.point_step];
            pcl::PointXYZI& point = cloud.points[i];
            point.intensity = 0.0f;

            if (!readFieldAsFloat(point_data, *x_field, point.x) ||
                !readFieldAsFloat(point_data, *y_field, point.y) ||
                !readFieldAsFloat(point_data, *z_field, point.z)) {
                point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
                continue;
            }
            if (intensity_field) {
                readFieldAsFloat(point_data, *intensity_field, point.intensity);
            }
        }

        return cloud;
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
    const std::string window_name_ = "falcons_zed_collector_left";
    cv::Mat latest_image_;
    ros::Time latest_image_stamp_;
    bool has_latest_image_ = false;
    std::mutex image_mutex_;
    pcl::PointCloud<pcl::PointXYZI> latest_cloud_;
    ros::Time latest_cloud_stamp_;
    bool has_latest_cloud_ = false;
    bool logged_cloud_fields_ = false;
    bool logged_intensity_stats_ = false;
    std::mutex cloud_mutex_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "falcons_zed_collector_node");
    FalconsZedCollector collector;
    ros::spin();
    return 0;
}
