#include "innovusion_zed/ros_node.hpp"
#include <iomanip>

RosNode::RosNode(const std::string &lidar_topic, const std::string &cam_topic,
                 const std::string &frame_id, const std::string param_path,
                 const std::string &publish_topic)
        : lidar_topic_(lidar_topic),
        cam_topic_(cam_topic),
        frame_id_(frame_id),
        param_(param_path),
        publish_topic_(publish_topic),
        undistorted_image_topic_(publish_topic_ + "/debug_image"),
        sparse_depth_topic_(publish_topic_ + "/depth_sparse"),
        completed_depth_topic_(publish_topic_ + "/depth_completed"),
        image_transport_(nh_),
        point_cloud_(new pcl::PointCloud<pcl::PointXYZI>) 
{
    // 初始化发布
    point_cloud_publisher_ =
        nh_.advertise<sensor_msgs::PointCloud2>(publish_topic_, 10);
    undistorted_image_publisher_ =
        nh_.advertise<sensor_msgs::Image>(undistorted_image_topic_, 10);
    sparse_depth_publisher_ =
        nh_.advertise<sensor_msgs::Image>(sparse_depth_topic_, 10);
    completed_depth_publisher_ =
        nh_.advertise<sensor_msgs::Image>(completed_depth_topic_, 10);

    // 订阅激光数据：每当激光消息到达时触发回调
    lidar_sub_ = nh_.subscribe(lidar_topic_, 10, &RosNode::lidarCallback, this);

    if (cam_topic_.size() >= 11 &&
        cam_topic_.compare(cam_topic_.size() - 11, 11, "/compressed") == 0) {
        compressed_image_sub_ = nh_.subscribe(
            cam_topic_, 60, &RosNode::compressedImageCallback, this);
        ROS_INFO_STREAM("innovusion_zed subscribing compressed image topic: " << cam_topic_);
    } else {
        raw_image_sub_ = image_transport_.subscribe(
            cam_topic_, 10, &RosNode::rawImageCallback, this);
        ROS_INFO_STREAM("innovusion_zed subscribing raw image topic: " << cam_topic_);
    }

    ROS_INFO_STREAM("innovusion_zed subscribing lidar topic: " << lidar_topic_);
    ROS_INFO_STREAM("innovusion_zed publishing completed cloud topic: " << publish_topic_);
    ROS_INFO_STREAM("innovusion_zed publishing sparse depth topic: " << sparse_depth_topic_);
    ROS_INFO_STREAM("innovusion_zed publishing completed depth topic: " << completed_depth_topic_);

    completion_module_ = std::make_unique<CompletionModule>(param_path);
}

RosNode::~RosNode() {
    // delete rslidar_sub_;
    // delete image_sub_;
    // delete sync_;
}

void RosNode::run()
{
    ros::spin();
}

// Helper variable to count saved images
static int image_count_ = 0;
static int lidar_count_ = 0;

// Image callback function (updates the latest image)
void RosNode::rawImageCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception &e) {
        ROS_ERROR_THROTTLE(2.0, "Failed to convert raw image: %s", e.what());
        return;
    }

    std::lock_guard<std::mutex> lock(image_mutex_);
    latest_image_ = cv_ptr->image.clone();
    latest_image_stamp_ = msg->header.stamp;
    has_latest_image_ = !latest_image_.empty();
}

void RosNode::compressedImageCallback(const sensor_msgs::CompressedImage::ConstPtr &msg) {
    cv::Mat decoded = cv::imdecode(
        cv::Mat(1, msg->data.size(), CV_8UC1, const_cast<uint8_t *>(msg->data.data())),
        cv::IMREAD_COLOR);
    if (decoded.empty()) {
        ROS_ERROR_THROTTLE(2.0, "Failed to decode compressed image on topic %s",
                           cam_topic_.c_str());
        return;
    }

    std::lock_guard<std::mutex> lock(image_mutex_);
    latest_image_ = decoded;
    latest_image_stamp_ = msg->header.stamp;
    has_latest_image_ = true;
}

// 激光数据回调：获取最新图像并调用数据处理函数
void RosNode::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr &rslidar) {
    cv::Mat latest_image;
    ros::Time image_stamp;
    {
        std::lock_guard<std::mutex> lock(image_mutex_);
        if (has_latest_image_) {
            latest_image = latest_image_.clone();
            image_stamp = latest_image_stamp_;
        }
    }

    if (latest_image.empty()) {
        ROS_WARN_THROTTLE(2.0, "Waiting for image topic %s before processing lidar topic %s",
                          cam_topic_.c_str(), lidar_topic_.c_str());
        return;
    }

    // // 保存 lidar 数据
    // std::stringstream ss;
    // ss << data_folder_ << "/lidar_" << std::setw(6) << std::setfill('0') << lidar_count_ << ".pcd";
    // savePointCloud(rslidar, ss.str());
    // lidar_count_++;
    // ROS_INFO("Saved lidar data: %s", ss.str().c_str());

    callback(rslidar, latest_image, image_stamp);
}


void RosNode::callback(const sensor_msgs::PointCloud2::ConstPtr &rslidar,
                       const cv::Mat &image,
                       const ros::Time &image_stamp) {
    ros::Time start_time = ros::Time::now();


    // 处理点云
    pcl::fromROSMsg(*rslidar, *point_cloud_);

    // 预分配内存并过滤无效点
    std::vector<size_t> valid_indices;
    valid_indices.reserve(point_cloud_->points.size());
    for (size_t i = 0; i < point_cloud_->points.size(); ++i) {
        const auto& pt = point_cloud_->points[i];
        if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z)) {
            valid_indices.push_back(i);
        }
    }

    // 构建有效点云矩阵
    Eigen::MatrixXf points(3, valid_indices.size());
    for (size_t i = 0; i < valid_indices.size(); ++i) {
        const auto& pt = point_cloud_->points[valid_indices[i]];
        points.col(i) << pt.x, pt.y, pt.z;
    }

    if (image.empty()) {
        ROS_ERROR("Received empty image");
        return;
    }

    auto result = completion_module_->forward(points, image);
    point_cloud_completion_ = result.completed_points;

    // 发布结果
    sensor_msgs::PointCloud2 point_cloud_msg;
    pcl::toROSMsg(*point_cloud_completion_, point_cloud_msg);
    point_cloud_msg.header = rslidar->header;
    point_cloud_msg.header.frame_id = frame_id_;
    point_cloud_publisher_.publish(point_cloud_msg);

    if (sparse_depth_publisher_.getNumSubscribers() > 0) {
        sensor_msgs::ImagePtr sparse_depth_msg =
            cv_bridge::CvImage(std_msgs::Header(),
                               sensor_msgs::image_encodings::TYPE_32FC1,
                               result.sparse_depth)
                .toImageMsg();
        sparse_depth_msg->header.stamp = rslidar->header.stamp;
        sparse_depth_msg->header.frame_id = frame_id_;
        sparse_depth_publisher_.publish(sparse_depth_msg);
    }

    if (completed_depth_publisher_.getNumSubscribers() > 0) {
        sensor_msgs::ImagePtr completed_depth_msg =
            cv_bridge::CvImage(std_msgs::Header(),
                               sensor_msgs::image_encodings::TYPE_32FC1,
                               result.completed_depth)
                .toImageMsg();
        completed_depth_msg->header.stamp = rslidar->header.stamp;
        completed_depth_msg->header.frame_id = frame_id_;
        completed_depth_publisher_.publish(completed_depth_msg);
    }

    if (undistorted_image_publisher_.getNumSubscribers() > 0) {
        sensor_msgs::ImagePtr debug_image =
            cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, image)
                .toImageMsg();
        debug_image->header.stamp = image_stamp.isZero() ? rslidar->header.stamp : image_stamp;
        debug_image->header.frame_id = frame_id_;
        undistorted_image_publisher_.publish(debug_image);
    }


    // // 保存处理后的点云
    // std::stringstream ss_completed;
    // ss_completed << data_folder_ << "/completed_" << std::setw(6) << std::setfill('0') << lidar_count_ << ".pcd";
    // savePointCloud(point_cloud_completion_, ss_completed.str());
    // lidar_count_++;
    // ROS_INFO("Saved completed point cloud: %s", ss_completed.str().c_str());


    ros::Time end_time = ros::Time::now();
    double elapsed_time = (end_time - start_time).toSec();
    ROS_INFO_STREAM(" [" << rslidar->header.stamp << "]: (" << elapsed_time
                        << "s) " << publish_topic_);
}


void RosNode::savePointCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, const std::string& filename) {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);
    pcl::io::savePCDFileBinary(filename, cloud);
}

// 保存处理后的点云
void RosNode::savePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const std::string& filename) {
    pcl::io::savePCDFileBinary(filename, *cloud);
}

void RosNode::saveImage(const sensor_msgs::CompressedImage::ConstPtr& image_msg, const std::string& filename) {
    cv::Mat image = cv::imdecode(cv::Mat(1, image_msg->data.size(), CV_8UC1, (void*)image_msg->data.data()), cv::IMREAD_COLOR);
    if (!image.empty()) {
        cv::imwrite(filename, image);
    } else {
        ROS_ERROR("Could not decode compressed image for saving.");
    }
}
