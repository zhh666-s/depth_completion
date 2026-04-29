#include "innovusion_zed/ros_node.hpp"


RosNode::RosNode(const std::string &lidar_topic, const std::string &cam_topic,
                 const std::string &frame_id, const std::string param_path,
                 const std::string &publish_topic)
        : lidar_topic_(lidar_topic),
        cam_topic_(cam_topic),
        frame_id_(frame_id),
        param_(param_path),
        publish_topic_(publish_topic),
        point_cloud_(new pcl::PointCloud<pcl::PointXYZI>) 
{
    // 初始化发布
    point_cloud_publisher_ =
        nh_.advertise<sensor_msgs::PointCloud2>(publish_topic_, 10);
    undistorted_image_publisher_ =
        nh_.advertise<sensor_msgs::Image>(undistorted_image_topic_, 10);

    // 初始化订阅
    rslidar_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(
        nh_, lidar_topic_, 10);
    image_sub_ = new message_filters::Subscriber<sensor_msgs::CompressedImage>(
        nh_, cam_topic_, 10);

    // 初始化同步器
    sync_ = new message_filters::Synchronizer<MySyncPolicy>(
        MySyncPolicy(10), *rslidar_sub_, *image_sub_);
    sync_->registerCallback(boost::bind(&RosNode::callback, this, _1, _2));

    completion_module_ = std::make_unique<CompletionModule>(param_path);
}

RosNode::~RosNode() {
    delete rslidar_sub_;
    delete image_sub_;
    delete sync_;
}

void RosNode::run()
{
    ros::spin();
}


void RosNode::callback(const sensor_msgs::PointCloud2::ConstPtr &rslidar,
                       const sensor_msgs::CompressedImage::ConstPtr &image) {
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

    // 处理图像
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr =
            cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (cv_ptr->image.empty()) {
        ROS_ERROR("Failed to convert compressed image");
        return;
    }

    auto result = completion_module_->forward(points, cv_ptr->image);
    point_cloud_completion_ = result.second;

    // 发布结果
    sensor_msgs::PointCloud2 point_cloud_msg;
    pcl::toROSMsg(*point_cloud_completion_, point_cloud_msg);
    point_cloud_msg.header = rslidar->header;
    point_cloud_msg.header.frame_id = frame_id_;
    point_cloud_publisher_.publish(point_cloud_msg);


    ros::Time end_time = ros::Time::now();
    double elapsed_time = (end_time - start_time).toSec();
    ROS_INFO_STREAM(" [" << rslidar->header.stamp << "]: (" << elapsed_time
                        << ") " << publish_topic_);
}
