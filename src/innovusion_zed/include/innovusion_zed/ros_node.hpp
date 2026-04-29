#ifndef ROS_NODE_HPP
#define ROS_NODE_HPP

#include <chrono>
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/cache.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <memory>
#include <mutex>

#include "innovusion_zed/parameter.hpp"
#include "innovusion_zed/completion_model.hpp"

class RosNode
{
private:
    std::string lidar_topic_;
    std::string cam_topic_;
    Parameter param_;
    std::string frame_id_;
    std::string publish_topic_;
    std::string undistorted_image_topic_;
    std::string sparse_depth_topic_;
    std::string completed_depth_topic_;
    std::string data_folder_ = "/home/yxc/Code/Guangtong_ws/tmp"; // Folder to save the data

    ros::NodeHandle nh_;
    image_transport::ImageTransport image_transport_;
    ros::Publisher point_cloud_publisher_;
    ros::Publisher undistorted_image_publisher_;
    ros::Publisher sparse_depth_publisher_;
    ros::Publisher completed_depth_publisher_;
    
    ros::Subscriber lidar_sub_;

    image_transport::Subscriber raw_image_sub_;
    ros::Subscriber compressed_image_sub_;
    cv::Mat latest_image_;
    ros::Time latest_image_stamp_;
    bool has_latest_image_ = false;
    std::mutex image_mutex_;

    // 数据
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_;  // 点云数据指针
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_completion_;  // 处理后的点云数据指针
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    cv_bridge::CvImagePtr cv_ptr;  // ROS图像消息指针

    std::unique_ptr<CompletionModule> completion_module_;

public:
    RosNode(const std::string &lidar_topic, const std::string &cam_topic,
            const std::string &frame_id, const std::string param_path,
            const std::string &publish_topic);
    ~RosNode();

    void run();

    // 激光消息回调：从缓存中获取最新图像
    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr &rslidar);

    void callback(const sensor_msgs::PointCloud2::ConstPtr &rslidar,
                  const cv::Mat &image,
                  const ros::Time &image_stamp);

    void rawImageCallback(const sensor_msgs::ImageConstPtr &msg);
    void compressedImageCallback(const sensor_msgs::CompressedImage::ConstPtr &msg);

private:
    void savePointCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, const std::string& filename);
    void savePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const std::string& filename); // Overloaded for processed cloud
    void saveImage(const sensor_msgs::CompressedImage::ConstPtr& image_msg, const std::string& filename);
};

#endif // ROS_NODE_HPP
