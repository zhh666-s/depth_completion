#ifndef ROS_NODE_HPP
#define ROS_NODE_HPP

#include <chrono>
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

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

    ros::NodeHandle nh_;
    ros::Publisher point_cloud_publisher_;
    ros::Publisher undistorted_image_publisher_;
    
    message_filters::Subscriber<sensor_msgs::PointCloud2> *rslidar_sub_;  // 激光雷达点云
    message_filters::Subscriber<sensor_msgs::CompressedImage> *image_sub_;  // 相机图像
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::PointCloud2, sensor_msgs::CompressedImage> 
        MySyncPolicy;  // 同步策略
    message_filters::Synchronizer<MySyncPolicy> *sync_;

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

    void callback(const sensor_msgs::PointCloud2::ConstPtr &rslidar,
                  const sensor_msgs::CompressedImage::ConstPtr &image);
};

#endif // ROS_NODE_HPP