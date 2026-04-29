#ifndef COMPLETION_MODEL_HPP
#define COMPLETION_MODEL_HPP

#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/eigen.hpp> 

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/filesystem.hpp>

#include <omp.h>

#include "innovusion_zed/parameter.hpp"

// Full kernels
const cv::Mat FULL_KERNEL_3 = cv::Mat::ones(3, 3, CV_8U);
const cv::Mat FULL_KERNEL_5 = cv::Mat::ones(5, 5, CV_8U);
const cv::Mat FULL_KERNEL_7 = cv::Mat::ones(7, 7, CV_8U);
const cv::Mat FULL_KERNEL_9 = cv::Mat::ones(9, 9, CV_8U);
const cv::Mat FULL_KERNEL_31 = cv::Mat::ones(31, 31, CV_8U);

// Cross and diamond kernels using initializer lists
const cv::Mat CROSS_KERNEL_3 = (cv::Mat_<uint8_t>(3,3) << 
    0,1,0, 
    1,1,1, 
    0,1,0);

const cv::Mat CROSS_KERNEL_5 = (cv::Mat_<uint8_t>(5,5) << 
    0,0,1,0,0,
    0,0,1,0,0,
    1,1,1,1,1,
    0,0,1,0,0,
    0,0,1,0,0);

const cv::Mat CROSS_KERNEL_7 = (cv::Mat_<uint8_t>(7,7) << 
    0,0,0,1,0,0,0,
    0,0,0,1,0,0,0,
    0,0,0,1,0,0,0,
    1,1,1,1,1,1,1,
    0,0,0,1,0,0,0,
    0,0,0,1,0,0,0,
    0,0,0,1,0,0,0);

const cv::Mat DIAMOND_KERNEL_5 = (cv::Mat_<uint8_t>(5,5) << 
    0,0,1,0,0,
    0,1,1,1,0,
    1,1,1,1,1,
    0,1,1,1,0,
    0,0,1,0,0);

const cv::Mat DIAMOND_KERNEL_7 = (cv::Mat_<uint8_t>(7,7) << 
    0,0,0,1,0,0,0,
    0,0,1,1,1,0,0,
    0,1,1,1,1,1,0,
    1,1,1,1,1,1,1,
    0,1,1,1,1,1,0,
    0,0,1,1,1,0,0,
    0,0,0,1,0,0,0);


class CompletionModule 
{
private:
    // 标定参数
    Parameter param_;

    // 模型参数
    float MIN_VALID_DEPTH = 0.1f;
    float MAX_DEPTH = 80.0f;
    float DEPTH_SCALE = 256.0f;
    float DEFAULT_THRESHOLD = 0.05f;
    bool EXTRAPOLATE = true;
    std::string BLUR_TYPE = "gaussian";
    
    void initializeParameters();
    void initMeshgrid();
    cv::Mat xyz2depth(const Eigen::MatrixXf& points_3d) const;  
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr depth2xyz(const cv::Mat& depth, const cv::Mat& image) const;
    void depthConstraintLayer(cv::Mat& depth, const cv::Mat& lidar, const cv::Mat& kernel) const;
    void process(cv::Mat& depth, const Eigen::MatrixXf& points);

    // 配置参数
    int height_, width_;
    
    // 相机参数
    cv::Mat camera_matrix_;
    cv::Mat new_camera_matrix_mat_;
    cv::Mat dist_coeffs_;
    Eigen::Matrix3f new_camera_matrix_;
    Eigen::Matrix3f rotation_matrix_;
    Eigen::Vector3f translation_vector_;
    
    Eigen::Affine3f transform_ = Eigen::Affine3f::Identity();

    Eigen::Matrix3f rotation_matrix_l2c_;
    Eigen::Vector3f translation_vector_l2c_;

    float fx_, fy_, cx_, cy_;

    // 畸变矫正查找表
    cv::Mat map1_, map2_;

    // 网格查找表
    cv::Mat x_mesh_;
    cv::Mat y_mesh_;


public:
    CompletionModule(const std::string& param_path);
    ~CompletionModule() = default;
    
    // 禁用拷贝
    CompletionModule(const CompletionModule&) = delete;
    CompletionModule& operator=(const CompletionModule&) = delete;

    struct ForwardOutput {
        cv::Mat sparse_depth;      // 32FC1, unit: meter
        cv::Mat completed_depth;   // 32FC1, unit: meter
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr completed_points;
    };

    // 处理函数
    ForwardOutput forward(const Eigen::MatrixXf& points, const cv::Mat& image);
};


#endif // COMPLETION_MODEL_HPP
