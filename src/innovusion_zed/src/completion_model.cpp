#include "innovusion_zed/completion_model.hpp"


template <typename Derived>
void printMatrix(const Eigen::EigenBase<Derived>& matrix, const std::string& path = "/workspace/code/Guangtong_ws/tmp/points_3d.txt"){
    std::cout << "*******************************" << std::endl;
    const auto& mat = matrix.derived();
    int rows = mat.rows();
    int cols = mat.cols();
    int size = mat.size();
    std::cout << "Matrix size: " << size << std::endl
                << "Matrix rows: " << rows << std::endl
                << "Matrix cols: " << cols << std::endl;
    if (!path.empty()) {
        std::ofstream out(path);
        for (int i = 0; i < cols; ++i) {
            out << "x: " << mat(0, i) << ", y: " << mat(1, i) << ", z: " << mat(2, i) << std::endl;
        }
    }

    std::cout << "*******************************" << std::endl;
}

void printMat(const cv::Mat& matrix, const std::string& path = "/workspace/code/Guangtong_ws/tmp/Mat.txt"){
    std::cout << "*******************************" << std::endl;
    int rows = matrix.rows;
    int cols = matrix.cols;
    int size = matrix.size().area();
    std::cout << "Matrix size: " << size << std::endl
              << "Matrix rows: " << rows << std::endl
              << "Matrix cols: " << cols << std::endl;
    if (!path.empty()) {
        cv::Mat floatMat;
        matrix.convertTo(floatMat, CV_32F);
        std::ofstream out(path);
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                out << floatMat.at<float>(i, j) << " ";
            }
            out << std::endl;
        }
    }
    std::cout << "*******************************" << std::endl;
}

Eigen::MatrixXf loadPCD(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open file");
    }

    std::string line;  // Line buffer
    std::vector<std::string> fields;  // Field names
    std::vector<Eigen::VectorXf> tempPointCloud;

    // Read lines until fields are set
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string marker;
        iss >> marker;
        
        if (marker == "FIELDS") {
            std::string field;
            while (iss >> field) {
                if (field == "x" || field == "y" || field == "z" || field == "intensity") {
                    fields.push_back(field);
                }
                // Optionally handle RGB differently
            }
        } else if (marker == "DATA") {
            break; // Assumes data section starts immediately after this line
        }
    }

    // Prepare to read the data points
    int numFields = fields.size();
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        Eigen::VectorXf point(numFields);
        float value;
        int i = 0;
        while (iss >> value && i < numFields) {
            point[i++] = value;
        }
        if (i == numFields) { // Ensure we have read enough fields
            if (point[0] != 0.0 && point[1] != 0.0 && point[2] != 0.0)  // Skip zero points
                tempPointCloud.push_back(point);
            // else
            //     std::cout << "Zero point skipped." << std::endl;
        }
    }

    // Convert std::vector<Eigen::VectorXf> to Eigen::MatrixXf
    Eigen::MatrixXf pointCloud(numFields, tempPointCloud.size());
    for (size_t i = 0; i < tempPointCloud.size(); ++i) {
        pointCloud.col(i) = tempPointCloud[i];
    }

    return pointCloud;
}

void savePCD(const std::string& file_name, const Eigen::MatrixXf& point_cloud, bool has_color){
    std::ofstream pcd_file(file_name);
    if (!pcd_file.is_open()) {
        std::cerr << "Error opening file for writing: " << file_name << std::endl;
        return;
    }

    // Write the PCD header
    pcd_file << "# .PCD v0.7 - Point Cloud Data file format\n"
             << "VERSION 0.7\n";
    
    if (has_color) {
        pcd_file << "FIELDS x y z rgb\n"
                 << "SIZE 4 4 4 4\n"
                 << "TYPE F F F U\n"
                 << "COUNT 1 1 1 1\n";
    } else {
        pcd_file << "FIELDS x y z\n"
                 << "SIZE 4 4 4\n"
                 << "TYPE F F F\n"
                 << "COUNT 1 1 1\n";
    }
    
    pcd_file << "WIDTH " << point_cloud.cols() << "\n"
             << "HEIGHT 1\n"
             << "VIEWPOINT 0 0 0 1 0 0 0\n"
             << "POINTS " << point_cloud.cols() << "\n"
             << "DATA ascii\n";

    for (int i = 0; i < point_cloud.cols(); ++i) {
        // 跳过无效点
        if (point_cloud(0, i) == 0 && point_cloud(1, i) == 0 && point_cloud(2, i) == 0) {
            continue;
        }
        if (std::isnan(point_cloud(0, i)) || std::isnan(point_cloud(1, i)) || std::isnan(point_cloud(2, i))) {
            continue;
        }

        // 写入XYZ坐标
        pcd_file << point_cloud(0, i) << " " 
                 << point_cloud(1, i) << " " 
                 << point_cloud(2, i);

        // 如果有颜色信息，将RGB转换为单个浮点数并写入
        if (has_color && point_cloud.rows() >= 6) {
            uint32_t rgb = ((uint32_t)point_cloud(3, i) << 16 | 
                           (uint32_t)point_cloud(4, i) << 8 | 
                           (uint32_t)point_cloud(5, i));
            float rgb_float = *reinterpret_cast<float*>(&rgb);
            pcd_file << " " << rgb;
        }
        pcd_file << "\n";
    }
    pcd_file.close();
}

CompletionModule::CompletionModule(const std::string& param_path) 
    : param_(param_path)
{
    initializeParameters();
}

void CompletionModule::initializeParameters() 
{   

    dist_coeffs_ =
        (cv::Mat_<double>(5, 1) << param_.getDistortion()[0],
        param_.getDistortion()[1], param_.getDistortion()[2],
        param_.getDistortion()[3], param_.getDistortion()[4]);

    camera_matrix_ = (cv::Mat_<double>(3, 3) << 
        param_.getIntrinsic()[0], param_.getIntrinsic()[1], param_.getIntrinsic()[2], 
        param_.getIntrinsic()[3], param_.getIntrinsic()[4], param_.getIntrinsic()[5],
        param_.getIntrinsic()[6], param_.getIntrinsic()[7], param_.getIntrinsic()[8]);
    
    new_camera_matrix_mat_ = (cv::Mat_<double>(3, 3) << 
        param_.getUndistortIntrinsic()[0], param_.getUndistortIntrinsic()[1], param_.getUndistortIntrinsic()[2], 
        param_.getUndistortIntrinsic()[3], param_.getUndistortIntrinsic()[4], param_.getUndistortIntrinsic()[5],
        param_.getUndistortIntrinsic()[6], param_.getUndistortIntrinsic()[7], param_.getUndistortIntrinsic()[8]);

    new_camera_matrix_ << 
        param_.getUndistortIntrinsic()[0], param_.getUndistortIntrinsic()[1], param_.getUndistortIntrinsic()[2], 
        param_.getUndistortIntrinsic()[3], param_.getUndistortIntrinsic()[4], param_.getUndistortIntrinsic()[5],
        param_.getUndistortIntrinsic()[6], param_.getUndistortIntrinsic()[7], param_.getUndistortIntrinsic()[8];
    
    rotation_matrix_ << 
        param_.getRotation()[0], param_.getRotation()[1], param_.getRotation()[2],
        param_.getRotation()[3], param_.getRotation()[4], param_.getRotation()[5],
        param_.getRotation()[6], param_.getRotation()[7], param_.getRotation()[8];


    translation_vector_ << param_.getTranslation()[0], param_.getTranslation()[1], param_.getTranslation()[2];

    // The calibration file stores camera<->lidar extrinsic in a different axis convention
    // from the online lidar stream. For stable projection, we use inverse extrinsic for
    // lidar->camera after axis remapping in xyz2depth().
    rotation_matrix_l2c_ = rotation_matrix_.inverse();
    translation_vector_l2c_ = -rotation_matrix_l2c_ * translation_vector_;

    // Depth back-projection uses camera->lidar transform first, then converts axes back to
    // the online lidar convention in depth2xyz().
    transform_.translation() << translation_vector_(0), translation_vector_(1), translation_vector_(2);
    transform_.rotate(rotation_matrix_);

    // 设置图像参数
    height_ = param_.getImageSize()[1];
    width_ = param_.getImageSize()[0];

    // 提取相机参数
    fx_ = new_camera_matrix_(0, 0);
    fy_ = new_camera_matrix_(1, 1);
    cx_ = new_camera_matrix_(0, 2);
    cy_ = new_camera_matrix_(1, 2);

    // 畸变矫正查找表初始化
    cv::initUndistortRectifyMap(camera_matrix_, dist_coeffs_, cv::Mat(),
                                new_camera_matrix_mat_, cv::Size(width_, height_), 
                                CV_16SC2, map1_, map2_);

    // 初始化内存
    initMeshgrid();
}

void CompletionModule::initMeshgrid() {
    x_mesh_=cv::Mat(cv::Size(width_, height_), CV_32FC1);
    y_mesh_=cv::Mat(cv::Size(width_, height_), CV_32FC1);
    
    for(int i = 0; i < height_; ++i) {
        for(int j = 0; j < width_; ++j) {
            x_mesh_.at<float>(i,j) = (j - cx_) / fx_;
            y_mesh_.at<float>(i,j) = (i - cy_) / fy_;
        }
    }
}

cv::Mat CompletionModule::xyz2depth(const Eigen::MatrixXf& points_3d) const {
    int points_nums = points_3d.cols();
    cv::Mat depth = cv::Mat::zeros(height_, width_, CV_16U);

    Eigen::MatrixXf points = points_3d.topRows(3); // Only the XYZ values

    // Remap online lidar axes to the calibration convention: [x,y,z] -> [z,-y,x]
    // (matches coordinate_mode 3 style used during calibration).
    Eigen::Matrix3f axis_map;
    axis_map << 0, 0, 1,
                0, -1, 0,
                1, 0, 0;
    points = axis_map * points;

    // lidar to camera
    points = (rotation_matrix_l2c_ * points).colwise() + translation_vector_l2c_;

    Eigen::ArrayXf z = points.row(2).array();
    Eigen::ArrayXf x = (points.row(0).array() / z.transpose() * fx_ + cx_).round();
    Eigen::ArrayXf y = (points.row(1).array() / z.transpose() * fy_ + cy_).round();
    Eigen::Array<bool, Eigen::Dynamic, 1> valid_mask = (z > 0);

    // std::string& depth_path;
    // cv::Mat depth_img;
    // if (!depth_path.empty()) {
    //     depth_img = cv::Mat::ones(height_, width_, CV_8U) * 255;
    // } 

    for (int i = 0; i < points_nums; ++i) {
        if (!valid_mask(i) || x[i] < 0 || x[i] >= width_ || y[i] < 0 || y[i] >= height_) continue;
        int ix = static_cast<int>(x(i));
        int iy = static_cast<int>(y(i));
        depth.at<uint16_t>(iy, ix) = std::round(z[i] * DEPTH_SCALE);
        // if (!depth_path.empty()) {
        //     depth_img.at<uint8_t>(iy, ix) = 0;
        // }
    }

    // if (!depth_path.empty()) {
    //     cv::imwrite(depth_path, depth_img);
    // }

    return depth;
}

// Eigen::MatrixXf CompletionModule::depth2xyz(const cv::Mat& depth, const cv::Mat& image) const {
//     Eigen::MatrixXf depth_mat;
//     cv::cv2eigen(depth, depth_mat);
//     depth_mat /= DEPTH_SCALE;

//     // 创建点云矩阵
//     Eigen::MatrixXf points(6, depth_mat.size());
//     std::atomic<int> idx{0};

//     #pragma omp parallel for collapse(2)
//     for(int i = 0; i < depth_mat.rows(); ++i) {
//         for(int j = 0; j < depth_mat.cols(); ++j) {
//             if(depth_mat(i,j) > MIN_VALID_DEPTH) {
//                 int current_idx = idx++;
        
//                 // 计算3D坐标
//                 points.col(current_idx).head<3>() = Eigen::Vector3f(
//                     (*x_mesh_)(i,j) * depth_mat(i,j),
//                     (*y_mesh_)(i,j) * depth_mat(i,j),
//                     depth_mat(i,j)
//                 );

//                 // 添加颜色信息
//                 if(!image.empty()) {
//                     const auto& color = image.at<cv::Vec3b>(i,j);
//                     points.col(current_idx).tail<3>() = Eigen::Vector3f(
//                         color[2], color[1], color[0]
//                     );
//                 }
//             }
//         }
//     }
//     points.conservativeResize(6,idx);  // 不会重新分配内存

//     // 应用变换
//     points.topRows(3) = (rotation_matrix_ * points.topRows(3)).colwise() + translation_vector_;

//     return points;
// }

pcl::PointCloud<pcl::PointXYZRGB>::Ptr CompletionModule::depth2xyz(const cv::Mat& depth, const cv::Mat& image) const {
    cv::Mat pointcloud_x = x_mesh_.mul(depth);
    cv::Mat pointcloud_y = y_mesh_.mul(depth);

    // 计算有效点数
    int valid_points = cv::countNonZero(depth > MIN_VALID_DEPTH);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points(new pcl::PointCloud<pcl::PointXYZRGB>);
    points->resize(valid_points);

    // 获取数据指针
    const float* depth_ptr = depth.ptr<float>(0);
    const float* x_ptr = pointcloud_x.ptr<float>(0);
    const float* y_ptr = pointcloud_y.ptr<float>(0);
    const cv::Vec3b* rgb_ptr = image.empty() ? nullptr : image.ptr<cv::Vec3b>(0);
    const int total_pixels = depth.rows * depth.cols;

    int idx = 0;
    for(int pos = 0; pos < total_pixels; ++pos) {
        if(depth_ptr[pos] > MIN_VALID_DEPTH) {
            pcl::PointXYZRGB &point = points->at(idx);
            
            // 一次性设置XYZ
            point.x = x_ptr[pos];
            point.y = y_ptr[pos];
            point.z = depth_ptr[pos];
            
            // 设置颜色
            if (!rgb_ptr) {
                point.r = point.g = point.b = 255;
            } else {
                const cv::Vec3b& rgb = rgb_ptr[pos];
                point.r = rgb[2];
                point.g = rgb[1];
                point.b = rgb[0];
            }
            idx++;
        }
    }

    // Apply camera->lidar (calibration) transform first
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud(*points, *transformed_cloud, transform_);

    // Convert from calibration lidar axes back to online lidar axes: [z,-y,x] -> [x,y,z].
    for (auto &pt : transformed_cloud->points) {
        const float old_x = pt.x;
        const float old_y = pt.y;
        const float old_z = pt.z;
        pt.x = old_z;
        pt.y = -old_y;
        pt.z = old_x;
    }

    return transformed_cloud;
}

void CompletionModule::depthConstraintLayer(cv::Mat& depth, const cv::Mat& lidar, 
                                          const cv::Mat& kernel) const {
    // Dilation of the lidar data to fill gaps
    cv::Mat lidar_max;
    cv::dilate(lidar, lidar_max, kernel);

    // Compute the depth difference and check against the threshold
    // |depth - lidar_max| / lidar_max
    cv::Mat depth_diff;
    cv::absdiff(depth, lidar_max, depth_diff);
    depth_diff.convertTo(depth_diff, CV_32F);  // Convert to float for division
    cv::divide(depth_diff, lidar_max, depth_diff, 1.0, CV_32F);  // Normalized depth difference

    // Create a mask where the lidar has valid values (greater than zero)
    cv::Mat anchor_mask = lidar > 0;

    // Calculate the invalid mask where depth difference is less than threshold or lidar is valid
    cv::Mat invalid_mask;
    cv::threshold(depth_diff, invalid_mask, DEFAULT_THRESHOLD, 255, cv::THRESH_BINARY_INV);
    invalid_mask.convertTo(invalid_mask, CV_8U);
    cv::bitwise_or(invalid_mask, anchor_mask, invalid_mask);

    // Apply the corrections: where invalid_mask is true, keep depth, otherwise use lidar
    lidar_max.copyTo(depth, ~invalid_mask);
}

void CompletionModule::process(cv::Mat& depth, const Eigen::MatrixXf& points) {
    // depth is expected as CV_32F in meters
    cv::Mat lidar = depth.clone();
    
    cv::Mat valid_mask = depth > MIN_VALID_DEPTH;
    cv::Mat depth_tmp = MAX_DEPTH - depth;
    depth_tmp.copyTo(depth, valid_mask);
    
    cv::dilate(depth, depth, DIAMOND_KERNEL_5);
    cv::morphologyEx(depth, depth, cv::MORPH_CLOSE, FULL_KERNEL_5);

    valid_mask = depth < MIN_VALID_DEPTH;
    cv::Mat dilated;
    cv::dilate(depth, dilated, FULL_KERNEL_7);
    dilated.copyTo(depth, valid_mask);
    
    // Apply different blurring techniques based on the blur type
    if (BLUR_TYPE == "bilateral") {
        cv::bilateralFilter(depth, depth, 5, 1.5, 2.0);
    } else if (BLUR_TYPE == "gaussian") {
        valid_mask = depth > MIN_VALID_DEPTH;
        cv::Mat blurred;
        cv::GaussianBlur(depth, blurred, cv::Size(5, 5), 0);
        blurred.copyTo(depth, valid_mask);
    }

    valid_mask = depth > MIN_VALID_DEPTH;
    depth_tmp = MAX_DEPTH - depth;
    depth_tmp.copyTo(depth, valid_mask);

    // Timing for depth2xyz and constraint layer
    depthConstraintLayer(depth, lidar, FULL_KERNEL_9);
}

cv::Mat CompletionModule::projectSparseDepth(const Eigen::MatrixXf& points) const {
    cv::Mat sparse_depth_u16 = xyz2depth(points);
    cv::Mat sparse_depth;
    sparse_depth_u16.convertTo(sparse_depth, CV_32F, 1.0 / DEPTH_SCALE);
    return sparse_depth;
}

CompletionModule::ForwardOutput CompletionModule::forward(const Eigen::MatrixXf& points, const cv::Mat& image) {
    // 1. 图像去畸变
    cv::Mat undistorted_image; 
    // cv::undistort(image, undistorted_image, camera_matrix_, dist_coeffs_);  // 耗时
    cv::remap(image, undistorted_image, map1_, map2_, cv::INTER_LINEAR);

    // 2. 点云转深度图
    cv::Mat sparse_depth = projectSparseDepth(points);

    cv::Mat depth = sparse_depth.clone();

    // 3. 算法处理
    process(depth, points);

    // 4. 深度转点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_3d = depth2xyz(depth, undistorted_image);

    return {sparse_depth, depth, points_3d};
}

// std::pair<cv::Mat, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> CompletionModule::forward(const Eigen::MatrixXf& points, const cv::Mat& image) {
//     auto total_start = std::chrono::steady_clock::now();
    
//     // 1. 图像去畸变
//     auto undistort_start = std::chrono::steady_clock::now();
//     cv::Mat undistorted_image; 
//     cv::remap(image, undistorted_image, map1_, map2_, cv::INTER_LINEAR);
//     auto undistort_end = std::chrono::steady_clock::now();
    
//     // 2. 点云转深度图
//     auto xyz2depth_start = std::chrono::steady_clock::now();
//     cv::Mat depth = xyz2depth(points);
//     auto xyz2depth_end = std::chrono::steady_clock::now();
    
//     // 3. 算法处理
//     auto process_start = std::chrono::steady_clock::now();
//     process(depth, points);
//     auto process_end = std::chrono::steady_clock::now();
    
//     // 4. 深度转点云
//     auto depth2xyz_start = std::chrono::steady_clock::now();
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_3d = depth2xyz(depth, undistorted_image);
//     auto depth2xyz_end = std::chrono::steady_clock::now();
    
//     auto total_end = std::chrono::steady_clock::now();

//     // 计算每个步骤的耗时(毫秒)
//     double undistort_time = std::chrono::duration_cast<std::chrono::milliseconds>(undistort_end - undistort_start).count();
//     double xyz2depth_time = std::chrono::duration_cast<std::chrono::milliseconds>(xyz2depth_end - xyz2depth_start).count();
//     double process_time = std::chrono::duration_cast<std::chrono::milliseconds>(process_end - process_start).count();
//     double depth2xyz_time = std::chrono::duration_cast<std::chrono::milliseconds>(depth2xyz_end - depth2xyz_start).count();
//     double total_time = std::chrono::duration_cast<std::chrono::milliseconds>(total_end - total_start).count();

//     std::cout << "\n=== Performance Statistics ===\n"
//               << "图像去畸变时间: " << undistort_time << "ms\n"
//               << "点云转深度图时间: " << xyz2depth_time << "ms\n"
//               << "算法处理时间: " << process_time << "ms\n"
//               << "深度转点云时间: " << depth2xyz_time << "ms\n"
//               << "总耗时: " << total_time << "ms\n"
//               << "========================\n";

//     return {depth, points_3d};
// }


// int main(){
//     std::string pcd_path = "/workspace/code/Guangtong_ws/tmp/2025_01_03/camera_left/1735873618.998106.pcd";
//     std::string image_path = "/workspace/code/Guangtong_ws/tmp/2025_01_03/20250108-093053.jpg";
//     std::string param_path = "/workspace/code/Guangtong_ws/src/innovusion_zed/config/camera_left.json";

//     Eigen::MatrixXf points = loadPCD(pcd_path);
//     cv::Mat img = cv::imread(image_path);

//     CompletionModule completion_module_(param_path);

//     cv::Mat depth;
//     Eigen::MatrixXf points_3d;
//     std::tie(depth, points_3d) = completion_module_.forward(points, img);

//     savePCD("/workspace/code/Guangtong_ws/tmp/points_3d.pcd", points_3d, true);
//     cv::imwrite("/workspace/code/Guangtong_ws/tmp/depth_result.png", depth);

//     printf("Done\n");

// }
