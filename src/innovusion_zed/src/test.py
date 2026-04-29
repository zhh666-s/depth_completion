import numpy as np
import open3d as o3d
import cv2
import json

def load_parameters(param_path):
    with open(param_path, 'r') as f:
        params = json.load(f)
    
    # 内参
    K = np.array(params['undistort_intrinsic']).reshape(3,3)
    
    # 旋转
    R = np.array(params['rotation']).reshape(3,3)
    
    # 平移
    t = np.array(params['translation'])
    
    # 畸变系数
    dist = np.array(params['distortion'])
    
    return K, R, t, dist

def project_lidar_to_image(pcd_path, img_path, K, R, t, dist):
    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points)
    
    img = cv2.imread(img_path)
    
    R_l2c = np.linalg.inv(R)
    t_l2c = -np.matmul(R_l2c, t.reshape(3,1)).ravel()
    points_camera = R_l2c @ points.T + t_l2c[:, None]

    # points_camera = R @ points.T + t.reshape(3,1)
    mask = points_camera[2] > 0
    points_camera = points_camera[:, mask]

    # 去畸变
    h, w = img.shape[:2]
    new_K, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w,h), 0, (w,h))
    img = cv2.undistort(img, K, dist, None, new_K)
    
    points_2d = np.matmul(new_K, points_camera)
    points_2d = points_2d / points_2d[2]
    points_2d = points_2d.T

    # 筛选在图像内的点
    mask = (points_2d[:, 0] >= 0) & (points_2d[:, 0] < img.shape[1]) & \
           (points_2d[:, 1] >= 0) & (points_2d[:, 1] < img.shape[0])
    points_2d = points_2d[mask].astype(np.int32)
    
    for pt in points_2d:
        cv2.circle(img, tuple(pt[:2]), 2, (0,0,255), -1)
    
    return img

def main():
    # 文件路径
    img_path = "tmp/2025_01_03/camera_left/1735873618.998106.png"
    pcd_path = "tmp/2025_01_03/camera_left/1735873618.998106.pcd"
    param_path = "src/innovusion_zed/config/camera_left.json"

    # # 文件路径
    # img_path = "tmp/frame0000.jpg"
    # pcd_path = "tmp/pointcloud_1697011889259245.pcd"
    # param_path = "src/innovusion_zed/config/cam_front.json"
    
    K, R, t, dist = load_parameters(param_path)
    result_img = project_lidar_to_image(pcd_path, img_path, K, R, t, dist)
    cv2.imwrite("result.png", result_img)

    print("Done!")

if __name__ == '__main__':
    main()