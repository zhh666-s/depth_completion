# LiDAR-Camera Calibration Scripts

这里放的是 `innovusion_zed` 里用于雷达-相机标定相关的辅助脚本。

## 1. 自动标定

脚本：

```bash
src/innovusion_zed/scripts/calibration/auto_calibrate_lidar_camera_checkerboard.py
```

当前方法是基于黑白棋盘格的自动检测：

- 图像中检测棋盘格角点；
- 用相机内参求棋盘格在相机坐标系下的平面；
- 在对应 PCD 点云中拟合标定板平面；
- 对比已有可靠标定文件，输出若干候选外参；
- 推荐优先看 `normal_align_t_prior_5cm` 这一类结果，它会约束平移不要偏离原始标定太多。

示例启动命令：

```bash
cd /workspace/code/Guangtong_ws

python3 src/innovusion_zed/scripts/calibration/auto_calibrate_lidar_camera_checkerboard.py \
  --data-dir src/innovusion_zed/data/true_data_3 \
  --config src/innovusion_zed/config/camera_left.json \
  --out-dir src/innovusion_zed/auto_calib_eval_true_data_3 \
  --pattern-cols 9 \
  --pattern-rows 6 \
  --square-size 0.13
```

参数说明：

- `--data-dir`：采集数据目录，目录下需要有 `left/` 和 `pcd/` 两个子目录。
- `--config`：已有的相机-雷达标定文件，一般用比较可靠的 `camera_left.json`。
- `--out-dir`：候选标定结果输出目录。
- `--pattern-cols`：棋盘格内角点列数，这里是 `9`。
- `--pattern-rows`：棋盘格内角点行数，这里是 `6`。
- `--square-size`：棋盘格单格边长，单位是米，这里 `13cm` 写成 `0.13`。

脚本运行后会打印每个候选结果的指标，例如：

- `plane_dist_mean`：相机检测平面和雷达拟合平面的平均距离误差，越小越好。
- `plane_ang_mean`：两个平面法向的平均角度误差，越小越好。
- `delta_t`：相对原始 `camera_left.json` 的平移变化量。
- `delta_R`：相对原始 `camera_left.json` 的旋转变化量。

当前整理后的推荐结果放在：

```bash
src/innovusion_zed/config/camera_left_auto_calibrated.json
```

原始可靠标定文件仍然保留在：

```bash
src/innovusion_zed/config/camera_left.json
```

## 2. 导出 rosbag

脚本：

```bash
src/innovusion_zed/scripts/calibration/export_true_data_to_lidar_camera_bag.py
```

这个脚本用于把 `true_data` 格式的数据导出成 rosbag，主要是为了兼容一些 ROS 标定工具。

示例命令：

```bash
cd /workspace/code/Guangtong_ws

python3 src/innovusion_zed/scripts/calibration/export_true_data_to_lidar_camera_bag.py \
  --data-dir src/innovusion_zed/data/true_data_3 \
  --config src/innovusion_zed/config/camera_left.json \
  --output src/innovusion_zed/data/true_data_3_calib.bag
```

导出的 bag 里默认包含：

- `/sensors/camera/image_color`
- `/sensors/camera/camera_info`
- `/sensors/velodyne_points`

如果要给其他标定包使用，可以根据目标工具的 topic 要求修改：

```bash
--image-topic
--camera-info-topic
--points-topic
--camera-frame
--lidar-frame
```

