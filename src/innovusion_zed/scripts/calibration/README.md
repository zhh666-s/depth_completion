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

如果要把 `true_data_2` 和 `true_data_3` 作为同一套传感器采集的数据一起参与标定，可以一次传入多个数据目录：

```bash
cd /workspace/code/Guangtong_ws

python3 src/innovusion_zed/scripts/calibration/auto_calibrate_lidar_camera_checkerboard.py \
  --data-dir src/innovusion_zed/data/true_data_2 src/innovusion_zed/data/true_data_3 \
  --config src/innovusion_zed/config/camera_left.json \
  --out-dir src/innovusion_zed/auto_calib_eval_true_data_2_3 \
  --pattern-cols 9 \
  --pattern-rows 6 \
  --square-size 0.13
```

本次合并数据生成的推荐标定文件放在：

```bash
src/innovusion_zed/config/camera_left_auto_calibrated_true_data_2_3.json
```

参数说明：

- `--data-dir`：采集数据目录，目录下需要有 `left/` 和 `pcd/` 两个子目录。
  可以传入一个或多个目录；多个目录会合并成一批样本共同求一个外参。
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

## 2. 投影效果检查

脚本：

```bash
src/innovusion_zed/scripts/calibration/project_lidar_depth_overlay.py
```

这个脚本用于在已经得到外参以后，把每帧 PCD 按标定参数转换到相机坐标系，生成 z-buffer 深度图，并把深度颜色投射叠加到左相机图像上。输出结果可用于直观看新外参下点云深度和图像内容的贴合程度。

示例命令：

```bash
cd /workspace/code/Guangtong_ws

python3 src/innovusion_zed/scripts/calibration/project_lidar_depth_overlay.py \
  --data-dir src/innovusion_zed/data/true_data_2 src/innovusion_zed/data/true_data_3 \
  --config src/innovusion_zed/config/camera_left_auto_calibrated_true_data_2_3.json \
  --out-dir src/innovusion_zed/data/remap \
  --min-depth 0.2 \
  --max-depth 80 \
  --color-mode auto \
  --point-size 2 \
  --alpha 0.55 \
  --board-view \
  --write-depth-png
```

输出目录：

- `overlay/`：深度图投射到原始左图后的叠加图，主要看拟合效果。
- `depth_color/`：只包含投影深度的伪彩图。
- `depth_mm/`：可选的 16-bit 深度图，单位为毫米，需要传 `--write-depth-png`。
- `board_overlay/`：只关注标定板区域的诊断图，画出图像检测到的棋盘格角点网格，并叠加标定板平面附近的 LiDAR 点。
- `board_crop/`：标定板区域放大裁剪图，适合观察投影点相对黑白方块边界的偏移。
- `projection_summary.json`：每帧投影点数量和深度范围统计。

常用参数：

- `--config`：要评估的标定 JSON，可以换成原始 `camera_left.json` 或其他候选结果做对比。
- `--out-dir`：输出目录；不指定时默认写到 `src/innovusion_zed/data/remap`。
- `--max-samples`：只输出前 N 帧，调试时可以先设成 `5`。
- `--use-undistort`：先对图像去畸变，再用 `undistort_intrinsic` 投影。
- `--color-mode auto`：默认按每帧有效深度的 2%-98% 分位自动拉伸颜色，这样近远深度差异更明显。
- `--color-mode fixed`：按 `--min-depth` 和 `--max-depth` 固定范围上色，适合不同帧之间做绝对深度颜色对比。
- `--board-view`：额外输出标定板诊断图。脚本会检测图像棋盘格，并筛选投影后位于棋盘格附近、且距离棋盘格平面较近的 LiDAR 点。
- `--board-color auto`：标定板诊断图默认优先用 PCD intensity 上色；如果当前 PCD 的 intensity 全为 0，则自动改用点到棋盘格平面的有符号距离上色。
- `--board-color intensity`：强制按 PCD intensity 上色。当前 `true_data_2/3` 的 intensity 字段存在但数值全为 0，所以这个模式在这批数据上不会显示黑白格反射差异。
- `--board-color plane_error`：按点到棋盘格平面的有符号距离上色，适合看点云在标定板处的前后漂移。
- `--board-plane-band`：标定板平面附近点的筛选厚度，默认 `0.12m`。
- `--point-size`：叠加点半径；点云太稀疏时可以设大一点。
- `--alpha`：深度颜色叠加透明度。

## 3. 导出 rosbag

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
