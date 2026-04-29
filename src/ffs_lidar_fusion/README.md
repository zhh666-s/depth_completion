# ffs_lidar_fusion

`ffs_lidar_fusion` 是一个把双目视觉深度和激光雷达深度融合到同一张深度图上的 ROS 功能包。

它目前是一个偏研究/验证性质的第一版流程，核心节点是 `scripts/ffs_lidar_fusion_node.py`。节点会读取 FFS 双目网络输出的视差图、ZED 左右相机的 `CameraInfo`、Innovusion 雷达点云，以及 `innovusion_zed` 里的相机-雷达标定 JSON，然后发布视觉深度、雷达投影深度和融合后的深度图。

## 它在做什么

整体流程如下：

1. 订阅左相机图像和 FFS 视差图，并用近似时间同步。
2. 从左右相机的 `CameraInfo` 中读取焦距和双目 baseline。
3. 用公式 `depth = fx * baseline / disparity` 把 FFS 视差转成视觉深度图。
4. 订阅 Innovusion 雷达点云，把每个雷达点通过 `calib_json` 里的外参转换到左相机坐标系。
5. 用相机内参把雷达点投影到左图像平面，生成一张稀疏的雷达深度图。
6. 融合两张深度图：有雷达深度的位置使用雷达深度，没有雷达深度的位置保留 FFS 视觉深度。
7. 发布可用于算法处理的 `32FC1` 深度图，以及方便在 `rqt_image_view`/RViz 中查看的彩色可视化图。

简单说：FFS 给出稠密但可能不够稳定的视觉深度，雷达给出稀疏但尺度更可靠的深度；本包把雷达深度投到相机图像上，用雷达结果覆盖对应像素，其余区域继续使用视觉深度。

## 需要的数据

启动融合节点前，ROS 系统里至少要有这些数据：

| 数据 | 默认话题/文件 | 类型 | 来源 |
| --- | --- | --- | --- |
| 左相机图像 | `/camera/left/image_raw` | `sensor_msgs/Image`，通常为 `bgr8` | `zed_cpu_ros` 或其他双目相机驱动 |
| 右相机图像 | `/camera/right/image_raw` | `sensor_msgs/Image` | FFS 节点需要，用来生成视差 |
| 左相机内参 | `/camera/left/camera_info` | `sensor_msgs/CameraInfo` | `zed_cpu_ros` |
| 右相机内参 | `/camera/right/camera_info` | `sensor_msgs/CameraInfo` | `zed_cpu_ros` |
| FFS 视差图 | `/ffs/disp` | `sensor_msgs/Image`，编码应为 `32FC1` | `ffs_ros` |
| 雷达点云 | `/iv_points` | `sensor_msgs/PointCloud2`，字段至少包含 `x/y/z` | `innovusion_pointcloud` |
| 雷达-相机标定 | `$(find innovusion_zed)/config/camera_left.json` | JSON 文件 | `innovusion_zed` |

注意：

- 左右相机必须是同一套双目标定，`CameraInfo.P` 或 `CameraInfo.K` 中需要有有效焦距；右相机 `CameraInfo.P[3]` 需要能推导出 baseline。
- `calib_json` 默认使用左相机标定文件 `camera_left.json`。如果你要融合到右相机或前向相机，需要换成对应的 JSON。
- 本节点使用雷达消息的“到达时间”判断新旧，避免雷达设备时间戳和 ROS 时间不同步导致误丢数据。

## 节点依赖

本包自身运行节点：

- `ffs_lidar_fusion_node`：由 `ffs_lidar_fusion_node.py` 启动，负责深度融合。

通常还需要外部节点提供输入：

- `zed_cpu_ros_node`：发布 `/camera/left/image_raw`、`/camera/right/image_raw`、`/camera/left/camera_info`、`/camera/right/camera_info`。
- `ffs_topic_node`：来自 `ffs_ros`，订阅左右图像，发布 `/ffs/disp` 和 `/ffs/vis`。
- Innovusion 点云驱动节点：通常通过 `innovusion_pointcloud` 的 launch 启动，发布 `/iv_points`。

ROS/Python 依赖包括：

- ROS 包：`rospy`、`sensor_msgs`、`std_msgs`、`cv_bridge`、`message_filters`
- Python 库：`numpy`、`opencv-python`/`cv2`
- 工程内功能包：`ffs_ros`、`innovusion_zed`、`zed_cpu_ros`

## 启动前检查

先确认工作空间已经编译并 source：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

如果你的工作空间路径不是 `~/catkin_ws`，请换成实际路径。

确认相机、雷达、FFS 的输入是否存在：

```bash
rostopic list
rostopic hz /camera/left/image_raw
rostopic hz /camera/right/image_raw
rostopic hz /camera/left/camera_info
rostopic hz /camera/right/camera_info
rostopic hz /iv_points
rostopic hz /ffs/disp
```

如果只启动融合节点，`/ffs/disp` 必须已经由其他终端里的 `ffs_ros` 发布。如果使用本包的 bringup 启动方式，`ffs_topic_node` 会一起启动。

## 启动方式一：只启动融合节点

适合你已经单独启动了相机、雷达和 FFS 视差节点的情况：

```bash
roslaunch ffs_lidar_fusion ffs_lidar_fusion.launch
```

常见自定义例子：

```bash
roslaunch ffs_lidar_fusion ffs_lidar_fusion.launch \
  left_topic:=/camera/left/image_raw \
  right_topic:=/camera/right/image_raw \
  left_info_topic:=/camera/left/camera_info \
  right_info_topic:=/camera/right/camera_info \
  lidar_topic:=/iv_points \
  ffs_disp_topic:=/ffs/disp \
  calib_json:=$(rospack find innovusion_zed)/config/camera_left.json \
  publish_debug_overlay:=true
```

### `ffs_lidar_fusion.launch` 参数说明

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `left_topic` | `/camera/left/image_raw` | 左相机图像。融合结果的图像尺寸、header、调试叠加图都以它为基准。 |
| `right_topic` | `/camera/right/image_raw` | 右相机图像。当前融合节点本身不直接使用它，但保留该参数用于和双目输入配置保持一致。 |
| `left_info_topic` | `/camera/left/camera_info` | 左相机内参和投影矩阵，用来计算视差转深度的焦距，也可作为雷达投影内参的备用来源。 |
| `right_info_topic` | `/camera/right/camera_info` | 右相机投影矩阵，用来计算双目 baseline。 |
| `lidar_topic` | `/iv_points` | Innovusion 雷达点云输入。 |
| `ffs_disp_topic` | `/ffs/disp` | FFS 输出的视差图输入，节点按 `32FC1` 读取。 |
| `calib_json` | `$(find innovusion_zed)/config/camera_left.json` | 雷达和左相机之间的标定文件，包含旋转、平移和相机内参。 |
| `visual_depth_topic` | `/ffs_lidar_fusion/visual_depth` | 发布由 FFS 视差换算得到的视觉深度图，编码 `32FC1`，单位米。 |
| `lidar_depth_topic` | `/ffs_lidar_fusion/lidar_depth` | 发布雷达点云投影到图像后的稀疏深度图，编码 `32FC1`，单位米。 |
| `fused_depth_topic` | `/ffs_lidar_fusion/fused_depth` | 发布融合深度图，编码 `32FC1`，单位米。 |
| `fused_depth_viz_topic` | `/ffs_lidar_fusion/fused_depth_viz` | 发布融合深度的彩色可视化图，编码 `bgr8`。 |
| `debug_overlay_topic` | `/ffs_lidar_fusion/debug_overlay` | 发布叠加在左图上的调试图，绿色像素表示该位置有雷达深度支撑。 |
| `sync_slop` | `0.30` | 左图和 FFS 视差图的近似同步容忍时间，单位秒。时间戳差小于该值才会进入融合回调。 |
| `max_lidar_age` | `0.50` | 允许使用的最新雷达点云最大年龄，单位秒。超过后节点会等待更新的雷达数据。 |
| `max_input_age` | `0.35` | 允许使用的 FFS 视差图最大年龄，单位秒。超过后会丢弃该组输入，降低延迟堆积。 |
| `lidar_stride` | `3` | 雷达点采样步长。`3` 表示每 3 个点处理 1 个点；值越小越密，CPU 开销越大。 |
| `publish_debug_overlay` | `false` | 是否发布调试叠加图。调试时建议设为 `true`，长期运行可保持 `false` 节省开销。 |

launch 文件中还固定设置了几个节点内部参数：

| 参数 | 固定值 | 作用 |
| --- | --- | --- |
| `min_disp` | `0.1` | 小于该值的视差视为无效，避免除以很小的数得到异常深度。 |
| `min_depth` | `0.3` | 小于该距离的深度视为无效，单位米。 |
| `max_depth` | `80.0` | 大于该距离的深度视为无效，单位米。 |
| `use_lidar_dilation` | `true` | 是否对投影后的雷达深度做一次膨胀，让稀疏点在图像上更容易被看到/使用。 |
| `lidar_dilation_kernel` | `3` | 雷达深度膨胀核大小。 |

## 启动方式二：启动 FFS 视差节点和融合节点

适合相机和雷达已经启动，但 FFS 还没有启动的情况：

```bash
roslaunch ffs_lidar_fusion ffs_lidar_fusion_bringup.launch
```

这个 launch 会做两件事：

1. 启动 `ffs_ros` 的 `ffs_topic_node`，从左右相机图像生成 `/ffs/disp`。
2. include `ffs_lidar_fusion.launch`，启动融合节点。

常见自定义例子：

```bash
roslaunch ffs_lidar_fusion ffs_lidar_fusion_bringup.launch \
  left_topic:=/camera/left/image_raw \
  right_topic:=/camera/right/image_raw \
  left_info_topic:=/camera/left/camera_info \
  right_info_topic:=/camera/right/camera_info \
  lidar_topic:=/iv_points \
  calib_json:=$(rospack find innovusion_zed)/config/camera_left.json
```

### `ffs_lidar_fusion_bringup.launch` 参数说明

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `left_topic` | `/camera/left/image_raw` | 同时传给 FFS 节点和融合节点，表示左相机图像输入。 |
| `right_topic` | `/camera/right/image_raw` | 传给 FFS 节点，表示右相机图像输入。 |
| `left_info_topic` | `/camera/left/camera_info` | 传给融合节点，提供左相机内参。 |
| `right_info_topic` | `/camera/right/camera_info` | 传给融合节点，提供右相机投影矩阵和 baseline。 |
| `lidar_topic` | `/iv_points` | 传给融合节点，表示雷达点云输入。 |
| `calib_json` | `$(find innovusion_zed)/config/camera_left.json` | 传给融合节点，表示雷达-左相机标定文件。 |

bringup 中的 FFS 节点使用这些固定参数：

| FFS 参数 | 固定值 | 作用 |
| --- | --- | --- |
| `disp_topic` | `/ffs/disp` | FFS 视差输出，随后作为融合节点输入。 |
| `vis_topic` | `/ffs/vis` | FFS 自带可视化输出。 |
| `model_dir` | `$(find ffs_ros)/ffs_runtime/weights/20-26-39/model_best_bp2_serialize.pth` | FFS 模型权重路径。 |
| `valid_iters` | `8` | FFS 推理迭代次数。 |
| `max_disp` | `192` | FFS 最大视差范围。 |
| `hiera` | `0` | FFS 模型内部配置。 |
| `skip` | `0` | FFS 跳帧设置。 |
| `show_local` | `false` | 不在本地 OpenCV 窗口显示图像。 |
| `display_scale` | `0.8` | 本地显示缩放比例，仅在 `show_local=true` 时有意义。 |
| `sync_slop` | `0.03` | FFS 节点同步左右相机图像的时间容忍，单位秒。 |

## 相机和雷达如何启动

本包的 launch 不会启动相机和雷达硬件驱动。你需要提前启动它们，常见方式如下：

只启动 ZED CPU 相机：

```bash
roslaunch zed_cpu_ros zed_cpu_ros.launch
```

启动 Innovusion 雷达、ZED 相机和 `innovusion_zed` 原有点云着色/处理流程：

```bash
roslaunch innovusion_zed system_bringup.launch
```

如果你只想用 `system_bringup.launch` 提供相机和雷达，不想启动它原来的 `innovusion_zed_node` 和 RViz，可以这样：

```bash
roslaunch innovusion_zed system_bringup.launch start_fusion:=false start_rviz:=false
```

然后再另开终端启动本包：

```bash
roslaunch ffs_lidar_fusion ffs_lidar_fusion_bringup.launch
```

## 启动后的结果

启动成功后，可以看到这些输出话题：

```bash
rostopic list | grep ffs_lidar_fusion
```

默认输出：

| 话题 | 类型/编码 | 内容 |
| --- | --- | --- |
| `/ffs_lidar_fusion/visual_depth` | `sensor_msgs/Image`，`32FC1` | FFS 视差换算得到的稠密视觉深度，单位米。 |
| `/ffs_lidar_fusion/lidar_depth` | `sensor_msgs/Image`，`32FC1` | 雷达点云投影到左图后的深度，通常较稀疏，单位米。 |
| `/ffs_lidar_fusion/fused_depth` | `sensor_msgs/Image`，`32FC1` | 融合深度。有雷达的位置使用雷达深度，其他位置使用视觉深度，单位米。 |
| `/ffs_lidar_fusion/fused_depth_viz` | `sensor_msgs/Image`，`bgr8` | 融合深度的彩色显示图。近处更亮/颜色更暖，远处更暗/颜色更冷，具体颜色来自 OpenCV TURBO colormap。 |
| `/ffs_lidar_fusion/debug_overlay` | `sensor_msgs/Image`，`bgr8` | 调试叠加图。需要 `publish_debug_overlay:=true` 才会发布；绿色表示有雷达深度支撑的位置。 |

查看可视化结果：

```bash
rqt_image_view /ffs_lidar_fusion/fused_depth_viz
```

查看调试叠加图：

```bash
roslaunch ffs_lidar_fusion ffs_lidar_fusion.launch publish_debug_overlay:=true
rqt_image_view /ffs_lidar_fusion/debug_overlay
```

如果你要在代码里使用融合深度，建议订阅 `/ffs_lidar_fusion/fused_depth`，它是 `32FC1` 浮点深度图，每个像素单位是米，`0.0` 表示无效深度。

## 常见问题

### 一直提示 `Waiting for left/right camera info`

说明没有收到 `/camera/left/camera_info` 或 `/camera/right/camera_info`。检查相机驱动是否启动，或者 launch 参数里的 `left_info_topic`、`right_info_topic` 是否和实际话题一致。

### 一直提示 `Waiting for lidar input`

说明没有收到雷达点云。检查 `/iv_points` 是否存在，或者把 `lidar_topic` 改成实际点云话题。

### 提示 `Lidar data too old`

说明最新雷达点云距离当前时间超过了 `max_lidar_age`。可能是雷达频率太低、节点卡顿、点云驱动停止，或者 `max_lidar_age` 设置太小。可以先用：

```bash
rostopic hz /iv_points
```

确认雷达频率，再适当调大 `max_lidar_age`。

### 提示 `Drop stale fusion pair`

说明 FFS 视差图太旧，超过 `max_input_age`。常见原因是 FFS 推理速度低于相机输入速度。可以降低相机帧率、优化 FFS 推理，或适当调大 `max_input_age`。

### 融合图没有深度，或者深度全是 0

优先检查这些点：

- `/ffs/disp` 是否真的在发布 `32FC1` 视差图。
- 左右 `CameraInfo` 是否有效，尤其是右相机 `P[3]` 是否能推导出 baseline。
- `min_disp`、`min_depth`、`max_depth` 是否把数据过滤掉了。
- `calib_json` 是否对应当前相机和雷达安装位置。

### 雷达点投影位置明显不对

通常是标定文件或相机话题不匹配。默认 `calib_json` 是 `camera_left.json`，因此默认融合到 `/camera/left/image_raw`。如果你换了相机、分辨率、裁剪方式或使用了右相机图像，需要同步更新标定文件和话题配置。

## 开发建议

当前融合规则很保守：雷达有值就覆盖，雷达无值就用视觉深度。后续可以继续改进：

- 引入 FFS 置信度图，根据置信度和雷达距离做加权融合。
- 统一图像去畸变流程，保证雷达投影和图像使用完全相同的相机模型。
- 把融合深度反投影成带颜色的点云。
- 加入时间滤波，让连续帧结果更稳定。
