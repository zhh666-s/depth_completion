# collector

`collector` 是独立的数据采集功能包，用来按“雷达型号 + 相机型号”管理不同硬件组合的数据采集节点。

当前已有采集器：

| 硬件组合 | 节点源码 | launch |
| --- | --- | --- |
| Falcon(s) 雷达 + ZED 相机 | `src/falcons_zed_collector_node.cpp` | `launch/falcons_zed_collector.launch` |

## falcons_zed_collector

`falcons_zed_collector_node` 迁移自 `innovusion_zed/src/true_data_collector_node.cpp`，用于 Falcon(s) 雷达和 ZED 相机采集，保留原有功能：

- 订阅雷达 `sensor_msgs/PointCloud2`，默认 `/iv_points`
- 订阅相机 `sensor_msgs/Image` 或 `sensor_msgs/CompressedImage`
- 手动按 `s` 或空格保存一组数据，按 `q` 或 Esc 退出
- 保存左图、深度图、原始点云
- 兼容 `intensity`、`reflectivity`、`reflectance`、`intensities`、`i` 等点云强度字段
- 使用 `rospy/roscpp` 收到的消息 header stamp 写入文件名和日志

## 输出结构

默认输出目录是：

```bash
$(find collector)/data/falcons_zed
```

保存后会生成：

```text
data/falcons_zed/
├── left/
│   └── sample_0000_<cloud_stamp>.png
├── depth/
│   └── sample_0000_<cloud_stamp>.png
└── pcd/
    └── sample_0000_<cloud_stamp>.pcd
```

深度图为 `16UC1` PNG，默认 `depth_scale=1000.0`，即毫米单位保存。

## 启动

```bash
cd /workspace/code/Guangtong_ws
catkin_make
source devel/setup.bash
roslaunch collector falcons_zed_collector.launch
```

如果在无显示器或 SSH 环境运行，关闭 OpenCV 预览窗口：

```bash
roslaunch collector falcons_zed_collector.launch preview_window:=false
```

## 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `lidar_topic` | `/iv_points` | 雷达点云输入。 |
| `cam_topic` | `/camera/left/image_raw` | 相机图像输入。以 `/compressed` 结尾时按压缩图像解码。 |
| `param_path` | `$(find innovusion_zed)/config/camera_left.json` | 雷达到相机投影使用的标定 JSON。 |
| `output_dir` | `$(find collector)/data/falcons_zed` | 采集数据保存目录。 |
| `depth_scale` | `1000.0` | 米到 `uint16` 深度图数值的缩放系数。 |
| `preview_window` | `true` | 是否显示 OpenCV 预览窗口并接受键盘保存。 |

## 兼容旧启动方式

旧的：

```bash
roslaunch innovusion_zed true_data_collector.launch
```

已经转发到 `collector/launch/falcons_zed_collector.launch`，参数名保持一致。
