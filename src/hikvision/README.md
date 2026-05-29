# hikvision

`hikvision` 是一个 ROS1/Noetic 功能包，用 OpenCV 通过 RTSP 读取 Hikvision 网络相机，并发布 `sensor_msgs/Image`。

当前默认配置匹配这台相机：

- 型号：`DS-2XC64ZZY-SHZJ`
- 相机 IP：`192.168.1.64`
- 电脑网卡：`lidar0`，本机 IP `192.168.1.100/24`
- 主码流：`rtsp://admin:hyzx_hyzx@192.168.1.64:554/Streaming/Channels/101`
- 子码流：`rtsp://admin:hyzx_hyzx@192.168.1.64:554/Streaming/Channels/102`
- 默认 ROS 图像话题：`/hikvison/image_raw`

注意：这里默认话题沿用当前需求里的拼写 `hikvison`。如果希望使用标准拼写或通用相机话题，可以在启动时传入 `image_topic:=/hikvision/image_raw` 或 `image_topic:=/camera/image_raw`。

## 启动

编译并 source 工作空间：

```bash
cd /workspace/code/Guangtong_ws
catkin_make
source devel/setup.bash
```

启动主码流：

```bash
roslaunch hikvision hikvision.launch
```

启动子码流：

```bash
roslaunch hikvision hikvision.launch stream_channel:=102
```

改成发布到通用话题：

```bash
roslaunch hikvision hikvision.launch image_topic:=/camera/image_raw
```

## 检查

先确认网络链路：

```bash
ip addr show lidar0
ip route get 192.168.1.64
ping -c 3 192.168.1.64
```

再确认 ROS 图像发布：

```bash
rostopic list | grep hikvison
rostopic hz /hikvison/image_raw
rqt_image_view /hikvison/image_raw
```

也可以继续使用已经验证过的 ffmpeg 抓帧命令：

```bash
ffmpeg -y -rtsp_transport tcp \
  -i "rtsp://admin:hyzx_hyzx@192.168.1.64:554/Streaming/Channels/101" \
  -frames:v 1 /workspace/data/hikvison/test/frame.jpg
```

## 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `camera_ip` | `192.168.1.64` | 相机 IP。 |
| `username` | `admin` | RTSP 用户名。 |
| `password` | `hyzx_hyzx` | RTSP 密码。 |
| `stream_channel` | `101` | Hikvision 通道，`101` 主码流，`102` 子码流。 |
| `rtsp_url` | 由上面参数拼出 | 完整 RTSP 地址，可直接覆盖。 |
| `image_topic` | `/hikvison/image_raw` | 发布的 `sensor_msgs/Image` 话题。 |
| `frame_id` | `hikvision_camera` | 图像 header 的坐标系 ID。 |
| `encoding` | `bgr8` | `cv_bridge` 输出编码。OpenCV 默认读到 BGR 图像。 |
| `capture_backend` | `ffmpeg` | OpenCV 后端，可改为 `gstreamer` 或 `any`。 |
| `ffmpeg_capture_options` | `rtsp_transport;tcp` | 传给 OpenCV FFmpeg 后端的选项，默认强制 RTSP over TCP。 |
| `target_fps` | `0.0` | 发布帧率限制。`0.0` 表示尽量发布每一帧。 |
| `reconnect_delay` | `2.0` | RTSP 断流后的重连间隔，单位秒。 |
| `read_fail_limit` | `25` | 连续读帧失败多少次后重连。 |
| `show_local` | `false` | 是否弹出本地 OpenCV 显示窗口。 |

## 时间戳

当前链路没有 PTP 硬件时钟，且相机能力里只看到 NTP/manual。节点按电脑收到并发布帧时的 `rospy.Time.now()` 写入 `Image.header.stamp`。

## camera_info

本包当前只发布图像。相机内参和 `camera_info` 可以后续用单独标定文件或 `camera_info_manager` 增加。
