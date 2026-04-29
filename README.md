---

# ⚙️ 项目交接文档：雷视一体机点云超分系统

---

## 📌 一、项目背景

本项目基于 ZED 双目相机与图达通 Falcon Prime 激光雷达，构建雷视融合系统，通过传统插值算法实现激光点云的超分辨率处理，提升稠密度与空间连续性，适用于低成本、高精度感知方案中。

---

## 🗂 二、项目结构概览

以下是项目目录结构的简要说明（`tree -L 2`）：

```bash
Guangtong_ws/
├── build/                   # 编译生成文件夹，catkin_make 后自动生成
│   └── ...                 
├── devel/                   # 构建后的开发环境文件夹
│   └── setup.* 脚本         # 用于 source 环境
├── docs/                    # 项目相关文档与驱动安装包
│   ├── Innovusion Falcon-K 产品手册.pdf
│   ├── Innovusion Falcon Prime 用户手册.pdf
│   └── innovusion-ros-noetic-3.102.6-gen1.4.3-x86-public.deb
├── guangtong.sh             # 一键启动脚本
├── README.md                # 简要项目介绍（可拓展完善）
├── src/                     # 核心源代码目录
│   ├── CMakeLists.txt       # 顶层构建脚本（符号链接）
│   ├── innovusion_zed/      # 点云插值补全模块
│   └── zed_cpu_ros/         # ZED 相机 CPU 驱动模块
├── tmp/                     # 临时数据目录（测试/输出）
│   ├── *.pcd                # 点云测试数据
│   ├── *.png / *.txt        # 深度图像及中间结果
│   ├── innovusion_zed/      # 模块备份代码
│   └── zed_cpu_ros/         # 模块备份代码
```

---

### 📁 目录说明详解

| 目录 / 文件名            | 功能说明 |
|--------------------------|----------|
| `build/`                 | 编译产物目录，`catkin_make` 自动生成；不需手动修改。 |
| `devel/`                 | ROS 工作空间开发环境，包含环境配置脚本。 |
| `docs/`                  | 项目参考资料、雷达用户手册与驱动安装包。 |
| `guangtong.sh`           | 启动脚本，一键执行所有核心模块和 RViz 可视化。 |
| `README.md`              | 项目说明文档（推荐更新完善，供后续维护查阅）。 |
| `src/innovusion_zed/`    | 插值超分模块代码，负责点云插值、发布处理结果。 |
| `src/zed_cpu_ros/`       | ZED 相机 CPU 驱动节点，采集图像、发布时间戳。 |
| `tmp/`                   | 测试数据与中间结果文件（非正式数据）。 |

---

## 🧱 三、环境配置

### 1. 系统环境要求

- 操作系统：Ubuntu 20.04
- ROS 版本：ROS Noetic
- 编译器：GCC 9+

### 2. 安装依赖项

请根据各子模块 `package.xml` 自动解析依赖项，执行：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

---

## 🔧 四、工程编译

### 编译命令

```bash
cd Guangtong_ws
catkin_make
source devel/setup.bash
```

---

## 🚀 五、各模块使用说明

---

### 1. 图达通激光雷达驱动模块

**路径**：`src/innovusion_pointcloud`

#### ✅ 驱动安装

安装 `.deb` 驱动包，例如：

```bash
sudo dpkg -i innovusion-ros-noetic-3.102.6-gen1.4.3-x86-public.deb
```

如需其他版本，请联系图达通技术支持。

#### ✅ 时间同步

参考《Innovusion Falcon Prime 激光雷达_用户手册_V1.1_CN_public_20230406.pdf》完成雷达时间与系统时间同步设置。

#### ✅ 坐标系调整

修改 `launch/innovusion_points.launch` 文件中的参数：

```xml
<arg name="coordinate_mode" default="3" />
```

> 默认输出为 x/y/z，建议设置为 `3` 对应 z/-y/x（需根据标定决定）。

#### ✅ 启动命令

```bash
roslaunch innovusion_pointcloud innovusion_points.launch device_ip:=192.168.1.13 udp_port:=8010 processed:=1
```

---

### 2. ZED 相机 CPU 驱动

**路径**：`src/zed_cpu_ros`

此模块模拟 ZED 相机在 CPU 环境下的运行，主要用于获取图像帧和时间戳。

#### ✅ 配置说明

- 修改 `launch/zed_cpu_ros.launch`，指定：
  - `device_name`：通过 `ls /dev/video*` 获取
  - `frame_rate`：设置帧率
- ZED 标定参数可通过官网查询，或使用校准工具导出 `zed_config`

#### ✅ 启动命令

```bash
roslaunch zed_cpu_ros zed_cpu_ros.launch
```

---

### 3. 点云超分模块（插值补全）

**路径**：`src/innovusion_zed`

此模块实现基于图像信息的稀疏点云插值处理。

#### ✅ 功能说明

- 订阅：
  - `/camera/image`（来自 ZED 相机）
  - `/innov/lidar_points`（来自激光雷达）
- 处理：
  - 使用 `completion_model` 模块进行插值补全
- 发布：
  - 插值后的点云 `/innovusion_zed/completed_points`

#### ✅ 配置说明

- 标定参数路径：`src/innovusion_zed/config`
- 可配置左右相机外参、雷达与相机之间的变换矩阵等

#### ✅ 启动命令

```bash
roslaunch innovusion_zed innovusion_zed.launch
```

---

## 🧩 六、一键启动脚本

脚本：`guangtong.sh`

作用：依次启动雷达、相机、超分模块，并打开 RViz。

建议将该脚本设为可执行：

```bash
chmod +x guangtong.sh
./guangtong.sh
```

---

## 📌 七、使用建议与注意事项

1. **确保各 ROS 节点正常启动**
   ```bash
   rostopic list
   ```
2. **检查设备时间戳差异**，否则会导致时间戳错位，影响点云质量。
3. **RViz 显示问题排查**：
   - 检查 `Fixed Frame` 是否为 `map` 或雷达坐标系
   - 检查超分后点云 Topic 是否正确发布
4. **ZED 相机注意 USB 带宽**，尽量避免使用usb2.0。
5. **completion_model** 模块依赖其他库（如 PCL、OpenCV），若遇构建失败，请检查版本兼容性。

---

## 📂 八、官方文档

  - [ZED SDK](https://www.stereolabs.com/docs/)
  - [Innovusion 雷达支持页面](https://www.innovusion.com/)

