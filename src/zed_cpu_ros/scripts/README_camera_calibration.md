# Camera Calibration Scripts

这个目录里有两个 OpenCV 标定脚本：

- `calibrate_intrinsics.py`: 单个相机内参标定。
- `calibrate_extrinsics.py`: 左右双目相机外参标定。

推荐先分别标定左相机、右相机内参，再用左右同步图像对标定双目外参。

## 0. 直接使用 3840x1080 原始拼接图

你的相机原始输出是 `3840x1080` 的左右拼接图，脚本支持直接输入这种原始图。默认切分规则是：

- 原始图左半边: 左目图像，尺寸 `1920x1080`。
- 原始图右半边: 右目图像，尺寸 `1920x1080`。
- 使用 `--raw-glob` 输入原始拼接图后，脚本会自动切图。
- 切出来的图会保存到 `--split-output-dir/left` 和 `--split-output-dir/right`。
- 后续 OpenCV 标定使用切分后的单目图像。

假设原始图在：

```text
/data/calib/raw/0001.png
/data/calib/raw/0002.png
/data/calib/raw/0003.png
```

推荐切图保存目录：

```text
/data/calib/split/left/
/data/calib/split/right/
```

推荐实际流程：

1. 采集一批 `3840x1080` 棋盘格原始拼接图。
2. 使用同一批原始图标定左目内参。
3. 使用同一批原始图标定右目内参。
4. 使用同一批原始图和左右内参标定双目外参。

## 1. 准备标定图片

### 内参标定图片

每个相机单独采集一组棋盘格图片，例如：

```text
/data/calib/left/0001.png
/data/calib/left/0002.png
/data/calib/left/0003.png

/data/calib/right/0001.png
/data/calib/right/0002.png
/data/calib/right/0003.png
```

建议数量：

- 最少 8 张可识别棋盘格图片。
- 推荐 15 到 30 张。
- 棋盘格要出现在画面不同位置、不同角度、不同远近。
- 不要只拍画面中心，否则边缘畸变估计会不稳定。

### 外参标定图片

外参标定需要左右相机同时看到同一块棋盘格，并且左右图片必须一一对应。

例如：

```text
/data/calib/stereo/left/0001.png
/data/calib/stereo/right/0001.png

/data/calib/stereo/left/0002.png
/data/calib/stereo/right/0002.png
```

脚本会按照文件名排序后配对，所以左右目录里的命名顺序必须一致。

## 2. 棋盘格参数怎么填

OpenCV 标定使用的是棋盘格“内角点”数量，不是格子数量。

假设你的棋盘格是：

- 横向 10 个黑白格子。
- 纵向 7 个黑白格子。

那么内角点数量是：

- `--board-cols 9`
- `--board-rows 6`

如果你的棋盘格是横向 9 个格子、纵向 6 个格子，那么应该填：

- `--board-cols 8`
- `--board-rows 5`

`--square-size` 是单个格子的真实边长，单位建议用米。

例如：

- 格子边长 25 mm: `--square-size 0.025`
- 格子边长 30 mm: `--square-size 0.030`
- 格子边长 50 mm: `--square-size 0.050`

这个数会直接影响外参平移量 `translation_vector` 的尺度，所以一定要填真实尺寸。

## 3. 内参标定

### 使用 3840x1080 原始拼接图标定左相机

```bash
python3 src/zed_cpu_ros/scripts/calibrate_intrinsics.py \
  --raw-glob "/data/calib/raw/*.png" \
  --split-output-dir "/data/calib/split" \
  --camera-side left \
  --raw-width 3840 \
  --raw-height 1080 \
  --board-cols 9 \
  --board-rows 6 \
  --square-size 0.025 \
  --camera-name camera/left \
  --output src/zed_cpu_ros/config/left_calibrated.yaml
```

### 使用 3840x1080 原始拼接图标定右相机

```bash
python3 src/zed_cpu_ros/scripts/calibrate_intrinsics.py \
  --raw-glob "/data/calib/raw/*.png" \
  --split-output-dir "/data/calib/split" \
  --camera-side right \
  --raw-width 3840 \
  --raw-height 1080 \
  --board-cols 9 \
  --board-rows 6 \
  --square-size 0.025 \
  --camera-name camera/right \
  --output src/zed_cpu_ros/config/right_calibrated.yaml
```

运行后会同时生成切分图片：

```text
/data/calib/split/left/
/data/calib/split/right/
```

### 左相机内参

如果你已经提前切好了左目图片，也可以使用 `--image-glob`：

```bash
python3 src/zed_cpu_ros/scripts/calibrate_intrinsics.py \
  --image-glob "/data/calib/left/*.png" \
  --board-cols 9 \
  --board-rows 6 \
  --square-size 0.025 \
  --camera-name camera/left \
  --output src/zed_cpu_ros/config/left_calibrated.yaml
```

### 右相机内参

如果你已经提前切好了右目图片，也可以使用 `--image-glob`：

```bash
python3 src/zed_cpu_ros/scripts/calibrate_intrinsics.py \
  --image-glob "/data/calib/right/*.png" \
  --board-cols 9 \
  --board-rows 6 \
  --square-size 0.025 \
  --camera-name camera/right \
  --output src/zed_cpu_ros/config/right_calibrated.yaml
```

### 内参参数说明

| 参数 | 含义 | 实际工程里怎么填 |
| --- | --- | --- |
| `--raw-glob` | 原始 `3840x1080` 左右拼接图路径匹配表达式 | 推荐使用，例如 `"/data/calib/raw/*.png"` |
| `--split-output-dir` | 切分后左右图保存目录 | 例如 `"/data/calib/split"`，会生成 `left/` 和 `right/` |
| `--camera-side` | 内参标定使用哪一半图像 | 左目填 `left`，右目填 `right` |
| `--raw-width` | 原始拼接图期望宽度 | 你的相机填 `3840`，填 `0` 表示不检查 |
| `--raw-height` | 原始拼接图期望高度 | 你的相机填 `1080`，填 `0` 表示不检查 |
| `--image-glob` | 已经切好的单目图片路径匹配表达式 | 不使用 `--raw-glob` 时才需要，例如 `"/data/calib/left/*.png"` |
| `--board-cols` | 棋盘格横向内角点数量 | 按你的标定板填写 |
| `--board-rows` | 棋盘格纵向内角点数量 | 按你的标定板填写 |
| `--square-size` | 棋盘格单格边长，单位米 | 用尺子量真实边长，例如 `0.025` |
| `--camera-name` | 输出 YAML 里的相机名 | 左相机建议 `camera/left`，右相机建议 `camera/right` |
| `--output` | 内参 YAML 输出路径 | 建议放到 `src/zed_cpu_ros/config/` |
| `--show` | 显示角点检测结果 | 调试时加上，批量跑时可以不加 |
| `--fix-k3` | 固定第三个径向畸变参数为 0 | 如果样本少或结果不稳定，可以尝试加上 |
| `--min-valid-images` | 最少有效角点图片数量 | 默认 `8`；只有少量图片试算时才建议临时改小 |

### 内参输出在哪里

由 `--output` 决定。推荐保存到：

```text
src/zed_cpu_ros/config/left_calibrated.yaml
src/zed_cpu_ros/config/right_calibrated.yaml
```

输出文件是 ROS `camera_info_manager` 常用格式，里面包含：

- `camera_matrix`: 相机内参矩阵 K。
- `distortion_coefficients`: 畸变参数 D。
- `rectification_matrix`: 单目标定时默认是单位矩阵。
- `projection_matrix`: 单目标定时由 K 扩展得到。
- `image_width` / `image_height`: 标定图片分辨率。使用 `3840x1080` 原始拼接图时，这里应该是切分后的 `1920x1080`。
- `camera_name`: 相机名称。

## 4. 双目外参标定

建议使用上一步得到的左右内参，再标定外参。

直接输入 `3840x1080` 原始拼接图：

```bash
python3 src/zed_cpu_ros/scripts/calibrate_extrinsics.py \
  --raw-glob "/data/calib/raw/*.png" \
  --split-output-dir "/data/calib/split" \
  --raw-width 3840 \
  --raw-height 1080 \
  --board-cols 9 \
  --board-rows 6 \
  --square-size 0.025 \
  --left-intrinsics src/zed_cpu_ros/config/left_calibrated.yaml \
  --right-intrinsics src/zed_cpu_ros/config/right_calibrated.yaml \
  --output src/zed_cpu_ros/config/stereo_extrinsics.yaml \
  --left-output src/zed_cpu_ros/config/left_stereo.yaml \
  --right-output src/zed_cpu_ros/config/right_stereo.yaml
```

如果你已经提前切好了左右图片，可以不用 `--raw-glob`，改成：

```bash
python3 src/zed_cpu_ros/scripts/calibrate_extrinsics.py \
  --left-glob "/data/calib/split/left/*.png" \
  --right-glob "/data/calib/split/right/*.png" \
  --board-cols 9 \
  --board-rows 6 \
  --square-size 0.025 \
  --left-intrinsics src/zed_cpu_ros/config/left_calibrated.yaml \
  --right-intrinsics src/zed_cpu_ros/config/right_calibrated.yaml \
  --output src/zed_cpu_ros/config/stereo_extrinsics.yaml \
  --left-output src/zed_cpu_ros/config/left_stereo.yaml \
  --right-output src/zed_cpu_ros/config/right_stereo.yaml
```

### 外参参数说明

| 参数 | 含义 | 实际工程里怎么填 |
| --- | --- | --- |
| `--raw-glob` | 原始 `3840x1080` 左右拼接图路径匹配表达式 | 推荐使用，例如 `"/data/calib/raw/*.png"` |
| `--split-output-dir` | 切分后左右图保存目录 | 例如 `"/data/calib/split"` |
| `--raw-width` | 原始拼接图期望宽度 | 你的相机填 `3840`，填 `0` 表示不检查 |
| `--raw-height` | 原始拼接图期望高度 | 你的相机填 `1080`，填 `0` 表示不检查 |
| `--left-glob` | 已经切好的左相机图片路径匹配表达式 | 不使用 `--raw-glob` 时才需要，例如 `"/data/calib/split/left/*.png"` |
| `--right-glob` | 已经切好的右相机图片路径匹配表达式 | 不使用 `--raw-glob` 时才需要，例如 `"/data/calib/split/right/*.png"` |
| `--board-cols` | 棋盘格横向内角点数量 | 必须和内参标定使用的标定板一致 |
| `--board-rows` | 棋盘格纵向内角点数量 | 必须和内参标定使用的标定板一致 |
| `--square-size` | 棋盘格单格边长，单位米 | 必须填真实尺寸 |
| `--left-intrinsics` | 左相机内参 YAML | 填内参脚本输出的左相机 YAML |
| `--right-intrinsics` | 右相机内参 YAML | 填内参脚本输出的右相机 YAML |
| `--output` | 双目外参 YAML 输出路径 | 建议放到 `src/zed_cpu_ros/config/stereo_extrinsics.yaml` |
| `--left-output` | 带双目校正结果的左相机 YAML | 建议放到 `src/zed_cpu_ros/config/left_stereo.yaml` |
| `--right-output` | 带双目校正结果的右相机 YAML | 建议放到 `src/zed_cpu_ros/config/right_stereo.yaml` |
| `--show` | 显示左右角点检测结果 | 调试时加上 |
| `--estimate-intrinsics` | 外参标定时同时优化内参 | 一般不建议加；只有没有可靠内参时再尝试 |
| `--min-valid-pairs` | 最少有效左右图像对数量 | 默认 `8`；只有少量图片试算时才建议临时改小 |

### 外参输出在哪里

由这三个参数决定：

```text
--output       src/zed_cpu_ros/config/stereo_extrinsics.yaml
--left-output  src/zed_cpu_ros/config/left_stereo.yaml
--right-output src/zed_cpu_ros/config/right_stereo.yaml
```

`stereo_extrinsics.yaml` 包含：

- `rotation_matrix`: 从左相机坐标系到右相机坐标系的旋转矩阵 R。
- `translation_vector`: 从左相机坐标系到右相机坐标系的平移向量 T。
- `essential_matrix`: 本质矩阵 E。
- `fundamental_matrix`: 基础矩阵 F。
- `reprojection_error`: OpenCV 返回的 RMS 重投影误差。

注意：`translation_vector` 的单位和 `--square-size` 的单位一致。如果 `--square-size` 用米，那么 `translation_vector` 也是米。

`left_stereo.yaml` 和 `right_stereo.yaml` 包含：

- 原始内参 `camera_matrix`。
- 原始畸变 `distortion_coefficients`。
- 双目校正矩阵 `rectification_matrix`。
- 双目投影矩阵 `projection_matrix`。

这两个文件更适合后续给 stereo rectification 或 ROS image pipeline 使用。

## 5. 结果检查建议

脚本运行结束会打印：

```text
Valid images / Valid pairs
Image size
RMS reprojection error
Saved ...
```

一般建议：

- `Image size` 应该是切分后的单目尺寸。你的原始图是 `3840x1080` 时，这里应该输出 `1920x1080`。
- `Valid images` 或 `Valid pairs` 不要太少，推荐 15 到 30。
- RMS 重投影误差越小越好，通常小于 1 pixel 比较理想。
- 如果 RMS 很大，优先检查棋盘格参数是否填错。
- 如果大量图片提示 `Chessboard not found`，检查棋盘格是否完整、清晰、曝光是否过暗或过亮。
- 外参标定时，左右图片必须是同一时刻或近似同一时刻拍到的同一个棋盘格姿态。

## 6. 最常见需要改的地方

实际工程里通常只需要改下面这些：

```bash
--image-glob
--raw-glob
--split-output-dir
--camera-side
--raw-width
--raw-height
--left-glob
--right-glob
--board-cols
--board-rows
--square-size
--camera-name
--output
--left-intrinsics
--right-intrinsics
--left-output
--right-output
--min-valid-images
--min-valid-pairs
```

如果你想直接改脚本默认值，也可以改两个脚本顶部的这些变量：

```python
DEFAULT_IMAGE_GLOB
DEFAULT_SPLIT_OUTPUT_DIR
DEFAULT_LEFT_GLOB
DEFAULT_RIGHT_GLOB
DEFAULT_BOARD_COLS
DEFAULT_BOARD_ROWS
DEFAULT_SQUARE_SIZE
DEFAULT_CAMERA_NAME
DEFAULT_OUTPUT
DEFAULT_LEFT_OUTPUT
DEFAULT_RIGHT_OUTPUT
```

命令行参数的优先级高于脚本里的默认值。
