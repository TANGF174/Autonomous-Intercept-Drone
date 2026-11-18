# Gazebo 相机与视觉 PNG 频率修改说明

## 1. 修改目的

本次修改的目标是提高视觉制导链路中新图像和比例导引计算的更新频率，并降低全高清图像对 Gazebo、`ros_gz_bridge`、DDS 和视觉检测节点造成的传输与计算压力。

修改前的主要配置为：

- IMX214 彩色相机：`1920 × 1080 @ 30 Hz`
- ROS 侧 `/camera/image`：受全高清图像桥接性能影响
- 视觉 PNG 定时器：`25 ms`，实际为 `40 Hz`
- PNG 使用的相机内参：按照 `1920 × 1080` 和水平视场角 `1.204 rad` 配置

修改后的目标配置为：

- IMX214 彩色相机：`1280 × 720 @ 60 Hz`
- PNG 相机参数：与新的分辨率保持一致
- 视觉 PNG 计算：约 `60 Hz`

---

## 2. 整体数据链路

视觉制导链路可以简化为：

```text
Gazebo IMX214 相机
        ↓ 生成图像
ros_gz_bridge
        ↓ /camera/image
uav_vision_dectect
        ↓ 检测框 x、y、width、height
/camera_detect_result
        ↓
uav_vision_png
        ↓ 像素坐标转换为 LOS 方向
PNG 制导计算
        ↓
PX4 速度与偏航指令
```

本次修改涉及其中两个环节：

1. Gazebo 相机图像的生成频率和分辨率；
2. `uav_vision_png` 的相机模型和 PNG 计算频率。

---

## 3. 修改一：IMX214 相机帧率改为 60 Hz

### 修改文件

```text
/home/ubuntu22/PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite/model.sdf
```

### 修改内容

```xml
<sensor name="IMX214" type="camera">
  ...
  <update_rate>60</update_rate>
</sensor>
```

### 参数含义

`update_rate` 表示 Gazebo 在每秒仿真时间内生成相机图像的目标次数：

```text
30 → 每秒最多生成 30 张图像
60 → 每秒最多生成 60 张图像
```

该参数控制的是 IMX214 彩色相机，不是下面的 `StereoOV7251` 深度相机。当前视觉检测桥接的话题是：

```text
/world/grass_world/model/x500_depth_1/link/camera_link/sensor/IMX214/image
```

因此只需要提高 IMX214 的频率，深度相机仍可保持 30 Hz，避免增加无用的深度渲染负担。

### 为什么不需要编译

`.sdf` 是 Gazebo 模型描述文件。Gazebo 在启动并生成模型时读取该文件，不会把它编译进 PX4 可执行程序。

```text
model.sdf
    ↓ Gazebo 启动时解析
创建相机传感器
```

因此修改后不需要编译 PX4，但必须彻底重启 Gazebo，使其重新读取 SDF 并重新创建相机传感器。

---

## 4. 修改二：IMX214 分辨率改为 1280 × 720

### 修改文件

与帧率相同：

```text
/home/ubuntu22/PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite/model.sdf
```

### 修改内容

```xml
<sensor name="IMX214" type="camera">
  <camera>
    <horizontal_fov>1.204</horizontal_fov>
    <image>
      <width>1280</width>
      <height>720</height>
    </image>
  </camera>
  <update_rate>60</update_rate>
</sensor>
```

本次保持水平视场角不变：

```xml
<horizontal_fov>1.204</horizontal_fov>
```

文件中还存在深度相机的：

```xml
<horizontal_fov>1.274</horizontal_fov>
```

`1.274` 属于 `StereoOV7251` 深度相机，当前视觉 PNG 使用的是 IMX214，因此不能用深度相机的视场角计算 PNG 焦距。

### 降低分辨率的作用

单张 RGB 图像的原始数据量近似为：

```text
宽度 × 高度 × 3 字节
```

在 60 Hz 下：

```text
1920 × 1080 × 3 × 60 ≈ 373 MB/s
1280 ×  720 × 3 × 60 ≈ 166 MB/s
```

降低分辨率能够明显减轻以下环节的负担：

- Gazebo 相机渲染；
- `ros_gz_bridge` 图像转换；
- ROS 2 DDS 序列化和传输；
- OpenCV 图像复制；
- YOLO 和 LightTrack 的预处理。

### 为什么不需要编译

分辨率仍然属于 SDF 运行时配置，Gazebo 重启后重新读取即可，无需重新编译 PX4。

---

## 5. 修改三：视觉 PNG 相机参数同步到 1280 × 720

### 修改文件

```text
/home/ubuntu22/Autonomous-Intercept-Drone/uav_vision_png/config/params.yaml
```

### 修改内容

```yaml
focal_length: 931.5
image_width:  1280
image_height: 720
```

### 为什么需要同步修改

`uav_vision_dectect` 输出的是目标检测框的像素坐标，例如：

```text
x、y、width、height
```

`uav_vision_png` 需要把检测框中心转换为相机视线 LOS：

```cpp
ex = bbox_center_x - image_width / 2;
ey = bbox_center_y - image_height / 2;

Eigen::Vector3d Nt(ex, ey, focal_length);
```

因此 PNG 必须知道：

- 图像宽度；
- 图像高度；
- 以像素为单位的焦距。

如果 Gazebo 已经输出 `1280 × 720` 图像，但 PNG 仍按照 `1920 × 1080` 计算，图像中心和 LOS 方向都会错误。

### 焦距为什么变成 931.5

水平视场角保持 `1.204 rad` 不变时：

```text
f = (图像宽度 / 2) / tan(水平视场角 / 2)
```

也可以直接按照宽度比例缩放：

```text
f_new = 1397.2 × 1280 / 1920
      ≈ 931.5 px
```

因此新的参数为：

```yaml
focal_length: 931.5
image_width: 1280
image_height: 720
```

### 为什么参数位于 PNG 包而不是检测包

检测包只负责从图像中找出目标的像素矩形框。它可以通过 OpenCV 的：

```cpp
frame.cols
frame.rows
```

直接获得实际图像尺寸。

PNG 包需要将二维像素位置转换为空间 LOS 方向，因此必须使用图像中心和焦距。这里的参数是制导几何参数，而不是 YOLO 网络输入参数。

检测代码中的：

```cpp
config.inputWidth = 640;
config.inputHeight = 640;
```

表示 YOLO 网络的推理输入尺寸，不需要跟随 Gazebo 相机分辨率修改。

### 为什么不需要编译

`params.yaml` 是 ROS 2 节点启动时读取的参数文件，不属于 C++ 二进制程序。

当前工作空间使用 `--symlink-install`，安装目录中的参数文件指向源码目录中的参数文件，因此修改 YAML 后不需要编译，但必须重启 `uav_vision_png` 节点，让节点重新读取参数。

---

## 6. 修改四：PNG 计算频率提高到约 60 Hz

### 修改文件

```text
/home/ubuntu22/Autonomous-Intercept-Drone/uav_vision_png/src/vision_png_control.cpp
```

### 修改前

```cpp
// 原注释写 20 Hz，但 25 ms 实际对应 40 Hz
timer_png_ = create_wall_timer(
    std::chrono::milliseconds(25),
    std::bind(&VisionPNG::png_calculate, this));
```

频率为：

```text
1 / 0.025 s = 40 Hz
```

### 修改后

```cpp
// 60 Hz 视觉 PNG LOS 计算
timer_png_ = create_wall_timer(
    std::chrono::microseconds(16667),
    std::bind(&VisionPNG::png_calculate, this));
```

频率为：

```text
16667 微秒 = 16.667 毫秒 = 0.016667 秒
1 / 0.016667 s ≈ 60 Hz
```

虽然数字 `16667` 看起来比 `25` 大，但两者单位不同：

```text
25 毫秒 = 25000 微秒
16667 微秒 = 16.667 毫秒
```

新的时间间隔实际上更短。

### 为什么这里必须重新编译

定时器周期写在 `.cpp` C++ 源代码中。ROS 2 实际运行的是编译生成的二进制可执行文件，而不是直接解释执行 `.cpp` 文件。

```text
vision_png_control.cpp
        ↓ colcon/CMake 编译
uav_vision_png 可执行文件
        ↓
ROS 2 运行该二进制程序
```

如果只修改 `.cpp` 而不重新编译，安装目录中的可执行文件仍然包含旧的 `25 ms` 定时器，PNG 仍以约 40 Hz 运行。

因此修改后必须执行：

```bash
cd /home/ubuntu22/autonomous_intercept_ws

colcon build --symlink-install \
  --packages-select uav_vision_png

source /home/ubuntu22/autonomous_intercept_ws/install/setup.bash
```

然后重启 `uav_vision_png` 节点。

---

## 7. 为什么有些修改需要编译，有些不需要

| 修改内容 | 文件类型 | 谁读取 | 是否编译 | 必须执行的操作 |
|---|---|---|---|---|
| 相机帧率 60 Hz | `.sdf` | Gazebo 启动时 | 否 | 重启 Gazebo |
| 相机分辨率 1280×720 | `.sdf` | Gazebo 启动时 | 否 | 重启 Gazebo |
| 焦距、图像宽高 | `.yaml` | ROS节点启动时 | 否 | 重启 PNG 节点 |
| PNG 定时器 16667 微秒 | `.cpp` | 编译器生成二进制 | **是** | `colcon build` 后重启节点 |

核心判断方法是：

```text
程序启动时读取的配置文件 → 通常不需要编译，只需重启
C/C++源代码中的逻辑       → 必须重新编译，再重启程序
```

---

## 8. 建议的完整重启与运行顺序

### 8.1 编译 PNG 包

只有修改了 `vision_png_control.cpp` 才需要执行：

```bash
cd /home/ubuntu22/autonomous_intercept_ws

colcon build --symlink-install \
  --packages-select uav_vision_png
```

### 8.2 重新加载环境

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu22/ros2_px4_116_ws/install/local_setup.bash
source /home/ubuntu22/autonomous_intercept_ws/install/setup.bash
```

### 8.3 重启 Gazebo 双机仿真

必须先关闭旧 Gazebo，使新的 SDF 相机配置生效：

```bash
cd /home/ubuntu22/PX4-Autopilot
./run_swarm.sh
```

### 8.4 启动视觉 PNG

```bash
ros2 launch uav_vision_png vision_png.launch.py \
  speed_cmd:=4.0 \
  N_png:=4.0 \
  csv_path:=/home/ubuntu22/Autonomous-Intercept-Drone/outputs/vpng_intercept_stats.csv
```

---

## 9. 修改后的验证方法

### 9.1 验证 Gazebo 原始相机频率

```bash
gz topic -f \
  -t /world/grass_world/model/x500_depth_1/link/camera_link/sensor/IMX214/image
```

预期接近：

```text
60 Hz
```

### 9.2 验证 ROS 图像频率

```bash
ros2 topic hz /camera/image \
  --qos-reliability best_effort \
  --qos-durability volatile
```

预期稳定接近：

```text
60 Hz
```

### 9.3 验证图像分辨率

```bash
ros2 topic echo /camera/image --once --field width
ros2 topic echo /camera/image --once --field height
```

预期分别为：

```text
1280
720
```

### 9.4 验证 PNG 参数

在 `uav_vision_png` 节点启动后执行：

```bash
ros2 param get /uav_vision_png focal_length
ros2 param get /uav_vision_png image_width
ros2 param get /uav_vision_png image_height
```

预期分别为：

```text
931.5
1280
720
```

### 9.5 粗略验证 PNG 输出频率

目标被检测、节点进入拦截状态后执行：

```bash
ros2 topic hz /vpng_data
```

该频率可能低于 60 Hz，因为 PNG 代码会在目标无效或数据完全重复时提前返回。

---

## 10. 实验时需要注意的问题

### 10.1 `/camera_detect_result` 的 60 Hz 不代表 60 张新检测结果

检测节点当前使用独立的 60 Hz 发布线程，重复发布缓存中的检测框。因此：

```text
ros2 topic hz /camera_detect_result ≈ 60 Hz
```

只能证明发布线程是 60 Hz，不能证明 YOLO 或 LightTrack 每秒处理了 60 张新图像。

### 10.2 提高 PNG 频率可能改变现有制导特性

当前 PNG 使用相邻帧 LOS 角度差，没有除以真实时间间隔，并且存在：

```cpp
0.02 rad
```

的更新阈值。频率从 40 Hz 提高到 60 Hz 后，每帧 LOS 角度变化通常会变小，可能更容易低于该阈值。

因此本轮实验建议保持：

```text
N_png = 4.0
```

先只比较频率变化，不要同时修改导航增益。

### 10.3 建议记录的指标

每次仿真至少记录：

- `/camera/image` 实际频率；
- `/vpng_data` 实际频率；
- 最近接距离；
- 水平与垂直最近接误差；
- `ex、ey` 是否在末端接近图像边缘；
- 目标丢失发生时的距离；
- 图像处理实际 FPS；
- Gazebo `real_time_factor`。

这样才能判断命中率变化来自：

- 更高的新图像频率；
- 更低的图像传输延迟；
- 更高的 PNG 计算频率；
- 或分辨率变化带来的检测精度变化。

---

## 11. 后续可改进方向

目前 PNG 频率写死在 C++ 中，每次调整都需要重新编译。后续可以把它参数化，例如：

```yaml
png_frequency: 60.0
```

节点通过参数计算定时器周期。这样以后测试 30、40、50、60 Hz 时，只需修改启动参数，不需要反复编译。

更规范的相机参数管理方式是桥接并订阅 ROS 2 `CameraInfo`，让 PNG 自动读取：

- `width、height`；
- `fx、fy`；
- `cx、cy`；
- 畸变参数。

这样修改 Gazebo 相机分辨率后，不再需要手动同步 PNG 参数。

---

## 12. 后续制导改进方案：框宽末端模式与时间域比例导引

针对最近一次仿真中“近距离目标出画、丢失后擦肩而过”的现象，目前有两个改进方向：

1. 利用目标框宽度判断接近程度，在末端降低速度、增强水平视野保持并缩短惯性续飞时间；
2. 引入真实检测时间间隔，改为基于 LOS 角速度和接近速度的时间域比例导引。

这两个思路并不冲突。第一个方向更直接针对当前故障，短期更可能提高命中率；第二个方向理论上更规范，但需要可靠的时间戳、距离或接近速度估计，适合在第一阶段稳定以后单独实现和对比。

在实施这两项改进前，应先修复两个基础问题：

- 丢失帧数处于 `coast_thresh～lost_thresh` 之间时，程序会落入正常拦截分支并继续使用旧视觉量；
- `vz_ey` 叠加在基础速度之后，最终三维速度没有再次按 `speed_cmd` 限幅。

否则后续试验结果会被这两个问题干扰。

### 12.1 利用目标框宽度建立末端模式

最近一次仿真中，目标框宽度和两机距离具有较明显的对应关系：

| 目标框宽度 | 两机距离 |
|---:|---:|
| 181 px | 2.997 m |
| 246 px | 2.221 m |
| 317 px | 1.731 m |
| 337 px | 1.571 m，随后丢失 |

这几组数据的“框宽 × 距离”大约都在 530～550 附近。因此在本次目标外形和姿态范围内，可以粗略使用：

```text
距离 ≈ K / 目标框宽
```

目标框宽可以作为进入末端阶段的触发指标，但不应直接作为精确距离，因为它还会受到以下因素影响：

- 目标姿态和朝向；
- 检测框抖动；
- 目标部分出画；
- 无人机滚转和俯仰；
- 检测框突然跳变。

建议使用经过低通滤波的归一化框宽：

```cpp
scale = filtered_width / image_width;
```

这样以后再次修改分辨率时，不需要重新按像素换算全部阈值。

以当前 1280 像素图像宽度为例，可以先尝试以下分段，再根据多次仿真统计结果调整：

```text
scale < 0.16，即框宽 < 205 px：
正常 PNG 拦截

0.16 <= scale < 0.22，即框宽约 205～282 px：
进入中近距离阶段，开始减小接近速度并增强偏航跟踪

scale >= 0.22，即框宽 >= 282 px：
进入近距离末端模式，明显缩短丢失后的惯性续飞时间
```

为了避免目标框在阈值附近抖动导致模式频繁切换，还应加入滞回区间。例如进入末端模式使用较高阈值，退出末端模式使用较低阈值。

### 12.2 末端减速不能降得过低

目标无人机自身也有接近 3 m/s 的运动速度。如果拦截机末端直接降到 2 m/s，可能反而无法继续接近目标。

可以先尝试分段限速：

```text
远距离：speed_cmd = 4.0 m/s
中近距离：降至约 3.4～3.6 m/s
非常近：控制相对接近速度，而不是把绝对速度直接降得很低
```

更好的做法是利用目标框宽度增长率估计简化碰撞剩余时间：

```text
w_dot = (w_now - w_prev) / dt
TTC ≈ w / w_dot
```

在目标真实尺寸基本不变、主要沿视线方向接近的条件下，框宽增长越快，说明接近越快、碰撞剩余时间越短，应当越早减小相对接近速度。

由于框宽变化率对检测抖动非常敏感，计算 `w_dot` 前必须对框宽进行滤波，并对异常跳变进行剔除或限幅。

### 12.3 增强水平视野保持

第一次目标丢失前：

```text
ex = -407.5 px
yaw_rate ≈ -0.22 rad/s
```

目标已经位于画面左侧约 24°的位置，但偏航指令只有约 12.6°/s，偏航跟踪能力不足。

不建议继续完全根据像素误差直接调整 `k1_yaw`。更适合把像素误差换算成视线角误差：

```cpp
double yaw_error = std::atan2(ex, focal_length);
double yaw_error_rate = (yaw_error - last_yaw_error) / dt;
double yaw_rate_cmd = kp_yaw * yaw_error
                    + kd_yaw * yaw_error_rate;
```

这种写法与图像分辨率基本解耦。以后从 1280 改回 1920 或改用其他分辨率时，偏航控制参数不需要简单地跟随像素数量成比例修改。

还可以根据目标框宽度调度偏航增益：

```text
目标较远：使用正常偏航增益，抑制检测抖动
目标逐渐变大：提高偏航增益和允许的偏航率
目标接近图像边缘：优先保证目标留在视野中
```

第一次擦肩还伴随明显的向上过冲。因此进入近距离末端模式后，不仅要增强水平补偿，还应降低或暂时冻结基于原始 `ey` 的垂直补偿，避免水平跟踪改善后仍从目标上方或下方擦过。

### 12.4 根据接近程度动态缩短惯性续飞时间

当前配置为：

```text
coast_thresh = 30 帧
检测结果频率约 60 Hz
惯性续飞时间约 0.5 s
```

拦截机以 4 m/s 飞行时，0.5 秒可以飞行约 2 米，而第一次目标丢失时两机距离只有约 1.57 米。因此固定 0.5 秒的惯性续飞明显偏长。

建议不再只根据固定帧数决定惯性续飞时间，而是使用真实时间和目标框宽度动态调整：

```text
目标较远时丢失：
允许保持最后指令约 0.15～0.25 s

目标框已经很大时丢失：
只允许保持最后指令约 0.05～0.10 s

超过允许时间：
立即减速或悬停，并继续偏航搜索目标
```

应使用检测时间戳计算丢失持续时间，而不是继续使用固定帧数。这样检测发布频率从 30 Hz 改为 60 Hz 时，飞行行为不会跟随帧率发生变化。

### 12.5 时间域比例导引不能只给当前公式除以 dt

当前实现近似使用：

```cpp
d_v_angle = last_v_angle + N * delta_LOS;
```

其对应关系是：

```text
dγ = N · dλ
```

如果它被正确地理解为对速度方向的离散积分，公式本身不一定需要显式出现时间，因为积分过程中时间可以抵消。因此不能简单地把 `delta_LOS` 除以 `dt` 后直接加到速度角上，否则量纲和控制强度都会发生变化。

当前实现真正的问题包括：

- LOS 变化量小于 0.02 rad 时不更新；
- 每次有效更新之间的时间间隔并不固定；
- 重复检测框会被跳过；
- 没有显式计算 LOS 角速度；
- 没有接近速度；
- 输出是速度方向，而不是法向加速度；
- 目标丢失后仍可能继续保留并使用旧结果。

如果要实现真正的时间域比例导引，应采用如下处理链：

```text
获得新的相机检测结果
    ↓
构造相机系 LOS 向量并转换到 NED 系
    ↓
归一化得到单位 LOS 向量
    ↓
使用真实检测时间戳计算 LOS 角速度
    ↓
估计接近速度 Vc
    ↓
计算比例导引法向加速度
    ↓
使用 dt 积分得到速度指令
    ↓
限制加速度、速度和指令变化率
```

经典比例导引的核心形式是：

```text
a_n = N · Vc · lambda_dot
```

其中：

- `N`：导航系数；
- `Vc`：接近速度；
- `lambda_dot`：LOS 角速度；
- `a_n`：法向制导加速度。

### 12.6 使用框宽估计距离和接近速度

当前系统只有单目相机，没有直接的目标距离。如果需要计算时间域比例导引中的接近速度，可以利用目标框宽进行估计：

```text
R ≈ K / w
Vc ≈ -(R_now - R_prev) / dt
```

其中常数 `K` 可以使用仿真真值离线标定；实际制导运行时只读取视觉框宽，不使用目标 GPS 真值。

因此两个改进方向可以组合为：

```text
目标框宽及其变化率
    → 估计距离和接近速度

LOS 随时间的变化
    → 估计 LOS 角速度

接近速度 + LOS 角速度
    → 时间域比例导引加速度
```

### 12.7 必须使用新图像对应的时间戳

时间域比例导引不能直接把控制定时器周期固定写成 `1/60 s`。

原因是检测节点可能出现：

- 重复发布上一帧检测结果；
- 某一帧推理时间变长；
- 图像或检测结果丢帧；
- 发布无效目标框。

真正的时间域比例导引应使用对应图像或检测结果的时间戳，只在确认获得新的视觉测量时更新 LOS 角速度。否则重复框会连续产生零角速度，下一张新图像到达后又会产生异常大的角速度跳变。

至少需要对以下变量进行滤波和异常值限制：

```text
LOS 角速度
目标框宽度
目标框宽变化率
估计距离
估计接近速度
```

### 12.8 推荐实施顺序

不建议同时完成全部改动，否则即使命中率提高，也难以判断具体是哪项修改起作用。

第一阶段先修正现有视觉 PNG 系统：

1. 修复 `coast_thresh～lost_thresh` 之间的丢失处理空档；
2. 在叠加 `vz_ey` 后重新限制三维速度总模长；
3. 使用滤波后的归一化框宽进入末端模式；
4. 改成基于视线角误差的偏航跟踪；
5. 近距离将惯性续飞缩短到约 0.05～0.10 秒；
6. 近距离降低或冻结原始 `ey` 垂直补偿；
7. 在相同目标初始条件下统计多次命中率、最近距离和丢框时间。

第二阶段再单独实现时间域比例导引：

1. 为视觉结果增加可靠的新鲜度标志和时间戳；
2. 使用 NED 系单位 LOS 向量计算 LOS 角速度；
3. 使用框宽离线标定距离并估计接近速度；
4. 输出法向加速度，再使用真实 `dt` 积分为速度指令；
5. 增加滤波、加速度限幅、速度限幅和指令变化率限制；
6. 与当前离散 PNG 在完全相同的目标轨迹、初始位置和随机种子下做对照。

总体判断是：框宽末端模式应优先实现，因为它直接解决当前的近距离视野保持和丢框问题；时间域比例导引更适合作为后续更规范的制导结构。即使比例导引公式变得更完整，如果目标仍然频繁出画且无法获得可靠的新 LOS 测量，末端仍然会退化为惯性盲飞。
