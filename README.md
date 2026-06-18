# 智能儿童安全监护系统 (Smart Child Safety Monitoring System)

基于 RK3588 平台的 YOLOv8 姿态估计 + 实例分割 + 双目深度估计的儿童安全实时监护系统。RK3588 端进行 AI 推理和危险行为分析，通过 TCP 将告警帧发送到 PC 上位机进行实时显示。

## 系统架构

```
┌─────────────────────────┐       TCP (Port 9527)       ┌──────────────────────┐
│     RK3588 边缘端        │ ──────────────────────────> │     PC 上位机 (UI)    │
│                          │    [JSON + JPEG + JPEG]     │                      │
│  ┌──────────────────┐   │                             │  ┌────────────────┐  │
│  │ 双目相机 (Stereo) │   │                             │  │  实时视频展示   │  │
│  ├──────────────────┤   │                             │  ├────────────────┤  │
│  │ YOLOv8 Pose 姿态 │   │                             │  │  深度图显示     │  │
│  │ YOLOv8 Seg  分割  │   │                             │  ├────────────────┤  │
│  ├──────────────────┤   │                             │  │  CPU/NPU 状态   │  │
│  │ SGBM/WLS 深度图   │   │                             │  ├────────────────┤  │
│  ├──────────────────┤   │                             │  │  告警弹窗+历史  │  │
│  │ 危险行为分析      │   │                             │  └────────────────┘  │
│  │ 系统资源监控      │   │                             │                      │
│  └──────────────────┘   │                             └──────────────────────┘
└─────────────────────────┘
```

### 告警类型

| 告警 | 说明 |
|------|------|
| 姿态失衡 (Balance) | 人体重心与脚部支撑点偏移过大，可能摔倒 |
| 危险攀爬 (Climbing) | 手腕高于眼睛且手肘弯曲角度 < 75° |
| 探头危险 (Leaning) | 鼻子到窗户/边界距离 < 0.5m |
| 距离过近 (Proximity) | 人体最近点距离 < 1.0m（可配置） |

## 目录结构

```
YOLO_V3.0/
├── UI/                          # PC 上位机 (Python)
│   ├── host_server.py           # 主程序 (tkinter GUI)
│   ├── requirements.txt         # Python 依赖
│   └── accounts.json            # 登录账号配置
├── yolo/                        # RK3588 边缘端 (C++)
│   ├── CMakeLists.txt           # CMake 构建配置
│   ├── include/                 # 头文件
│   │   ├── yolov8_pose.h        # YOLOv8 姿态估计
│   │   ├── yolov8_seg.h         # YOLOv8 实例分割
│   │   ├── behavior_analyzer.h  # 行为分析
│   │   ├── danger_monitor.h     # 危险监测
│   │   ├── stereo_camera.h      # 双目相机
│   │   ├── network_sender.h     # TCP 发送
│   │   └── sys_monitor.h       # 系统资源监控
│   ├── src/                     # 源文件
│   ├── model/                   # RKNN 模型文件
│   │   ├── yolov8_pose.rknn     # 姿态估计模型
│   │   └── yolov8_seg.rknn      # 实例分割模型
│   ├── rknn_lib/                # RKNN 运行时库
│   ├── 3rdparty/                # 第三方库
│   ├── utils/                   # 工具函数
│   ├── inputimage/              # 测试图片
│   ├── outputimage/             # 推理输出
│   └── stereo_calib.yaml        # 双目相机标定参数
├── .gitignore
└── README.md
```

## 硬件要求

### RK3588 边缘端
- **开发板**: Rockchip RK3588 (Orange Pi 5 / Firefly ITX-3588J 等)
- **系统**: Ubuntu 20.04+ / Debian 11+ (aarch64)
- **相机**: USB 双目相机 (UVC stereo camera, 设备号 cam41)
- **NPU 驱动**: RKNN 2.x runtime

### PC 上位机
- 任何运行 Python 3.8+ 的 Windows/Linux/macOS
- 与 RK3588 在同一局域网

## 软件依赖

### RK3588 端

- **RKNN Runtime**: `librknnrt.so` (已包含在 `yolo/rknn_lib/`)
- **RGA**: Rockchip 图形加速库 (已包含在 `yolo/3rdparty/librga/`)
- **OpenCV 4.x**: `sudo apt install libopencv-dev`
- **CMake 3.10+** / **GCC 9+** (支持 C++17)

环境准备:
```bash
# 安装 OpenCV
sudo apt update
sudo apt install libopencv-dev cmake build-essential

# RKNN 驱动 (如未安装)
# 参考: https://github.com/rockchip-linux/rknpu2
```

### PC 上位机端

```bash
cd UI
pip install -r requirements.txt
```

依赖: `Pillow>=8.0.0` (tkinter 为 Python 自带)

## 编译 (RK3588 端)

```bash
cd yolo
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

编译产物: `yolo_pose_seg_rk3588`

## 使用说明

### 步骤 1: 启动 PC 上位机

在 PC 端启动监控程序:

```bash
cd UI
python host_server.py
```

- 默认账号: `admin` / `123456`
- 监听地址: `0.0.0.0:9527`
- **请修改 `accounts.json` 中的默认密码**

程序启动后等待 RK3588 设备连接，状态栏显示 "等待设备连接"。

### 步骤 2: 标定双目相机

首次使用需标定双目相机，生成 `stereo_calib.yaml`。可使用 OpenCV 标定工具或项目自带的标定程序。

### 步骤 3: 运行 RK3588 端

#### 相机实时模式 (默认)

```bash
./yolo_pose_seg_rk3588 \
    -c 41 \                          # 相机设备 ID
    -f ./stereo_calib.yaml \         # 标定文件路径
    -s 0.5 \                         # SGBM 缩放比例
    --host 192.168.1.100 \           # PC 上位机 IP
    --port 9527                      # PC 上位机端口
```

#### 单张图片测试模式

```bash
./yolo_pose_seg_rk3588 --image ./inputimage/0001.jpg
```

### 命令行参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `--image <path>` | 单张图片推理模式 | - |
| `-c, --camera <id>` | 相机设备 ID | 41 |
| `-p, --pipeline` | GStreamer pipeline 字符串 | - |
| `-f, --calib <file>` | 双目标定 YAML 文件 | stereo_calib.yaml |
| `-s, --scale <f>` | SGBM 深度缩放 | 0.5 |
| `--fast` | 跳过 WLS 滤波 (加速) | false |
| `--host <ip>` | PC 服务器 IP | 192.168.137.1 |
| `--port <port>` | PC 服务器端口 | 9527 |
| `-h, --help` | 显示帮助 | - |

### 通信协议

RK3588 → PC 上位机，TCP 二进制帧格式:

```
[4字节 JSON长度 (大端)] [JSON 元数据] [4字节 JPEG1长度] [标注图JPEG] [4字节 JPEG2长度] [深度图JPEG]
```

JSON 元数据包含: 时间戳、CPU/NPU 使用率、告警状态、人物关键点信息等。

## 注意事项

1. **模型文件**: 确保 `yolo/model/` 目录下有 `yolov8_pose.rknn` 和 `yolov8_seg.rknn`
2. **RKNN 库**: `yolo/rknn_lib/librknnrt.so` 需与 RK3588 系统的 NPU 驱动版本匹配
3. **双目相机**: 需要在 `/dev/video*` 下有两个设备节点 (通常 cam41 对应双目相机的左/右拼接设备)
4. **网络连通**: RK3588 和 PC 需在同一网段，防火墙放行 TCP 9527 端口
5. **默认密码**: 首次使用请修改 `UI/accounts.json` 中的密码
6. **标定文件**: 不同相机需重新标定，`stereo_calib.yaml` 仅适用于标定时的相机

## License

MIT License
