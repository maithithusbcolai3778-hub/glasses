# 导盲眼镜 (Guide Glasses)

[English Version](./README.md)

基于 **ESP32-P4 + OV5647 摄像头 + 扬声器 + ATK-MS601M 六轴陀螺仪** 的可穿戴助行原型，感知与决策由 PC 端 Agent 完成。

> **状态**：研究原型。当前架构把 YOLO 检测、大模型推理放在与眼镜通过 Wi-Fi 连接的 PC 上；眼镜端负责图像采集、IMU 读取、语音播报和 HTTP 服务。

---

## 功能特性

- **实时目标检测**：行人、车辆、红绿灯、盲道、斑马线、障碍物等。
- **IMU 姿态感知**：ATK-MS601M 六轴陀螺仪输出 Roll / Pitch / Yaw，把画面中的目标位置映射到真实方向。
- **局部路径引导**：有盲道时沿盲道行走，无盲道时在可通行区域中选择安全方向。
- **中文语音播报**：将识别结果和引导建议用自然语言播报到板载扬声器。
- **Web 仪表盘**：轻量 HTML 界面，叠加显示 MJPEG 视频流、检测框和 IMU 状态。
- **云端大模型兜底**：复杂或不确定场景调用 DashScope Qwen-VL 进行综合理解。

---

## 系统架构

![系统架构](./assets/system_architecture.png)

```text
┌─────────────────────────────────────────────┐
│  ESP32-P4（眼镜端）                          │
│  OV5647 ──→ JPEG ──→ HTTP (/snapshot.jpg)   │
│  ATK-MS601M ──→ UART ──→ IMU (/imu)         │
│  扬声器 ←── PCM ←── POST /play              │
└─────────────────┬───────────────────────────┘
                  │ Wi-Fi / HTTP
                  ▼
┌─────────────────────────────────────────────┐
│  PC Agent（Python）                          │
│  取流 → YOLO 检测 → 引导决策                 │
│  ↳ Qwen-VL 云端兜底（DashScope）             │
│  ↳ edge-tts / pyttsx3 → PCM → /play         │
│  ↳ Dashboard（http://localhost:8080）        │
└─────────────────────────────────────────────┘
```

更多流程图见 [`./assets`](./assets) 目录。

---

## 硬件清单

| 硬件 | 用途 |
|------|------|
| ESP32-P4 开发板 | 主控、Wi-Fi（通过 esp_wifi_remote + 协处理芯片）、摄像头 ISP、音频 I2S |
| OV5647 MIPI 摄像头 | 图像采集 |
| 板载扬声器 / 音频功放 | 语音播报 |
| ATK-MS601M（ICM-20602 + STM32） | 六轴陀螺仪，UART 直接输出融合后的姿态角 |
| 带 NVIDIA GPU 的 PC | 运行 YOLO 推理和 PC Agent |
| Wi-Fi 路由器 | 眼镜与 PC 通信 |
| 移动电源 | 户外供电 |

---

## 仓库结构

```text
.
├── assets/              # 架构图与流程图
├── components/          # ESP-IDF 本地组件
├── docs/                # 中文设计文档
├── main/                # ESP-IDF 固件源码
├── managed_components/  # ESP-IDF 托管组件
├── pc_agent/            # PC 端 Python Agent
│   ├── pc_agent.py      # 主感知 + 引导循环
│   ├── dashboard.py     # Web 仪表盘服务
│   ├── dashboard.html   # 仪表盘页面
│   ├── frame_grabber.py # MJPEG 流读取
│   ├── qwen_agent.py    # Qwen-VL 云端 Agent
│   ├── tts_helper.py    # 离线 TTS 辅助脚本
│   └── tests/           # 测试脚本
├── scripts/             # Windows 编译/烧录/运行批处理
├── tools/               # IMU HTML 仪表盘生成工具
├── .env.example         # 环境变量示例
├── .gitignore
├── CMakeLists.txt       # ESP-IDF 工程入口
├── LICENSE
├── README.md
├── README_CN.md
├── requirements.txt
└── sdkconfig*           # ESP-IDF 配置
```

---

## 快速开始

### 1. 克隆并配置

```bash
git clone <仓库地址>
cd guide-glasses
```

复制示例环境文件并填入真实值：

```bash
cp .env.example .env
# 编辑 .env：填入 DashScope API Key、Wi-Fi 密码等
```

> **不要把 `.env` 提交到 Git**，它已被 `.gitignore` 忽略。

### 2. 编译并烧录 ESP32-P4 固件

在 ESP-IDF 终端中（推荐 v5.5.4）执行：

```bash
idf.py set-target esp32p4
idf.py menuconfig   # 在 Example Connection Configuration 中设置 Wi-Fi SSID / 密码
idf.py build
idf.py -p PORT flash monitor
```

Windows 也可使用提供的批处理：

```powershell
.\scripts\build_p4.bat
.\scripts\flash_p4.bat
```

### 3. 运行 PC Agent

安装 Python 依赖（推荐 Python 3.10+）：

```bash
cd pc_agent
pip install -r ../requirements.txt
```

启动主 Agent：

```bash
python pc_agent.py --host <esp32-ip-or-mdns>
```

仅启动 Qwen-VL Agent（不需要本地 GPU）：

```bash
python qwen_agent.py --host <esp32-ip-or-mdns>
```

浏览器打开 `http://localhost:8080` 查看仪表盘。

---

## 配置说明

`menuconfig` 中需要关注：

- **Wi-Fi SSID / 密码**：`Example Connection Configuration`
- **摄像头分辨率**：`Example Configuration → Camera preview resolution`（默认 800×480）
- **音频 / SD 卡**：`Example Configuration → AUDIO_SPEAKER_ENABLE / SD_CARD_ENABLE`
- **分区表**：`CONFIG_PARTITION_TABLE_SINGLE_APP_LARGE=y` 使用 2 MB 大 App 分区

环境变量见 `.env.example`：

| 变量 | 用途 |
|------|------|
| `DASHSCOPE_API_KEY` | DashScope API Key，用于 Qwen-VL |
| `WIFI_SSID` / `WIFI_PASSWORD` | 部分脚本需要 |
| `EDGE_TTS_PROXY` | `edge-tts` 可选 HTTP 代理 |
| `ESP32_HOST` | 默认 ESP32 IP |

---

## 模型权重

PC Agent 需要 YOLO 权重文件。开发中使用过以下模型，但因体积较大**未放入本仓库**：

- `yolov8n.pt` — Ultralytics 官方 COCO 预训练模型（可通过 `ultralytics` 自动下载）
- `yolo26n.pt` — 针对导盲场景自定义训练的模型

请自行准备 `.pt` 文件，放入 `models/` 目录（已被 Git 忽略），或在脚本中修改 `MODEL_PATH`。

---

## 许可证

本项目采用 [MIT License](./LICENSE)。

---

## 致谢

- Espressif ESP-IDF 与 ESP32-P4 示例
- Ultralytics YOLO
- 阿里云 DashScope / Qwen-VL
