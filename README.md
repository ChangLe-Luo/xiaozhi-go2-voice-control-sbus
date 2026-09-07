# 小智 AI 语音助手 · Go2 机器狗 SBUS 语音控制器

> 面包板 ESP32-S3 + OV2640 改造版 · 基于开源项目 [xiaozhi-esp32](https://github.com/78/xiaozhi-esp32) v2.2.4

## 项目简介

本项目是小智 AI 语音助手固件（xiaozhi-esp32 v2.2.4）的二次开发分支。它把一块「面包板 Bread Compact WiFi + LCD + Camera」开发板（ESP32-S3 + OV2640 摄像头）改造成**宇树 Unitree Go2 Air 机器狗的语音遥控器**：

- 语音经云端大模型（Qwen / DeepSeek 等）理解，通过小智的 MCP 工具 `self.serial.send_command` 下发动作指令；
- ESP32-S3 把指令编码为 **SBUS 遥控帧**，经 GPIO47 硬件 UART（100000bps 8E2 反相）输出；
- 帧经 Go2 Air 的 J4 扩展口（Pin3 SBUS RX）进入狗端，由 Python 桥接器 `go2_bridge.py` 解码，最终调用 Unitree SportClient SDK → DDS → 电机。

一句话链路：

```text
语音 → 云端 LLM → 小智 MCP(self.serial.send_command)
     → ESP32-S3 SBUS UART(GPIO47) → Go2 J4 → go2_bridge.py → SportClient/DDS → 电机
```

## 硬件与环境

| 项目 | 说明 |
|------|------|
| 目标芯片 | ESP32-S3（`CONFIG_IDF_TARGET="esp32s3"`） |
| 板型 | 面包板 Bread Compact WiFi + LCD + Camera（`BOARD_TYPE_BREAD_COMPACT_WIFI_CAM`） |
| 摄像头 | OV2640（DVP 8-bit，VGA RGB565） |
| 显示屏 | ST7789 240×240 —— 本分支**已禁用**，GPIO47/21 让给 SBUS |
| 音频 | I2S 麦克风（INMP441）+ I2S 功放，Simplex 模式 |
| ESP-IDF | v5.5.4（上游要求 ≥ 5.4） |
| Flash 分区表 | `partitions/v2/16m.csv`（16MB Flash） |
| 监视串口波特率 | 115200 |

SBUS 接线（Go2 Air）：

| ESP32-S3 | Go2 Air J4 接口 |
|----------|-----------------|
| GPIO47 (TX) | Pin 3 (SBUS RX) |
| GND | Pin 1 (GND) |

## 相对上游的改动

1. **复用上游板型目录** `main/boards/bread-compact-wifi-s3cam/`，把「面包板 + 摄像头」板卡改造为 Go2 狗控制器。
2. **新增 SBUS 发送逻辑**（`compact_wifi_board_s3cam.cc`）：25 字节/帧、16 通道 × 11 bit，5ms 心跳（约 200Hz），硬件反相 UART；支持多帧脉冲 toggle 与摇杆渐变移动。
3. **新增 11 条中文语音命令**（`serial_commands.h`）：停下来 / 站立模式 / 卧倒 / 蹲下 / 跑步模式 / 前进 / 后退 / 向左走 / 向右走 / 向左转弯 / 向右转弯，通过 MCP 工具 `self.serial.send_command` 暴露给云端 LLM。
4. **禁用 LCD**：GPIO47/21 原为 LCD DC/RST，现让给 SBUS TX/RX，LCD 初始化代码已注释。
5. **新增架构文档** `SBUS_ARCHITECTURE.md`，完整记录 ESP32 与狗端的协议、通道映射、状态机与部署运维。
6. 未改动上游唤醒词、字体、表情、MQTT/OTA 服务器配置（沿用上游公共默认值）。

## 编译与烧录

> 以下为通用步骤；**不要**在代码或本文档中写入真实 WiFi 凭据。WiFi 账号密码在首次开机时通过配网页面写入设备 NVS，不在源码中硬编码。

```bash
# 1. 准备 ESP-IDF（v5.4 及以上），并激活环境
#    export IDF_PATH=/path/to/esp-idf

# 2. 设置目标芯片
idf.py set-target esp32s3

# 3. 选择板型（menuconfig）
#    Xiaozhi Assistant → Board Type → "Bread Compact WiFi + LCD + Camera (面包板)"
idf.py menuconfig

# 4. 编译
idf.py build

# 5. 烧录并打开监视器（<串口> 替换为实际串口，如 COM3 / /dev/ttyUSB0）
idf.py -p <串口> flash monitor
```

首次上电会进入配网模式（设备开热点或 Web 配网页），在其中填写：

- WiFi：`<WiFi_SSID>` / `<WiFi_密码>`
- 服务器：默认接入小智官方服务器（xiaozhi.me），可按需改为自有 WebSocket/MQTT 地址 `<服务器_地址>`

## 功能特性

- 离线语音唤醒（ESP-SR）+ 流式 ASR/LLM/TTS 语音交互（OPUS 音频编解码）
- 摄像头（OV2640）图像采集，支持把画面发给云端大模型
- 11 条中文语音指令遥控 Go2 机器狗：站立 / 卧倒 / 蹲下 / 跑步 / 前后左右移动 / 转向
- 移动指令自动「先站立再移动」，避免蹲/卧姿态下无响应
- 多帧脉冲 toggle + 1s 滑动窗口，解决站立/卧倒误判
- 电量检测（ADC 采样 + 线性插值估算）
- 设备端 MCP 控制（音量、灯光、GPIO 等）

## 目录结构

```text
├── main/
│   ├── boards/bread-compact-wifi-s3cam/   # 核心改动：Go2 SBUS 板级实现
│   │   ├── compact_wifi_board_s3cam.cc    # 板级初始化 + SBUS 编码/心跳 + MCP 工具注册
│   │   ├── serial_commands.h              # SBUS 引脚/通道常量/命令列表/移动参数
│   │   ├── config.h                       # 引脚定义、LCD/摄像头/音频参数
│   │   ├── power_manager.h                # 电池电量与充电状态
│   │   ├── README.md                      # 板级引脚映射说明
│   │   └── SBUS_ARCHITECTURE.md           # SBUS 完整架构文档
│   ├── application.cc / mcp_server.cc / ota.cc / ...   # 上游应用层（未改动）
│   └── CMakeLists.txt / Kconfig.projbuild             # 板型注册（沿用上游板型）
├── docs/                                   # 上游文档（MCP/WebSocket/MQTT 等）
├── partitions/v2/16m.csv                   # 16MB 分区表
├── backups/                                # 改造前快照（开发备份，发布时可移除）
└── sdkconfig.defaults                      # 默认编译配置
```

## 上游致谢

本项目基于 [xiaozhi-esp32](https://github.com/78/xiaozhi-esp32)（小智 AI 聊天机器人）二次开发，感谢 78 与全体贡献者，以及 Shenzhen Xinzhi Future Technology Co., Ltd. 的开源工作。上游采用 [MIT License](https://github.com/78/xiaozhi-esp32/blob/main/LICENSE)，本项目亦遵循 MIT 许可证，请保留原版权与许可声明。

本分支仅新增/修改 Go2 控制相关代码与文档，其余均继承自上游。
