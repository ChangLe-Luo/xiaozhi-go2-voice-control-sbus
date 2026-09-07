# 面包板 ESP32-S3 CAM (bread-compact-wifi-s3cam)

> 基于 ESP32-S3 + OV2640 摄像头, Go2 机器狗 SBUS 语音控制器

> ⚠️ **当前模式**: SBUS GPIO47 占据原 LCD DC 引脚, 显示屏已禁用。恢复 LCD 需改 `serial_commands.h` 中的 `SBUS_TX_PIN` 到 GPIO 3

---

## 硬件规格

| 项目 | 参数 |
|------|------|
| 主控 | ESP32-S3 |
| 摄像头 | OV2640 (DVP 8-bit) |
| 显示屏 | ST7789 240×240 SPI (⚠️ SBUS 模式下禁用) |
| 音频 | I2S 麦克风 + I2S 功放 (Simplex) |
| 网络 | WiFi 2.4GHz |
| SBUS 输出 | 硬件 UART GPIO47, 100000bps 8E2 反相 |

---

## 完整引脚映射

### 🎤 音频 — I2S (Simplex 模式)

| 功能 | GPIO | 说明 |
|------|------|------|
| MIC_WS | 1 | 麦克风 左右声道切换 |
| MIC_SCK | 2 | 麦克风 时钟 |
| MIC_DIN | 42 | 麦克风 数据输入 |
| SPK_DOUT | 39 | 喇叭 数据输出 |
| SPK_BCLK | 40 | 喇叭 位时钟 |
| SPK_LRCK | 41 | 喇叭 左右声道切换 |

> 采样率: 输入 16kHz / 输出 24kHz

> **备选 Duplex 模式** (共用 I2S): WS=1, BCLK=2, DIN=42, DOUT=39 — 需取消 `AUDIO_I2S_METHOD_SIMPLEX` 宏

---

### 📺 显示屏 — SPI3 (⚠️ 当前 SBUS 模式下禁用)

| 功能 | GPIO | 说明 |
|------|------|------|
| MOSI | 20 | 数据 |
| CLK | 19 | 时钟 |
| DC | 47 | ⚠️ SBUS TX 占用, LCD 不可用 |
| RST | 21 | ⚠️ SBUS RX 占用, LCD 不可用 |
| CS | 45 | 片选 |
| BACKLIGHT | 38 | 背光 PWM |

> 驱动: ST7789, 240×240, SPI Mode 0, RGB 顺序
>
> ⚠️ SBUS (Go2 控制) 与 LCD 共享 GPIO47/21, 二选一。当前选择 SBUS, LCD 初始化代码已注释

---

### 📷 摄像头 — DVP 8-bit 并行

| 功能 | GPIO | 说明 |
|------|------|------|
| D0 | 11 | 数据 bit 0 |
| D1 | 9 | 数据 bit 1 |
| D2 | 8 | 数据 bit 2 |
| D3 | 10 | 数据 bit 3 |
| D4 | 12 | 数据 bit 4 |
| D5 | 18 | 数据 bit 5 |
| D6 | 17 | 数据 bit 6 |
| D7 | 16 | 数据 bit 7 |
| XCLK | 15 | 主时钟 20MHz (LEDC) |
| PCLK | 13 | 像素时钟 |
| VSYNC | 6 | 垂直同步 |
| HREF | 7 | 水平参考 |
| SIOC | 5 | SCCB 时钟 (类 I2C) |
| SIOD | 4 | SCCB 数据 |
| PWDN | NC | 断电 (未接) |
| RESET | NC | 复位 (未接) |

> 型号: OV2640, 输出 RGB565 VGA, JPEG 质量 12

---

### 🔘 按键 & 💡 LED

| 功能 | GPIO | 说明 |
|------|------|------|
| BOOT | 0 | 启动按键 (短按=对话, 长按=配网) |
| LED | 48 | 板载指示灯 |
| LAMP | 14 | 外接灯控 (MCP 可控制开关) |
| TOUCH | NC | 触摸按键 (未接) |
| VOL+ | NC | 音量+ (未接) |
| VOL- | NC | 音量- (未接) |

---

### 🤖 SBUS 输出 — 硬件 UART (UART1)

| 功能 | GPIO | 说明 |
|------|------|------|
| SBUS_TX | 47 | SBUS 信号输出 (硬件反相) |
| SBUS_RX | 21 | 预留 (本场景不用) |

> 协议: 100000bps 8E2 反相 (硬件 UART), 25 字节/帧, 16 通道 × 11 位

> `uart_set_line_inverse(UART_SIGNAL_TXD_INV)` 实现硬件反相

> 接法: GPIO 47 → Go2 Air J4 接口 Pin 3 (SBUS RX), ESP GND → Go2 J4 Pin 1 (GND)

> 配置: [serial_commands.h](serial_commands.h)

---

### 🔋 电池 & 电源

| 功能 | 说明 |
|------|------|
| 充电检测 | NC (未接) |
| 电池 ADC | ADC2 CH3, 12-bit, 衰减 12dB |

> 电量通过 ADC 采样 + 线性插值估算 (0%–100%)

---

## GPIO 占用总览

```
GPIO  0: BOOT 按键
GPIO  1: I2S MIC WS
GPIO  2: I2S MIC SCK
GPIO  3: 未用
GPIO  4: CAM SIOD
GPIO  5: CAM SIOC
GPIO  6: CAM VSYNC
GPIO  7: CAM HREF
GPIO  8: CAM D2
GPIO  9: CAM D1
GPIO 10: CAM D3
GPIO 11: CAM D0
GPIO 12: CAM D4
GPIO 13: CAM PCLK
GPIO 14: LAMP 灯控
GPIO 15: CAM XCLK
GPIO 16: CAM D7
GPIO 17: CAM D6
GPIO 18: CAM D5
GPIO 19: LCD CLK (SPI) — LCD 已禁用
GPIO 20: LCD MOSI (SPI) — LCD 已禁用
GPIO 21: SBUS RX (预留) / 原 LCD RST
GPIO 38: 未用 (原 LCD 背光)
GPIO 39: I2S SPK DOUT
GPIO 40: I2S SPK BCLK
GPIO 41: I2S SPK LRCK
GPIO 42: I2S MIC DIN
GPIO 45: 未用 (原 LCD CS)
GPIO 47: SBUS TX (UART1, 硬件反相)  ⬅ Go2 核心控制
GPIO 48: LED

未用: 3, 38, 43, 44, 45, 46
```

---

## 编译 & 烧录

```bash
idf.py set-target esp32s3
idf.py menuconfig
# Xiaozhi Assistant → Board Type → 面包板新版接线（WiFi）+ LCD + Camera
# 注意: SBUS 模式下 LCD 自动禁用, 选此 Board Type 仅为统一编译配置
idf.py build flash monitor
```

---

## 文件说明

| 文件 | 用途 |
|------|------|
| [config.h](config.h) | 引脚定义、LCD 型号、摄像头参数 |
| [serial_commands.h](serial_commands.h) | SBUS 引脚、通道常量、命令列表、移动参数 |
| [compact_wifi_board_s3cam.cc](compact_wifi_board_s3cam.cc) | 板级初始化、SBUS 编码/心跳、MCP 工具注册 |
| [power_manager.h](power_manager.h) | 电池电量检测 & 充电状态 |
| [SBUS_ARCHITECTURE.md](SBUS_ARCHITECTURE.md) | SBUS 架构文档 (ESP32 + Go2 端完整说明) |
