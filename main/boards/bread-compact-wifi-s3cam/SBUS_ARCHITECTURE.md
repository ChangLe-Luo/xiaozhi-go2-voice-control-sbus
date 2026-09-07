# Go2 Air SBUS 控制架构

> ESP32-S3 小智 AI 助手 → 宇树 Go2 Air 机器狗 SBUS 遥控方案

---

## 系统总览

```
┌─────────────────────────────────────────────────────────────┐
│  ESP32-S3 (小智)                                            │
│  ┌──────────┐    ┌──────────┐    ┌───────────────────────┐ │
│  │ 语音输入  │ → │ AI 识别   │ → │ MCP Tool:             │ │
│  │ (I2S麦克风)│    │ (云端LLM) │    │ self.serial.send_cmd │ │
│  └──────────┘    └──────────┘    └──────────┬────────────┘ │
│                                              │               │
│                              SBUS UART (100000bps 8E2 inv)  │
│                              GPIO47 TX ──────────────────┐   │
└─────────────────────────────────────────────────────────│───┘
                                                          │
                    ┌─────────────────────────────────────┘
                    ▼
┌─────────────────────────────────────────────────────────────┐
│  Go2 Air 机器狗                                             │
│                                                             │
│  J4 扩展口 Pin3 (SBUS RX)                                    │
│       │                                                     │
│       ▼                                                     │
│  /dev/ttyS0 (Rockchip UART, 100000bps 8E2)                  │
│       │                                                     │
│       ├── sbus_handle (官方模块, 只配置串口参数, 不发布DDS)     │
│       │                                                     │
│       └── go2_bridge.py (Python 桥接, 并行读取)               │
│            │  SBUS 解码 (25字节/帧, 16ch×11bit)             │
│            │  状态机命令检测 (1s滑动窗口)                     │
│            │  多线程执行 (do() 不阻塞主循环)                  │
│            ▼                                                │
│       SportClient SDK → DDS → mcf → 电机                    │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 硬件连接

| ESP32-S3 | Go2 Air J4 接口 |
|----------|----------------|
| GPIO47 (TX) | Pin 3 (SBUS RX) |
| GND | Pin 1 (GND) |

---

## SBUS 协议参数

| 参数 | 值 |
|------|-----|
| 波特率 | 100000 bps |
| 数据位 | 8 |
| 校验 | Even (偶校验) |
| 停止位 | 2 |
| 信号 | **反相** (ESP32-S3 硬件反相) |
| 帧长 | 25 字节 |
| 通道数 | 16 路 × 11 bit |
| 帧率 | ~200 Hz (每 5ms 一帧心跳) |

### 帧格式

```
Byte 0:       0x0F (帧头)
Byte 1-22:    16通道 × 11bit 编码数据
Byte 23:      标志位
Byte 24:      0x00 (帧尾)
```

### 通道映射

| SBUS 通道 | 索引 | 功能 | 默认值 |
|-----------|------|------|--------|
| CH1 | ch[0] | Rx (转弯) | 992 |
| CH2 | ch[1] | Ry | 992 |
| CH3 | ch[2] | Ly (前进后退) | 992 |
| CH4 | ch[3] | Lx (左右平移) | 992 |
| CH5 | ch[4] | 模式选择 | 992 |
| CH6 | ch[5] | 蹲下 | **192** |
| CH7 | ch[6] | 解除锁定 | **192** |
| CH8 | ch[7] | 站立/卧倒 | 992 |

### 通道值

| 名称 | 值 |
|------|-----|
| LOW | 192 |
| NEUTRAL | 992 |
| HIGH | 1792 |

---

## 命令映射

### Toggle 命令 (多帧脉冲, ~165ms)

| 语音命令 | 通道 | 翻转序列 | Go2 API |
|---------|------|---------|---------|
| 解除锁定 | CH7 | 192→1792→192 | Damp |
| 站立模式 | CH8 | 992→1792→192 | StandUp |
| 卧倒 | CH8 | 992→192→992 | StandUp(2s) → StandDown |
| 蹲下 | CH6 | 192→1792→192 | StandUp(2s) → Sit |
| 跑步模式 | CH8 + CH5=HIGH | 992→1792→992 | StandUp(2s) → SwitchGait |

> 每状态发 10 帧 (sync write, ~5.5ms/帧), 3 状态共 30 帧 ~165ms
> Go2 1 秒滑动窗口内可靠捕获 HIGH/LOW, 解决 3 帧方案中 HIGH 帧丢失导致的"站立误判为卧倒"

### 持续移动命令

| 语音命令 | 摇杆 | Go2 API |
|---------|------|---------|
| 前进 | Ly > 1192 | StandUp(条件) → Move(vx, 0, 0) |
| 后退 | Ly < 792 | StandUp(条件) → Move(-vx, 0, 0) |
| 向左走 | Lx < 792 | StandUp(条件) → Move(0, vy, 0) |
| 向右走 | Lx > 1192 | StandUp(条件) → Move(0, -vy, 0) |
| 向左转弯 | Rx < 792 | StandUp(条件) → Move(0, 0, vyaw) |
| 向右转弯 | Rx > 1192 | StandUp(条件) → Move(0, 0, -vyaw) |

> **所有移动命令均先调 `ensure_stand(sp)`** — 狗在蹲/卧状态也会先站立再移动
> 通过 `DOG_STANDING` 全局标志位追踪姿态: 已站立则跳过 StandUp, 只执行 Move (~1.5s)

---

## Go2 桥接实现

### 数据流

```
/dev/ttyS0 (与 sbus_handle 并行打开, sbus_handle 负责配置串口参数)
    │
    ▼
sbus_decode() → 25字节 → 16通道值
    │
    ▼
帧校验: CH5/CH6/CH7/CH8 必须在 [L,N,H] ±150 范围内
    │
    ▼
StateMachine.update()
    │
    ├── 状态分类: st() → 0=LOW, 1=NEUTRAL, 2=HIGH
    │
    ├── Toggle 检测 (1秒滑动窗口)
    │   ├── CH8 HIGH+LOW → stand (CH5高=run_mode)
    │   ├── CH8 LOW only (无HIGH, CH7无HIGH) → lie_down
    │   ├── CH7 HIGH+LOW (CH8无HIGH) → unlock
    │   └── CH6 HIGH+LOW → sit
    │
    └── 移动检测 (单帧, 偏移 >200)
        └── Ly/Lx/Rx 偏移 → forward/backward/left/right/turn
    │
    ▼
threading.Thread(target=do)  ← 不阻塞主循环
    │
    ├── ensure_stand()  ← DOG_STANDING 检查, 已站立则跳过
    │   StandUp() + sleep(2s)
    │
    ├── Move / StandDown / Sit / SwitchGait / Damp
    │
    └── DOG_STANDING 标志更新
         stand→True  坐/卧/解除→False
    │
    ▼
SportClient SDK → DDS → mcf → 电机
```

### 关键设计决策

| 决策 | 原因 |
|------|------|
| 所有动作先 `ensure_stand(sp)` | 狗在任意姿态都能响应指令 (蹲/卧 → 站起 → 执行) |
| `DOG_STANDING` 姿态追踪 | 已站立时跳过 StandUp, 避免 2s 冗余等待 |
| `ensure_stand` 在移动前 | StandUp 不锁死 Move, 先站再走可正常执行 |
| do() 跑在独立线程 | 防止 sleep 阻塞串口读取 |
| 1秒 toggle 检测窗口 + 多帧脉冲 (每状态 10 帧) | 100kbps 下 HIGH/LOW 可靠捕获, 解决站立/卧倒误判 |
| sbus_write_frame_sync 阻塞发送 | 帧间无重叠, Go2 解析更稳定 |

### 已知局限

| 问题 | 原因 | 影响 | 状态 |
|------|------|------|------|
| ~~"站立"偶被误判"卧倒"~~ | ~~toggle 仅 3 帧~8ms, HIGH 帧可能丢失~~ | ~~站立命令不稳定~~ | ✅ 已修复 (多帧脉冲) |
| ~~移动命令不自动站立~~ | ~~StandUp 会锁死 Move~~ | ~~需先手动说"站立模式"~~ | ✅ 已修复 (ensure_stand+姿态追踪) |
| 无避障 | 未启用 obstacle_avoid 通道 | 移动时不避障 | 待实现 |
| 首次移动慢 | 初始姿态未知, 首次必调 StandUp(2s) | 第一个移动命令 ~3.5s | 可接受 |

> **2026-07-16 修复**:
> 1. Toggle 从 3 帧 → 3×10 帧 (~165ms), 解决站立/卧倒误判问题
> 2. 所有移动命令加入 `ensure_stand(sp)`, 搭配 `DOG_STANDING` 姿态追踪 —— 已站立时跳过, 仅首次/姿态变化后触发 StandUp

## 部署运维

### 服务管理

```bash
systemctl status go2-sbus-bridge     # 状态
journalctl -u go2-sbus-bridge -f     # 实时日志
systemctl restart go2-sbus-bridge    # 重启
```

### 文件

| 文件 | 路径 |
|------|------|
| 桥接脚本 | `/root/go2_bridge.py` |
| systemd 服务 | `/etc/systemd/system/go2-sbus-bridge.service` |
| ESP32 配置 | `main/boards/bread-compact-wifi-s3cam/serial_commands.h` |
| ESP32 主逻辑 | `main/boards/bread-compact-wifi-s3cam/compact_wifi_board_s3cam.cc` |

### 可调参数

```python
SP = 1.0     # 平移速度
DUR = 1.5    # 平移持续时间 (s)
TSP = 1.5    # 转弯速度
TDUR = 2.0   # 转弯持续时间 (s)
STD = 3.0    # StandUp 等待时间 (s) — 给足时间让狗完全站好
```

**姿态追踪**: `DOG_STANDING` 全局标志位
- `ensure_stand()` 先检查: 若 `True` 直接返回 (不调 StandUp, 不 sleep)
- "站立模式" → `True`, "卧倒/蹲下/解除锁定" → `False`

**状态机 Cooldown**: 1.0s (从 2.0s 缩短, 165ms 多帧 toggle 保证不重复触发, 同时更快响应连续命令)

---

## ESP32 编译

```bash
cd xiaozhi-esp32-2.2.4
idf.py set-target esp32s3
idf.py menuconfig  # Board → 面包板新版接线(WiFi)+LCD+Camera
idf.py build flash monitor
```
