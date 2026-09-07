#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/lcd_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "serial_commands.h"
#include "mcp_server.h"
#include "lamp_controller.h"
#include "led/single_led.h"
#include "esp32_camera.h"

#include <map>
#include <cstring>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_rom_sys.h>
#include <driver/i2c_master.h>
#include <driver/uart.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <driver/spi_common.h>

#if defined(LCD_TYPE_ILI9341_SERIAL)
#include "esp_lcd_ili9341.h"
#endif

#if defined(LCD_TYPE_GC9A01_SERIAL)
#include "esp_lcd_gc9a01.h"
static const gc9a01_lcd_init_cmd_t gc9107_lcd_init_cmds[] = {
    //  {cmd, { data }, data_size, delay_ms}
    {0xfe, (uint8_t[]){0x00}, 0, 0},
    {0xef, (uint8_t[]){0x00}, 0, 0},
    {0xb0, (uint8_t[]){0xc0}, 1, 0},
    {0xb1, (uint8_t[]){0x80}, 1, 0},
    {0xb2, (uint8_t[]){0x27}, 1, 0},
    {0xb3, (uint8_t[]){0x13}, 1, 0},
    {0xb6, (uint8_t[]){0x19}, 1, 0},
    {0xb7, (uint8_t[]){0x05}, 1, 0},
    {0xac, (uint8_t[]){0xc8}, 1, 0},
    {0xab, (uint8_t[]){0x0f}, 1, 0},
    {0x3a, (uint8_t[]){0x05}, 1, 0},
    {0xb4, (uint8_t[]){0x04}, 1, 0},
    {0xa8, (uint8_t[]){0x08}, 1, 0},
    {0xb8, (uint8_t[]){0x08}, 1, 0},
    {0xea, (uint8_t[]){0x02}, 1, 0},
    {0xe8, (uint8_t[]){0x2A}, 1, 0},
    {0xe9, (uint8_t[]){0x47}, 1, 0},
    {0xe7, (uint8_t[]){0x5f}, 1, 0},
    {0xc6, (uint8_t[]){0x21}, 1, 0},
    {0xc7, (uint8_t[]){0x15}, 1, 0},
    {0xf0,
    (uint8_t[]){0x1D, 0x38, 0x09, 0x4D, 0x92, 0x2F, 0x35, 0x52, 0x1E, 0x0C,
                0x04, 0x12, 0x14, 0x1f},
    14, 0},
    {0xf1,
    (uint8_t[]){0x16, 0x40, 0x1C, 0x54, 0xA9, 0x2D, 0x2E, 0x56, 0x10, 0x0D,
                0x0C, 0x1A, 0x14, 0x1E},
    14, 0},
    {0xf4, (uint8_t[]){0x00, 0x00, 0xFF}, 3, 0},
    {0xba, (uint8_t[]){0xFF, 0xFF}, 2, 0},
};
#endif
 
#define TAG "CompactWifiBoardS3Cam"

// ============================================================
// SBUS 硬件 UART 实现 (宇树 Go2 Air)
// 协议: 100000bps, 8E2, 信号反相, 25字节/帧, 16×11bit
// 参考: bfs::SbusTx (Arduino) — write()非阻塞, 帧连续排队
// ============================================================

static void sbus_pack(const uint16_t ch[16], uint8_t frame[25]) {
    frame[0] = 0x0F;
    memset(&frame[1], 0, 22);
    frame[23] = 0x00;
    frame[24] = 0x00;
    for (int i = 0; i < 16; i++) {
        uint16_t val = ch[i] & 0x07FF;
        int bit_pos = i * 11;
        int byte_idx = 1 + (bit_pos / 8);
        int shift = bit_pos % 8;
        frame[byte_idx]   |= (uint8_t)((val << shift) & 0xFF);
        frame[byte_idx+1] |= (uint8_t)((val >> (8 - shift)) & 0xFF);
        if (shift > 5)
            frame[byte_idx+2] |= (uint8_t)((val >> (16 - shift)) & 0xFF);
    }
}

// 非阻塞发送: 塞进 TX FIFO 立刻返回 (匹配 Arduino write())
static void sbus_write_frame(const uint16_t ch[16]) {
    uint8_t frame[25];
    sbus_pack(ch, frame);
    uart_write_bytes(SBUS_UART_PORT, (const char*)frame, 25);
}

// 阻塞发送 (仅心跳用, 防帧重叠)
static void sbus_write_frame_sync(const uint16_t ch[16]) {
    uint8_t frame[25];
    sbus_pack(ch, frame);
    uart_write_bytes(SBUS_UART_PORT, (const char*)frame, 25);
    uart_wait_tx_done(SBUS_UART_PORT, pdMS_TO_TICKS(10));
}

// --- 心跳: 模拟 Arduino loop() 持续发帧, ~5ms 一帧 ---
static esp_timer_handle_t sbus_timer = nullptr;
static uint16_t sbus_state[16];
static int      sbus_move_left = 0;
static uint16_t sbus_target_Lx = SBUS_NEUTRAL;
static uint16_t sbus_target_Ly = SBUS_NEUTRAL;
static uint16_t sbus_target_Rx = SBUS_NEUTRAL;
static uint16_t sbus_target_Ry = SBUS_NEUTRAL;
static bool     sbus_timer_paused = false;

static void sbus_state_reset() {
    for (int i = 0; i < 16; i++) sbus_state[i] = SBUS_NEUTRAL;
    sbus_state[4] = SBUS_NEUTRAL;
    sbus_state[5] = SBUS_LOW;     // CH6 默认 192 (Arduino 一致)
    sbus_state[6] = SBUS_LOW;     // CH7 默认 192 (Arduino 一致)
    sbus_state[8] = SBUS_CH9_FIXED;
    sbus_state[9] = SBUS_CH10_FIXED;
    sbus_move_left = 0;
}

static uint16_t sbus_lerp(uint16_t cur, uint16_t target, int step) {
    if (step <= 0 || cur == target) return target;
    int diff = (int)target - (int)cur;
    if (abs(diff) <= step) return target;
    return cur + (diff > 0 ? step : -step);
}

static void sbus_timer_cb(void* arg) {
    if (sbus_timer_paused) return;

    if (sbus_move_left > 0) {
        sbus_state[0] = sbus_lerp(sbus_state[0], sbus_target_Rx, 80);
        sbus_state[1] = sbus_lerp(sbus_state[1], sbus_target_Ry, 80);
        sbus_state[2] = sbus_lerp(sbus_state[2], sbus_target_Ly, 80);
        sbus_state[3] = sbus_lerp(sbus_state[3], sbus_target_Lx, 80);
        sbus_move_left--;
    } else {
        sbus_state[0] = sbus_lerp(sbus_state[0], SBUS_NEUTRAL, 80);
        sbus_state[1] = sbus_lerp(sbus_state[1], SBUS_NEUTRAL, 80);
        sbus_state[2] = sbus_lerp(sbus_state[2], SBUS_NEUTRAL, 80);
        sbus_state[3] = sbus_lerp(sbus_state[3], SBUS_NEUTRAL, 80);
    }
    sbus_write_frame_sync(sbus_state);  // 阻塞: 确保发完再等下一轮
}

static void sbus_heartbeat_start() {
    sbus_state_reset();
    esp_timer_create_args_t args = {};
    args.callback = sbus_timer_cb;
    args.dispatch_method = ESP_TIMER_TASK;
    args.name = "sbus_hb";
    esp_timer_create(&args, &sbus_timer);
    esp_timer_start_periodic(sbus_timer, 5000); // 5ms
}

// --- 多帧脉冲 Toggle ---
// 每状态发 TOGGLE_FRAMES 帧 (用 sync write 确保帧完再发下一帧)
// 100kbps 下每帧 ~3ms + 2.5ms 间隔 ≈ 5.5ms/帧, 3×10=30帧 ~165ms
// Go2 1s 滑动窗口: 10 帧足以保证 has_hi/has_lo 可靠检测
#define TOGGLE_FRAMES 10

static void sbus_send_toggle(int ch, uint16_t ch5,
                             uint16_t v1, uint16_t v2, uint16_t v3) {
    sbus_timer_paused = true;
    uint16_t backup   = sbus_state[ch];
    uint16_t backup5  = sbus_state[4];
    sbus_state[4] = ch5;

    // v1 (上下文帧)
    sbus_state[ch] = v1;
    for (int i = 0; i < TOGGLE_FRAMES; i++) {
        sbus_write_frame_sync(sbus_state);
        esp_rom_delay_us(2500);
    }

    // v2 (主脉冲 — 核心帧, 确保 Go2 可靠捕获)
    sbus_state[ch] = v2;
    for (int i = 0; i < TOGGLE_FRAMES; i++) {
        sbus_write_frame_sync(sbus_state);
        esp_rom_delay_us(2500);
    }

    // v3 (结束帧)
    sbus_state[ch] = v3;
    for (int i = 0; i < TOGGLE_FRAMES; i++) {
        sbus_write_frame_sync(sbus_state);
        esp_rom_delay_us(2500);
    }

    sbus_state[ch] = backup;
    sbus_state[4]  = backup5;
    sbus_timer_paused = false;
}

// --- 持续移动 ---
static void sbus_start_movement(uint16_t Lx, uint16_t Ly,
                                uint16_t Rx, uint16_t Ry, int duration_ms) {
    sbus_target_Lx = Lx;
    sbus_target_Ly = Ly;
    sbus_target_Rx = Rx;
    sbus_target_Ry = Ry;
    sbus_move_left = duration_ms / 5;  // 5ms per frame
}


class CompactWifiBoardS3Cam : public WifiBoard {
private:

    Button boot_button_;
    LcdDisplay* display_;
    Esp32Camera* camera_;

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片
        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
#if defined(LCD_TYPE_ILI9341_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel));
#elif defined(LCD_TYPE_GC9A01_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(panel_io, &panel_config, &panel));
        gc9a01_vendor_config_t gc9107_vendor_config = {
            .init_cmds = gc9107_lcd_init_cmds,
            .init_cmds_size = sizeof(gc9107_lcd_init_cmds) / sizeof(gc9a01_lcd_init_cmd_t),
        };        
#else
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));
#endif
        
        esp_lcd_panel_reset(panel);

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
#ifdef  LCD_TYPE_GC9A01_SERIAL
        panel_config.vendor_config = &gc9107_vendor_config;
#endif
        display_ = new SpiLcdDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    void InitializeCamera() {
        camera_config_t config = {};
        config.pin_d0 = CAMERA_PIN_D0;
        config.pin_d1 = CAMERA_PIN_D1;
        config.pin_d2 = CAMERA_PIN_D2;
        config.pin_d3 = CAMERA_PIN_D3;
        config.pin_d4 = CAMERA_PIN_D4;
        config.pin_d5 = CAMERA_PIN_D5;
        config.pin_d6 = CAMERA_PIN_D6;
        config.pin_d7 = CAMERA_PIN_D7;
        config.pin_xclk = CAMERA_PIN_XCLK;
        config.pin_pclk = CAMERA_PIN_PCLK;
        config.pin_vsync = CAMERA_PIN_VSYNC;
        config.pin_href = CAMERA_PIN_HREF;
        config.pin_sccb_sda = CAMERA_PIN_SIOD;
        config.pin_sccb_scl = CAMERA_PIN_SIOC;
        config.sccb_i2c_port = 0;
        config.pin_pwdn = CAMERA_PIN_PWDN;
        config.pin_reset = CAMERA_PIN_RESET;
        config.xclk_freq_hz = XCLK_FREQ_HZ;
        config.pixel_format = PIXFORMAT_RGB565;
        config.frame_size = FRAMESIZE_VGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
        camera_ = new Esp32Camera(config);
        camera_->SetHMirror(false);
         camera_->SetVFlip(1);//OV3360设置为1

    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

    void InitializeUart() {
        // SBUS 硬件 UART: 100000bps, 8E2, TX 反相
        uart_config_t cfg = {
            .baud_rate = SBUS_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_EVEN,
            .stop_bits = UART_STOP_BITS_2,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };
        ESP_ERROR_CHECK(uart_driver_install(SBUS_UART_PORT, 256, 0, 0, NULL, 0));
        ESP_ERROR_CHECK(uart_param_config(SBUS_UART_PORT, &cfg));
        ESP_ERROR_CHECK(uart_set_pin(SBUS_UART_PORT, SBUS_TX_PIN, SBUS_RX_PIN,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
        // SBUS 信号必须反相!
        ESP_ERROR_CHECK(uart_set_line_inverse(SBUS_UART_PORT, UART_SIGNAL_TXD_INV));
        ESP_LOGI(TAG, "SBUS UART ready: TX=GPIO%d RX=GPIO%d %dbps 8E2 inv",
                 SBUS_TX_PIN, SBUS_RX_PIN, SBUS_BAUD_RATE);
        // 启动持续心跳 (Go2 需要不间断信号)
        sbus_heartbeat_start();
    }

    void InitializeTools() {
        auto& mcp_server = McpServer::GetInstance();

        // 构建命令名列表 (给AI看)
        std::string cmd_list;
        std::map<std::string, int> cmd_index;  // name -> index
        int idx = 0;
        #define X(name) \
            cmd_list += "\n  - " + std::string(name); \
            cmd_index[name] = idx++;
        SERIAL_COMMANDS
        #undef X

        mcp_server.AddTool("self.serial.send_command",
            "通过SBUS串口控制宇树Go2机器狗。可用命令:"
            + cmd_list + "\n\n"
            "参数 `command`: 要执行的命令名称",
            PropertyList({ Property("command", kPropertyTypeString) }),
            [cmd_index](const PropertyList& properties) -> ReturnValue {
                std::string cmd = properties["command"].value<std::string>();

                // 精确匹配 / 模糊匹配
                auto it = cmd_index.find(cmd);
                if (it == cmd_index.end()) {
                    for (auto& kv : cmd_index) {
                        if (kv.first.find(cmd) != std::string::npos ||
                            cmd.find(kv.first) != std::string::npos) {
                            it = cmd_index.find(kv.first);
                            break;
                        }
                    }
                }
                if (it == cmd_index.end()) {
                    std::string list;
                    for (auto& kv : cmd_index) list += kv.first + ",";
                    throw std::runtime_error("未知命令: " + cmd + " 可用: " + list);
                }

                // 速度换算: extreme按MOVE_SPEED_PCT缩放到992之间, 实现渐变摇杆
                auto spd = [](uint16_t extreme) -> uint16_t {
                    if (extreme == SBUS_NEUTRAL) return SBUS_NEUTRAL;
                    int d = (int)extreme - (int)SBUS_NEUTRAL;
                    return (uint16_t)((int)SBUS_NEUTRAL + d * MOVE_SPEED_PCT / 100);
                };

                // ================================================
                // 命令 → SBUS 参数映射 (改 SBUS 值在这里改)
                // Toggle: 三帧脉冲    Move: 持续移动+渐变
                // 通道: 0=Rx 1=Ry 2=Ly 3=Lx 4=CH5 5=CH6 6=CH7 7=CH8
                // ================================================
                switch (it->second) {
                case 0: // 停下来 (软急停): CH5=992, 翻转CH7  LOW→HIGH→LOW
                    sbus_send_toggle(6, SBUS_NEUTRAL, SBUS_LOW, SBUS_HIGH, SBUS_LOW);
                    break;
                case 1: // 站立模式: CH5=992, 翻转CH8  MID→HIGH→LOW
                    sbus_send_toggle(7, SBUS_NEUTRAL, SBUS_NEUTRAL, SBUS_HIGH, SBUS_LOW);
                    break;
                case 2: // 卧倒: CH5=992, 翻转CH8  MID→LOW→MID
                    sbus_send_toggle(7, SBUS_NEUTRAL, SBUS_NEUTRAL, SBUS_LOW, SBUS_NEUTRAL);
                    break;
                case 3: // 蹲下: CH5=992, 翻转CH6  LOW→HIGH→LOW
                    sbus_send_toggle(5, SBUS_NEUTRAL, SBUS_LOW, SBUS_HIGH, SBUS_LOW);
                    break;
                case 4: // 跑步模式: CH5=1792, 翻转CH8  MID→HIGH→MID
                    sbus_send_toggle(7, SBUS_HIGH, SBUS_NEUTRAL, SBUS_HIGH, SBUS_NEUTRAL);
                    break;
                case 5: // 前进: Ly渐变到目标 (默认60%=1472)
                    sbus_start_movement(SBUS_NEUTRAL, spd(SBUS_HIGH),
                                        SBUS_NEUTRAL, SBUS_NEUTRAL, MOVE_DURATION_MS);
                    break;
                case 6: // 后退: Ly渐变到目标 (默认60%=512)
                    sbus_start_movement(SBUS_NEUTRAL, spd(SBUS_LOW),
                                        SBUS_NEUTRAL, SBUS_NEUTRAL, MOVE_DURATION_MS);
                    break;
                case 7: // 向左走: Lx渐变到左 (432~992, 默认60%=656)
                    sbus_start_movement(spd(SBUS_LX_LEFT), SBUS_NEUTRAL,
                                        SBUS_NEUTRAL, SBUS_NEUTRAL, MOVE_DURATION_MS);
                    break;
                case 8: // 向右走: Lx渐变到右 (992~1552, 默认60%=1328)
                    sbus_start_movement(spd(SBUS_LX_RIGHT), SBUS_NEUTRAL,
                                        SBUS_NEUTRAL, SBUS_NEUTRAL, MOVE_DURATION_MS);
                    break;
                case 9: // 向左转弯: Rx渐变转左
                    sbus_start_movement(SBUS_NEUTRAL, SBUS_NEUTRAL,
                                        spd(SBUS_LX_LEFT), SBUS_NEUTRAL, MOVE_DURATION_MS);
                    break;
                case 10: // 向右转弯: Rx渐变转右
                    sbus_start_movement(SBUS_NEUTRAL, SBUS_NEUTRAL,
                                        spd(SBUS_LX_RIGHT), SBUS_NEUTRAL, MOVE_DURATION_MS);
                    break;
                default:
                    throw std::runtime_error("命令未实现: " + cmd);
                }
                ESP_LOGI(TAG, "SBUS cmd: %s", cmd.c_str());
                return std::string("已发送SBUS命令: ") + cmd;
            });
    }

public:
    CompactWifiBoardS3Cam() :
        boot_button_(BOOT_BUTTON_GPIO),
        display_(nullptr) {
        // LCD 已禁用 (GPIO47/21 给 SBUS 用)
        // InitializeSpi();
        // InitializeLcdDisplay();
        InitializeButtons();
        InitializeCamera();
        InitializeUart();
        InitializeTools();

    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        if (display_ == nullptr) {
            static NoDisplay no_display;
            return &no_display;
        }
        return display_;
    }

    virtual Backlight* GetBacklight() override {
        return nullptr;  // 无 LCD, 无背光
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }
};

DECLARE_BOARD(CompactWifiBoardS3Cam);
