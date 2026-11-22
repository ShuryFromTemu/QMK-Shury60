// Copyright 2025 shury
// SPDX-License-Identifier: GPL-2.0-or-later
//
// TrackPoint (I2C / TPM 协议) 驱动 —— QMK custom pointing device driver 实现
// 移植自 F072TP 的 HAL 版本，协议、命令、时序保持一致。
//
// 关于 I2C 传输层（重要）：
//   QMK 在 STM32 上默认走 platforms/chibios/drivers/i2c_master.c -> ChibiOS
//   I2Cv2 LLD，而 ChibiOS 的 I2C 在 F072 这个板级配置里是 DMA + 中断模式
//   (STM32_I2C_USE_DMA TRUE)，每次传输出错还会 i2cStop() 把外设关掉。
//   实测那条路径在 F072 上所有传输都等不到任何事件（纯超时、无 NACK），
//   所以这里干脆不用它：直接按 STM32F0 参考手册 / HAL 的做法操作 I2C1 寄存器
//   （轮询 TXIS/RXNE/STOPF，AUTOEND），和原来跑通的 HAL 版一一对应。
//   好处是行为完全可预期：谁都没开中断、没开 DMA、没有状态机。

#include "trackpoint.h"

#include "gpio.h"
#include "wait.h"
#include "timer.h"
#include "pointing_device.h"
#include "hal.h"

#ifdef CONSOLE_ENABLE
#    include "print.h"
#    include "usb_util.h"
#    define tp_dprintf(...) uprintf(__VA_ARGS__)
#else
#    define tp_dprintf(...) \
        do {                 \
        } while (0)
#endif

/* ================= I2C 底层（寄存器直操） ================= */

#ifndef I2C1_SCL_PIN
#    define I2C1_SCL_PIN B6
#endif
#ifndef I2C1_SDA_PIN
#    define I2C1_SDA_PIN B7
#endif
#ifndef I2C1_SCL_PAL_MODE
#    define I2C1_SCL_PAL_MODE 1  // STM32F0: I2C1 是 AF1
#endif
#ifndef I2C1_SDA_PAL_MODE
#    define I2C1_SDA_PAL_MODE 1
#endif

// 和 config.h 里 CubeMX 算出来的值一致（I2CCLK = HSI 8MHz，100kHz）
#ifndef I2C1_TIMINGR_PRESC
#    define I2C1_TIMINGR_PRESC 0U
#endif
#ifndef I2C1_TIMINGR_SCLDEL
#    define I2C1_TIMINGR_SCLDEL 2U
#endif
#ifndef I2C1_TIMINGR_SDADEL
#    define I2C1_TIMINGR_SDADEL 0U
#endif
#ifndef I2C1_TIMINGR_SCLH
#    define I2C1_TIMINGR_SCLH 29U
#endif
#ifndef I2C1_TIMINGR_SCLL
#    define I2C1_TIMINGR_SCLL 43U
#endif

#define TP_I2C I2C1

#define TP_TIMINGR                                                                                                        \
    (STM32_TIMINGR_PRESC(I2C1_TIMINGR_PRESC) | STM32_TIMINGR_SCLDEL(I2C1_TIMINGR_SCLDEL) |                                \
     STM32_TIMINGR_SDADEL(I2C1_TIMINGR_SDADEL) | STM32_TIMINGR_SCLH(I2C1_TIMINGR_SCLH) | STM32_TIMINGR_SCLL(I2C1_TIMINGR_SCLL))

// ICR 里所有能清的中断标志（ADDRCF/NACKCF/STOPCF/BERRCF/ARLOCF/OVRCF/
// PECERRCF/TIMOUTCF/ALERTCF = 0x00003F38；F0 的 CMSIS 头里个别位名拼写不一致，
// 直接用数值，等价于 CMSIS 的 I2C_ICR_CLEAR_MASK
#define TP_ICR_CLEAR 0x00003F38U

// ISR 里的错误位
#define TP_ISR_ERRORS                                                                                                     \
    (I2C_ISR_NACKF | I2C_ISR_BERR | I2C_ISR_ARLO | I2C_ISR_OVR | I2C_ISR_PECERR | I2C_ISR_TIMEOUT | I2C_ISR_ALERT)

typedef enum {
    TP_XFER_OK = 0,
    TP_XFER_TIMEOUT = 1,
    TP_XFER_NACK = 2,
    TP_XFER_ERROR = 3,
} tp_xfer_status_t;

static uint32_t trackpoint_errors = 0;

// 诊断用：初始化结果 + 最近的数据统计
static uint8_t  trackpoint_state_code = 0;
static uint32_t trackpoint_packets    = 0;  // 有效数据包（有位移或按键变化）
static uint32_t trackpoint_empty      = 0;  // 空闲时的"空包"（dx=dy=0 且按键没变）
static uint32_t trackpoint_events     = 0;  // 供 console 判断"有没有事情发生"
static uint32_t trackpoint_read_fail  = 0;
static int8_t   trackpoint_last_dx    = 0;
static int8_t   trackpoint_last_dy    = 0;
static uint8_t  trackpoint_last_btn   = 0;

// 诊断用：最后一次失败的快照
static uint32_t tp_last_isr   = 0;
static uint32_t tp_last_cr2   = 0;
static uint8_t  tp_last_code  = 0;
static uint8_t  tp_bus_low_at_init = 0;  // bit0=SCL 被拉低, bit1=SDA 被拉低

uint8_t trackpoint_state(void) {
    return trackpoint_state_code;
}

static void trackpoint_bus_recover(void);

// 把 SCL/SDA 交还给 I2C 复用功能（开漏 + 最高速度，和 CubeMX 里的 GPIO_SPEED_FREQ_HIGH 对应）
static void trackpoint_i2c_pins_af(void) {
    palSetLineMode(I2C1_SCL_PIN, PAL_MODE_ALTERNATE(I2C1_SCL_PAL_MODE) | PAL_OUTPUT_TYPE_OPENDRAIN | PAL_STM32_OSPEED_HIGHEST);
    palSetLineMode(I2C1_SDA_PIN, PAL_MODE_ALTERNATE(I2C1_SDA_PAL_MODE) | PAL_OUTPUT_TYPE_OPENDRAIN | PAL_STM32_OSPEED_HIGHEST);
}

// 把 SCL/SDA 变成带内部上拉的输入，读总线电平（1 = 高 = 上拉正常/总线空闲）
static uint8_t trackpoint_i2c_read_bus(void) {
    uint8_t bits = 0;

    palSetLineMode(I2C1_SCL_PIN, PAL_MODE_INPUT_PULLUP);
    palSetLineMode(I2C1_SDA_PIN, PAL_MODE_INPUT_PULLUP);
    wait_us(200);

    if (palReadLine(I2C1_SCL_PIN)) {
        bits |= 0x01;
    }
    if (palReadLine(I2C1_SDA_PIN)) {
        bits |= 0x02;
    }

    trackpoint_i2c_pins_af();
    return bits;
}

// 外设初始化：等价于 HAL_I2C_Init()（HSI 时钟源由板级 hal_lld 设好）
static void trackpoint_i2c_hw_init(void) {
    TP_I2C->CR1 &= ~I2C_CR1_PE;  // 改 TIMINGR 前必须先关 PE
    rccEnableI2C1(true);         // APB1 时钟
    rccResetI2C1();              // 复位外设寄存器，清掉可能残留的状态

    TP_I2C->CR1 = 0;             // ANFOFF = 0：保持模拟滤波器打开（HAL 版就是 ENABLE）
    TP_I2C->TIMINGR = TP_TIMINGR;
    TP_I2C->CR1 = I2C_CR1_PE;    // 使能外设
    TP_I2C->ICR = TP_ICR_CLEAR;
}

/**
 * 轮询等待某个 ISR 位，同时盯着 NACK / 总线错误 / 超时。
 * 返回 false 时 *result 里是失败原因。
 */
static bool trackpoint_i2c_wait_flag(uint32_t flag, tp_xfer_status_t *result) {
    uint32_t start = timer_read32();

    while (true) {
        uint32_t isr = TP_I2C->ISR;

        if ((isr & TP_ISR_ERRORS) != 0) {
            tp_last_isr  = isr;
            *result      = (isr & I2C_ISR_NACKF) ? TP_XFER_NACK : TP_XFER_ERROR;
            return false;
        }
        if ((isr & flag) != 0) {
            return true;
        }
        if (timer_elapsed32(start) > TRACKPOINT_I2C_TIMEOUT) {
            tp_last_isr = isr;
            *result     = TP_XFER_TIMEOUT;
            return false;
        }
    }
}

/**
 * 一次主机传输（写或读），和 HAL_I2C_Master_Transmit / Master_Receive 的
 * 轮询流程一致：等 BUSY 清 -> 写 CR2(NBYTES+AUTOEND+START) -> 逐字节搬
 * TXIS/RXNE -> 等 STOPF。
 *
 * AUTOEND 交给硬件在 NBYTES 传完后自动发 STOP，所以中间不需要干预。
 */
static tp_xfer_status_t trackpoint_i2c_xfer(uint8_t addr7, const uint8_t *tx, uint16_t txlen, uint8_t *rx, uint16_t rxlen) {
    uint32_t          start = timer_read32();
    uint32_t          isr;
    uint32_t          cr2;
    uint16_t          i;
    tp_xfer_status_t  result = TP_XFER_OK;

    // 1. 等上一次事务彻底结束（BUSY = 0）
    while ((TP_I2C->ISR & I2C_ISR_BUSY) != 0) {
        if (timer_elapsed32(start) > TRACKPOINT_I2C_TIMEOUT) {
            tp_last_isr = TP_I2C->ISR;
            return TP_XFER_TIMEOUT;
        }
    }

    // 2. 清掉所有历史标志，免得误判 NACK
    TP_I2C->ICR = TP_ICR_CLEAR;

    // 3. 组 CR2：7 位地址 + NBYTES + AUTOEND（读的时候再加 RD_WRN）
    uint32_t nbytes = (rx != NULL) ? rxlen : txlen;
    cr2             = ((uint32_t)addr7 << 1U) | I2C_CR2_AUTOEND | ((uint32_t)nbytes << 16U);
    if (rx != NULL) {
        cr2 |= I2C_CR2_RD_WRN;
    }
    cr2 |= I2C_CR2_START;
    tp_last_cr2 = cr2;
    TP_I2C->CR2 = cr2;

    // 4. 搬数据（顺序、判断条件和 HAL 完全一样）
    if (rx != NULL) {
        for (i = 0; i < rxlen; i++) {
            if (!trackpoint_i2c_wait_flag(I2C_ISR_RXNE, &result)) {
                break;
            }
            rx[i] = TP_I2C->RXDR;
        }
    } else {
        for (i = 0; i < txlen; i++) {
            if (!trackpoint_i2c_wait_flag(I2C_ISR_TXIS, &result)) {
                break;
            }
            TP_I2C->TXDR = tx[i];
        }
    }

    // 5. 等 STOPF（AUTOEND 硬件自动发的 STOP）
    if (result == TP_XFER_OK) {
        if (!trackpoint_i2c_wait_flag(I2C_ISR_STOPF, &result)) {
            // NACK/错误在最后一步才被看到
        }
    }

    TP_I2C->ICR = TP_ICR_CLEAR;

    // 6. 出错时兜底：总线可能还挂着，手动补一个 STOP 并等它释放
    if (result != TP_XFER_OK) {
        isr = TP_I2C->ISR;
        if ((isr & I2C_ISR_BUSY) != 0) {
            TP_I2C->CR2 |= I2C_CR2_STOP;
            start = timer_read32();
            while ((TP_I2C->ISR & I2C_ISR_BUSY) != 0 && timer_elapsed32(start) < 5) {
            }
            TP_I2C->ICR = TP_ICR_CLEAR;
        }
        tp_last_code = (uint8_t)result;
    }

    return result;
}

// 发一条写命令给 TrackPoint（参数是 HAL 风格的 8 位地址）
static bool trackpoint_write(uint8_t addr8, const uint8_t *cmd, uint16_t len) {
    tp_xfer_status_t status = trackpoint_i2c_xfer(addr8 >> 1, cmd, len, NULL, 0);

    if (status != TP_XFER_OK) {
        if (++trackpoint_errors >= TRACKPOINT_ERR_LIMIT) {
            trackpoint_bus_recover();
        }
        return false;
    }

    trackpoint_errors = 0;
    return true;
}

/**
 * I2C 总线恢复。
 *
 * 从机如果因为干扰把 SDA 一直拉低（clock stretching 卡死），只复位外设是
 * 拉不回来的：这里手动打 9 个 SCL 时钟让从机把剩余的位移完，再重新初始化
 * 外设和引脚复用。
 */
static void trackpoint_bus_recover(void) {
    // SDA 放成输入（靠上拉），SCL 开漏输出，靠打时钟解锁从机
    palSetLineMode(I2C1_SDA_PIN, PAL_MODE_INPUT_PULLUP);
    palSetLineMode(I2C1_SCL_PIN, PAL_MODE_OUTPUT_OPENDRAIN);
    wait_us(10);

    for (uint8_t i = 0; i < 9; i++) {
        palClearLine(I2C1_SCL_PIN);
        wait_us(5);
        palSetLine(I2C1_SCL_PIN);
        wait_us(5);
    }

    // 手动产生一个 STOP 条件（SCL 高时 SDA 由低变高）
    palSetLine(I2C1_SCL_PIN);
    wait_us(5);
    palSetLineMode(I2C1_SDA_PIN, PAL_MODE_OUTPUT_OPENDRAIN);
    palClearLine(I2C1_SDA_PIN);
    wait_us(5);
    palSetLine(I2C1_SDA_PIN);
    wait_us(5);

    // 引脚交还 I2C，外设重新初始化（PE 关->TIMINGR->PE）
    trackpoint_i2c_pins_af();
    trackpoint_i2c_hw_init();
    wait_ms(2);

    trackpoint_errors = 0;
    tp_dprintf("trackpoint: i2c bus recovered\n");
}

/* ================= TrackPoint 命令 ================= */

bool trackpoint_reset(void) {
    static const uint8_t cmd[] = {0x25, 0x00, 0x06, 0x00, 0x29, 0x77, 0x77, 0x77};
    return trackpoint_write(TRACKPOINT_WRITE_ADDRESS, cmd, sizeof(cmd));
}

bool trackpoint_set_speed(uint8_t up_speed, uint8_t down_speed, uint8_t left_speed, uint8_t right_speed) {
    // 上下速度
    uint8_t cmd_ud[] = {0x25, 0x00, 0x06, 0x00, 0x29, 0x42, up_speed, down_speed};
    if (!trackpoint_write(TRACKPOINT_WRITE_ADDRESS, cmd_ud, sizeof(cmd_ud))) {
        return false;
    }
    wait_ms(10);

    // 左右速度
    uint8_t cmd_lr[] = {0x25, 0x00, 0x06, 0x00, 0x29, 0x43, left_speed, right_speed};
    return trackpoint_write(TRACKPOINT_WRITE_ADDRESS, cmd_lr, sizeof(cmd_lr));
}

bool trackpoint_sleep(void) {
    static const uint8_t cmd[] = {0x22, 0x00, 0x01, 0x08};
    return trackpoint_write(TRACKPOINT_WRITE_ADDRESS, cmd, sizeof(cmd));
}

bool trackpoint_wakeup(void) {
    static const uint8_t cmd[] = {0x22, 0x00, 0x00, 0x08};
    return trackpoint_write(TRACKPOINT_WRITE_ADDRESS, cmd, sizeof(cmd));
}

/* ================= 读数据 ================= */

uint8_t trackpoint_data_available(void) {
    // PC13 被 QMK 的 pointing_device_init() 配成上拉输入，低电平 = 有新数据
    return gpio_read_pin(TRACKPOINT_DR_PIN) ? 0 : 1;
}

bool trackpoint_read(trackpoint_data_t *out) {
    uint8_t packet[TRACKPOINT_PACKET_SIZE];

    tp_xfer_status_t status = trackpoint_i2c_xfer(TRACKPOINT_READ_ADDRESS >> 1, NULL, 0, packet, sizeof(packet));

    if (status != TP_XFER_OK) {
        trackpoint_read_fail++;
        if (++trackpoint_errors >= TRACKPOINT_ERR_LIMIT) {
            trackpoint_bus_recover();
        }
        return false;
    }
    trackpoint_errors = 0;

    // 验证数据包头：07 00 01
    if (packet[0] != 0x07 || packet[1] != 0x00 || packet[2] != 0x01) {
        trackpoint_read_fail++;
        return false;
    }

    if (out) {
        out->buttons = packet[3] & 0x07;
        out->dx      = (int8_t)packet[4];
        out->dy      = (int8_t)packet[5];
    }

    return true;
}

/* ================= 初始化 ================= */

bool trackpoint_init(void) {
    // 先把引脚从复用功能里放出来，看看总线是不是真的能拉高（缺上拉 / 被拉死在这里就能看出来）
    palSetLineMode(I2C1_SCL_PIN, PAL_MODE_INPUT_PULLUP);
    palSetLineMode(I2C1_SDA_PIN, PAL_MODE_INPUT_PULLUP);
    wait_us(500);
    tp_bus_low_at_init = 0;
    if (!palReadLine(I2C1_SCL_PIN)) {
        tp_bus_low_at_init |= 0x01;
    }
    if (!palReadLine(I2C1_SDA_PIN)) {
        tp_bus_low_at_init |= 0x02;
    }

    trackpoint_i2c_pins_af();
    trackpoint_i2c_hw_init();

    // 手册要求上电后等待
    wait_ms(150);

    // 复位（带重试）
    bool reset_ok = false;
    for (uint16_t i = 0; i < TRACKPOINT_INIT_RETRIES; i++) {
        if (trackpoint_reset()) {
            reset_ok = true;
            break;
        }
        wait_ms(20);
    }
    if (!reset_ok) {
        tp_dprintf("trackpoint: reset failed (code=%u isr=0x%08lx cr2=0x%08lx)\n", tp_last_code, (unsigned long)tp_last_isr, (unsigned long)tp_last_cr2);
        trackpoint_state_code = 2;
        return false;
    }

    wait_ms(20);

    // 设置指针速度（带重试）
    bool speed_ok = false;
    for (uint16_t i = 0; i < TRACKPOINT_INIT_RETRIES; i++) {
        if (trackpoint_set_speed(TRACKPOINT_SPEED, TRACKPOINT_SPEED, TRACKPOINT_SPEED, TRACKPOINT_SPEED)) {
            speed_ok = true;
            break;
        }
        wait_ms(20);
    }
    if (!speed_ok) {
        tp_dprintf("trackpoint: set speed failed (code=%u isr=0x%08lx)\n", tp_last_code, (unsigned long)tp_last_isr);
        trackpoint_state_code = 3;
        return false;
    }

    // 清掉复位后残留的第一次数据（这一步失败属于正常现象：复位后 TP 还没吐新包，
    // 所以顺手把统计清零，免得 console 里永远挂着 read_fail=1）
    trackpoint_data_t dummy;
    trackpoint_read(&dummy);

    trackpoint_read_fail = 0;
    trackpoint_errors    = 0;
    trackpoint_state_code = 1;
    return true;
}

/* ================= QMK pointing device driver ================= */

bool pointing_device_driver_init(void) {
    return trackpoint_init();
}

/**
 * 把 TrackPoint 的位移和按键交给 QMK。
 *
 * 原来的 HAL 版是自己攒位移、自己发 USBD_HID 报告；到 QMK 这边这些都不用管了：
 * get_report 只需要返回"本次的位移 + 当前按键"，累积 / 打包 / 发送 / 与
 * mousekey 合并都由 pointing_device 统一处理。
 */
report_mouse_t pointing_device_driver_get_report(report_mouse_t mouse_report) {
    int16_t  acc_x    = 0;
    int16_t  acc_y    = 0;
    uint8_t  buttons  = mouse_report.buttons;
    uint8_t  packets  = 0;
    bool     got_data = false;

    // DR 低就说明还有数据，一次任务里尽量把缓存读空（读失败立刻退出，别死等）
    while (packets < TRACKPOINT_MAX_PACKETS_PER_TASK && trackpoint_data_available()) {
        trackpoint_data_t data;
        if (!trackpoint_read(&data)) {
            break;
        }
        packets++;

        // TrackPoint 空闲时会周期性吐"空包"（dx=dy=0、按键没变）。这些包只是
        // 保活/心跳，读掉就行，不需要生成 HID 报告，也不该算成"有动作"。
        if (data.dx == 0 && data.dy == 0 && data.buttons == buttons) {
            trackpoint_empty++;
            continue;
        }

        got_data = true;
        acc_x += data.dx;
        acc_y += data.dy;
        buttons = data.buttons;

        trackpoint_packets++;
        trackpoint_events++;
        trackpoint_last_dx  = data.dx;
        trackpoint_last_dy  = data.dy;
        trackpoint_last_btn = data.buttons;
    }

    if (!got_data) {
        // 只有空包或者没有新数据，报告保持原样（不产生无意义的 HID 上报）
        return mouse_report;
    }

    // 一帧里读到的位移可能超过 int8 范围，按 HID 能力截断（TrackPoint 单包最大 ±127，
    // 实际使用中几乎不会触发截断）
    mouse_report.x       = CONSTRAIN_HID_XY(acc_x);
    mouse_report.y       = CONSTRAIN_HID_XY(acc_y);
    mouse_report.buttons = buttons;

    return mouse_report;
}

/* ================= 运行时补救 + console 诊断 ================= */

/**
 * 开机那次初始化如果失败（比如上电时序没跟上），主循环里定期重试。
 * TrackPoint 复位失败时 QMK 会把 pointing_device 状态置成 INIT_FAILED，
 * 那样 get_report() 永远不会被调用，重试成功后要把状态改回 SUCCESS。
 * 这样即使首次初始化失败，插上后过几秒也能自己恢复。
 */
static void trackpoint_runtime_retry(void) {
    static uint32_t last_try   = 0;
    static uint8_t  try_count  = 0;

    if (trackpoint_state_code == 1) {
        return;
    }
    if (try_count >= 10) {  // 最多补 10 次，之后就等重新插拔
        return;
    }
    if (last_try != 0 && timer_elapsed32(last_try) < 3000) {
        return;
    }
    last_try = timer_read32();
    try_count++;

    if (trackpoint_init()) {
        trackpoint_state_code = 1;
        pointing_device_set_status(POINTING_DEVICE_STATUS_SUCCESS);
    }
}

#ifdef CONSOLE_ENABLE
// 一次性自检：把"可能出错的几个点"（时钟源 / 外设使能 / TIMINGR / 引脚 / 总线电平）
// 压成一行。全部正常时就是一行 ok，不正常才把原始寄存器打出来。
static void trackpoint_dump_diag(void) {
    uint8_t bus     = trackpoint_i2c_read_bus();
    bool    clk_hsi = (RCC->CFGR3 & (1UL << 4UL)) == 0;  // CFGR3.I2C1SW：0 = HSI，1 = SYSCLK
    bool    pe      = (TP_I2C->CR1 & I2C_CR1_PE) != 0;
    bool    timing  = TP_I2C->TIMINGR == TP_TIMINGR;
    bool    pins    = ((GPIOB->MODER & 0xF000UL) == 0xA000UL) && ((GPIOB->AFRL & 0xFF000000UL) == 0x11000000UL) &&
                   ((GPIOB->OTYPER & 0x00C0UL) == 0x00C0UL);  // PB6/PB7 = AF1 开漏
    bool    bus_ok = (bus & 0x03) == 0x03;

    if (clk_hsi && pe && timing && pins && bus_ok) {
        uprintf("trackpoint: i2c ok (clk=HSI timingr=0x%08lx pins=PB6/PB7 AF1 OD bus=idle)\n", (unsigned long)TP_I2C->TIMINGR);
        return;
    }

    uprintf("trackpoint: i2c CHECK clk=%s pe=%u timingr=0x%08lx(expect 0x%08lx) pins=%s bus=scl%u/sda%u\n",
            clk_hsi ? "HSI" : "SYSCLK", (unsigned int)pe, (unsigned long)TP_I2C->TIMINGR, (unsigned long)TP_TIMINGR,
            pins ? "ok" : "BAD", (unsigned int)(bus & 0x01), (unsigned int)((bus >> 1) & 0x01));
    uprintf("trackpoint: i2c raw CFGR3=0x%08lx CR1=0x%08lx ISR=0x%08lx MODER=0x%08lx AFRL=0x%08lx OTYPER=0x%08lx OSPEEDR=0x%08lx PUPDR=0x%08lx\n",
            (unsigned long)RCC->CFGR3, (unsigned long)TP_I2C->CR1, (unsigned long)TP_I2C->ISR, (unsigned long)GPIOB->MODER,
            (unsigned long)GPIOB->AFRL, (unsigned long)GPIOB->OTYPER, (unsigned long)GPIOB->OSPEEDR, (unsigned long)GPIOB->PUPDR);
}
#endif

void housekeeping_task_kb(void) {
    trackpoint_runtime_retry();

#ifdef CONSOLE_ENABLE
    static bool     connected      = false;
    static bool     status_printed = false;
    static uint32_t start          = 0;
    static uint32_t last_print     = 0;
    static uint32_t printed_events = 0;
    static uint32_t printed_fail   = 0;

    if (!connected) {
        if (!usb_connected_state()) {
            return;
        }
        connected = true;
        start     = timer_read32();
        return;  // 先什么都别打
    }

    // 主机侧 console 工具在设备重新枚举后会晚一点才打开 HID 接口，
    // 连上瞬间打印会整批丢掉，先等 1 秒。
    if (timer_elapsed32(start) < 1000) {
        return;
    }

    if (!status_printed) {
        status_printed = true;
        uprintf("trackpoint: init %s\n", trackpoint_state_code == 1 ? "OK" : "FAILED");
        trackpoint_dump_diag();
        printed_events = trackpoint_events;
        printed_fail   = trackpoint_read_fail;
        last_print     = timer_read32();
        return;
    }

    // 只在"有事发生"（有真实位移/按键，或者读失败）时每秒打一行，空闲时完全安静。
    // 空包数量一直在涨，所以它不参与"有没有变化"的判断。
    if (last_print != 0 && timer_elapsed32(last_print) < 1000) {
        return;
    }
    if (trackpoint_events == printed_events && trackpoint_read_fail == printed_fail) {
        return;
    }
    last_print     = timer_read32();
    printed_events = trackpoint_events;
    printed_fail   = trackpoint_read_fail;

    uprintf("trackpoint: pkts=%lu empty=%lu fail=%lu last=(%d,%d) btn=%u\n", (unsigned long)trackpoint_packets,
            (unsigned long)trackpoint_empty, (unsigned long)trackpoint_read_fail, trackpoint_last_dx, trackpoint_last_dy,
            trackpoint_last_btn);
#endif
}
