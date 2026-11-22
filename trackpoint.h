// Copyright 2025 shury
// SPDX-License-Identifier: GPL-2.0-or-later
//
// TrackPoint (I2C / TPM 协议) —— QMK 版本
// 由已经跑通的 STM32F072C8T6 HAL 驱动 (F072TP) 移植而来，硬件完全一致：
//   * I2C1: PB6=SCL / PB7=SDA，HSI 时钟；传输层是 trackpoint.c 里直接操作
//     I2C1 寄存器的轮询实现（和 HAL 版一致），不走 QMK/ChibiOS 的 i2c_master
//   * 数据就绪(DR)引脚: PC13，低电平有效
//   * USB / HID 由 QMK 自己管理，驱动只负责"把位移和按键交给 pointing_device"
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* ---------------- TrackPoint I2C ---------------- */

#define TRACKPOINT_I2C_ADDRESS 0x15  // 7 位地址

// 注意：QMK 的 i2c_master(ChibiOS 后端) 要求传入 8 位（已左移）地址，
// 内部会再右移一位，所以这里保持和 HAL 版一致的 0x2A / 0x2B。
#define TRACKPOINT_WRITE_ADDRESS ((TRACKPOINT_I2C_ADDRESS << 1) & 0xFE)  // 0x2A
#define TRACKPOINT_READ_ADDRESS  ((TRACKPOINT_I2C_ADDRESS << 1) | 0x01)  // 0x2B

#ifndef TRACKPOINT_I2C_TIMEOUT
#    define TRACKPOINT_I2C_TIMEOUT 20  // ms（QMK 的 i2c 超时单位是毫秒）
#endif

/* ---------------- 数据就绪引脚 DR ---------------- */

// 默认跟 config.h 里的 POINTING_DEVICE_MOTION_PIN 保持一致（PC13，低有效）
#if defined(POINTING_DEVICE_MOTION_PIN)
#    define TRACKPOINT_DR_PIN POINTING_DEVICE_MOTION_PIN
#else
#    define TRACKPOINT_DR_PIN C13
#endif

/* ---------------- 可调参数 ---------------- */

// 指针速度（TrackPoint 原厂默认是 1，可调 0~8）
#ifndef TRACKPOINT_SPEED
#    define TRACKPOINT_SPEED 1
#endif

// 一次任务里最多连读几个数据包。每次读 7 字节在 100kHz 下约 0.7ms（轮询阻塞），
// 所以别设太大，否则会明显拖慢矩阵扫描；TrackPoint 一帧内的有效数据一般也就 1~2 包。
#ifndef TRACKPOINT_MAX_PACKETS_PER_TASK
#    define TRACKPOINT_MAX_PACKETS_PER_TASK 4
#endif

// 连续多少次 I2C 失败后做一次总线/外设恢复
#ifndef TRACKPOINT_ERR_LIMIT
#    define TRACKPOINT_ERR_LIMIT 5
#endif

// 初始化的重试次数（复位 / 设速度）
#ifndef TRACKPOINT_INIT_RETRIES
#    define TRACKPOINT_INIT_RETRIES 10
#endif

#define TRACKPOINT_PACKET_SIZE 7

/* ---------------- 数据结构 ---------------- */

typedef struct {
    uint8_t buttons;  // bit0 左键 / bit1 右键 / bit2 中键
    int8_t  dx;       // X 相对位移
    int8_t  dy;       // Y 相对位移
} trackpoint_data_t;

/* ---------------- 对外接口 ---------------- */

// 初始化（内部会调用 i2c_init()，等待上电、复位、设速度、清掉残留数据）
bool trackpoint_init(void);

// 初始化结果：0 = 还没跑过，1 = 成功，2 = 失败（可用于 console 诊断）
uint8_t trackpoint_state(void);

// 读一个 7 字节数据包，成功返回 true
bool trackpoint_read(trackpoint_data_t *out);

// DR 是否为低（true = 有新数据）
uint8_t trackpoint_data_available(void);

// 复位 / 设速度 / 睡眠 / 唤醒
bool trackpoint_reset(void);
bool trackpoint_set_speed(uint8_t up_speed, uint8_t down_speed, uint8_t left_speed, uint8_t right_speed);
bool trackpoint_sleep(void);
bool trackpoint_wakeup(void);
