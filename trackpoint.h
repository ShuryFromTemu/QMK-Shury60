//
// Created by wwwqi on 2026/1/28.
//
#pragma once
#include "pointing_device.h"
#include "i2c_master.h"
#include "timer.h"
#include "gpio.h"
#include <stdint.h>

#define TRACKPOINT_I2C_ADDRESS         0x15  // 7-bit address
#define TRACKPOINT_I2C_TIMEOUT         100

#define I2C1_SLC_PIN  B6
#define I2C1_SDA_PIN  B7

void i2c_init(void);


// 数据包结构
typedef struct {
    uint8_t buttons;        // 按钮状态(低3位表示)
    int8_t delta_x;         // X方向相对位移
    int8_t delta_y;         // Y方向相对位移
    uint8_t reserved3;
} TrackPoint_DataPacket;

// 设备状态
typedef struct {
    TrackPoint_DataPacket data;
    uint8_t is_data_ready;
} TrackPoint_HandleTypeDef;

// 函数声明
bool TrackPoint_Init(TrackPoint_HandleTypeDef *htp);
bool TrackPoint_ReadData(TrackPoint_HandleTypeDef *htp);
bool TrackPoint_Sleep(TrackPoint_HandleTypeDef *htp);
bool TrackPoint_Wakeup(TrackPoint_HandleTypeDef *htp);
bool TrackPoint_Reset(TrackPoint_HandleTypeDef *htp);
bool TrackPoint_SetSpeed(TrackPoint_HandleTypeDef *htp,
                                      uint8_t up_speed, uint8_t down_speed,
                                      uint8_t left_speed, uint8_t right_speed);
void TrackPoint_GPIO_EXTI_Callback(TrackPoint_HandleTypeDef *htp);
uint8_t TrackPoint_IsDataReady(TrackPoint_HandleTypeDef *htp);
bool TrackPoint_SendHidReport(TrackPoint_HandleTypeDef *htp);
