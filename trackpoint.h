//
// Created by wwwqi on 2026/1/28.
//

#ifndef TPTEST_TRACKPOINT_H
#define TPTEST_TRACKPOINT_H


#include "stm32f4xx_hal.h"
#include "usbd_hid.h"


#define TRACKPOINT_I2C_ADDRESS         0x15  // 7-bit address
#define TRACKPOINT_WRITE_ADDRESS       ((TRACKPOINT_I2C_ADDRESS << 1) & 0xFE) // 8-bit write address
#define TRACKPOINT_READ_ADDRESS        ((TRACKPOINT_I2C_ADDRESS << 1) | 0x01)  // 8-bit read address
#define TRACKPOINT_I2C_TIMEOUT         100

// 数据包结构
typedef struct {
    uint8_t buttons;        // 按钮状态(低3位表示)
    int8_t delta_x;         // X方向相对位移
    int8_t delta_y;         // Y方向相对位移
    uint8_t reserved3;
} TrackPoint_DataPacket;

// 设备状态
typedef struct {
    I2C_HandleTypeDef *hi2c;
    TrackPoint_DataPacket data;
    uint8_t is_data_ready;
} TrackPoint_HandleTypeDef;

// 函数声明
HAL_StatusTypeDef TrackPoint_Init(TrackPoint_HandleTypeDef *htp,
                                  I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef TrackPoint_ReadData(TrackPoint_HandleTypeDef *htp);
HAL_StatusTypeDef TrackPoint_Sleep(TrackPoint_HandleTypeDef *htp);
HAL_StatusTypeDef TrackPoint_Wakeup(TrackPoint_HandleTypeDef *htp);
HAL_StatusTypeDef TrackPoint_Reset(TrackPoint_HandleTypeDef *htp);
HAL_StatusTypeDef TrackPoint_SetSpeed(TrackPoint_HandleTypeDef *htp,
                                      uint8_t up_speed, uint8_t down_speed,
                                      uint8_t left_speed, uint8_t right_speed);
void TrackPoint_GPIO_EXTI_Callback(TrackPoint_HandleTypeDef *htp);
uint8_t TrackPoint_IsDataReady(TrackPoint_HandleTypeDef *htp);
HAL_StatusTypeDef TrackPoint_SendHidReport(TrackPoint_HandleTypeDef *htp, USBD_HandleTypeDef *hdref);

#endif //TPTEST_TRACKPOINT_H