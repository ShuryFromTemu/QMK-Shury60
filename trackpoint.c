//
// Created by wwwqi on 2026/1/28.
//

// trackpoint.c

#include "trackpoint.h"

#define DEFAULT_SPEED 1

#define TRACKPOINT_INTERRUPT_PIN       GPIO_PIN_13
#define TRACKPOINT_INTERRUPT_PORT      GPIOC





void i2c_init(void)
{
    gpio_set_pin_input_mode(I2C1_SLC_PIN);
    gpio_set_pin_input_mode(I2C1_SDA_PIN);
    wait_ms(10);

    palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_PUPDR_PULLUP);
    palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_PUPDR_PULLUP);

    
}

// 私有函数声明
static HAL_StatusTypeDef TrackPoint_SendCommand(TrackPoint_HandleTypeDef *htp, uint8_t *cmd);

// 初始化TrackPoint设备
HAL_StatusTypeDef TrackPoint_Init(TrackPoint_HandleTypeDef *htp,
                                  I2C_HandleTypeDef *hi2c) {
    if (htp == NULL || hi2c == NULL) {
        return HAL_ERROR;
    }

    // 保存参数
    htp->hi2c = hi2c;
    htp->is_data_ready = 0;

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = TRACKPOINT_INTERRUPT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(TRACKPOINT_INTERRUPT_PORT, &GPIO_InitStruct);

    // 等待100ms（文档要求）
    HAL_Delay(150);

    // 重置设备
    HAL_StatusTypeDef statusA = HAL_ERROR;
    for (uint16_t i = 0; statusA != HAL_OK; i++)
    {
        statusA = TrackPoint_Reset(htp);
        HAL_Delay(20);
        if (i > 100)
        {
            break;
        }
    }

    //设定速度
    HAL_StatusTypeDef statusB = HAL_ERROR;
    for (uint16_t i = 0; statusB != HAL_OK; i++)
    {
        statusB = TrackPoint_SetSpeed(htp,
        DEFAULT_SPEED, DEFAULT_SPEED,
        DEFAULT_SPEED, DEFAULT_SPEED);
        HAL_Delay(20);
        if (i > 100)
        {
            break;
        }
    }

    if (statusA != HAL_OK || statusB != HAL_OK)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

// 读取TrackPoint数据（从I2C读取7字节数据包）
HAL_StatusTypeDef TrackPoint_ReadData(TrackPoint_HandleTypeDef *htp) {
    uint8_t packet[7];

    // 从I2C读取7字节数据
    HAL_StatusTypeDef status = HAL_I2C_Master_Receive(htp->hi2c,
                                                      TRACKPOINT_READ_ADDRESS,
                                                      packet,
                                                      sizeof(packet),
                                                      TRACKPOINT_I2C_TIMEOUT);

    if (status != HAL_OK) {
        return status;
    }

    // 验证数据包头
    if (packet[0] != 0x07 || packet[1] != 0x00 || packet[2] != 0x01) {
        return HAL_ERROR;
    }

    // 解析数据包
    //uint8_t header = packet[0];
    //uint8_t reserved1 = packet[1];
    //uint8_t reserved2 = packet[2];
    //uint8_t button_left = (packet[3] >> 0) & 0x01;
    //uint8_t button_right = (packet[3] >> 1) & 0x01;
    //uint8_t button_middle = (packet[3] >> 2) & 0x01;
    htp->data.delta_x = (int8_t)packet[4];
    htp->data.delta_y = (int8_t)packet[5];
    //uint8_t reserved3 = packet[6];

    htp->is_data_ready = 1;

    return HAL_OK;
}

// 发送命令到TrackPoint设备
static HAL_StatusTypeDef TrackPoint_SendCommand(TrackPoint_HandleTypeDef *htp, uint8_t *cmd) {
    return HAL_I2C_Master_Transmit(htp->hi2c,
                                  TRACKPOINT_WRITE_ADDRESS,
                                  cmd,
                                  sizeof(cmd),
                                  TRACKPOINT_I2C_TIMEOUT);
}

// 进入睡眠模式
HAL_StatusTypeDef TrackPoint_Sleep(TrackPoint_HandleTypeDef *htp) {
    uint8_t cmd[] = {0x22, 0x00, 0x01, 0x08};
    return TrackPoint_SendCommand(htp, cmd);
}

// 唤醒设备
HAL_StatusTypeDef TrackPoint_Wakeup(TrackPoint_HandleTypeDef *htp) {
    uint8_t cmd[] = {0x22, 0x00, 0x00, 0x08};
    return TrackPoint_SendCommand(htp, cmd);
}

// 重置设备
HAL_StatusTypeDef TrackPoint_Reset(TrackPoint_HandleTypeDef *htp) {
    uint8_t cmd[] = {0x25, 0x00, 0x06, 0x00, 0x29, 0x77, 0x77, 0x77};
    return TrackPoint_SendCommand(htp, cmd);
}

// 设置移动速度
HAL_StatusTypeDef TrackPoint_SetSpeed(TrackPoint_HandleTypeDef *htp,
                                     uint8_t up_speed, uint8_t down_speed,
                                     uint8_t left_speed, uint8_t right_speed) {
    HAL_StatusTypeDef status;

    // 设置上下速度
    uint8_t cmd_ud[] = {0x25, 0x00, 0x06, 0x00, 0x29, 0x42, up_speed, down_speed};
    status = TrackPoint_SendCommand(htp, cmd_ud);
    if (status != HAL_OK){return HAL_ERROR;}
    HAL_Delay(10);

    // 设置左右速度
    uint8_t cmd_lr[] = {0x25, 0x00, 0x06, 0x00, 0x29, 0x43, left_speed, right_speed};
    status = TrackPoint_SendCommand(htp, cmd_lr);
    return status;
}

// GPIO外部中断回调函数（在主程序的EXTI回调中调用）
void TrackPoint_GPIO_EXTI_Callback(TrackPoint_HandleTypeDef *htp) {
    // 读取数据
    TrackPoint_ReadData(htp);
}

// 检查数据是否准备好
uint8_t TrackPoint_IsDataReady(TrackPoint_HandleTypeDef *htp) {
    uint8_t ready = htp->is_data_ready;
    if (ready) {
        htp->is_data_ready = 0; // 读取后清除标志
    }
    return ready;
}

HAL_StatusTypeDef TrackPoint_SendHidReport(TrackPoint_HandleTypeDef* htp, USBD_HandleTypeDef* hdref)
{
    return USBD_HID_SendReport(hdref, (uint8_t*)&htp->data, sizeof(htp->data));
}
