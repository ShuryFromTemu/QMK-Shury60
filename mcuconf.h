#pragma once
#include_next "mcuconf.h"

//启用IIC1接口
#undef STM32_I2C_USE_I2C1
#define STM32_I2C_USE_I2C1 TRUE

// F072 I2C1时钟源，绝大多数QMK F0工程选HSI（8M内部RC）
#define STM32_I2C_I2C1_CLK        STM32_I2CCLK_HSI
// I2C timeout，F0不加这个也会启动失败
#define STM32_I2C_I2C1_TIMEOUT    10000