#pragma once
#include_next "mcuconf.h"

//启用IIC1接口
#undef STM32_I2C_USE_I2C1
#define STM32_I2C_USE_I2C1 TRUE

//启用IIC DMA功能
#undef STM32_I2C_USE_DMA
#define STM32_I2C_USE_DMA TRUE