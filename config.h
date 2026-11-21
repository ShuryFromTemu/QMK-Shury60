// I2C 设置，参考：
// https://docs.qmk.fm/drivers/i2c#arm-configuration

// 指定使用IIC1接口
#define I2C_DRIVER I2CD1

// 指定使用IIC1接口的SCL和SDA引脚
#define I2C1_SCL_PIN  B6
#define I2C1_SDA_PIN  B7

// 以下为用STM32CubeMX算出来：
// 无外部晶振，I2CCLK=8MHz，100kHz，TIMEMINGR：0x00201D2B
#define I2C1_TIMINGR_PRESC     0U
#define I2C1_TIMINGR_SCLDEL    2U
#define I2C1_TIMINGR_SDADEL    0U
#define I2C1_TIMINGR_SCLH      29U
#define I2C1_TIMINGR_SCLL      43U


// 指定使用IIC1接口的SCL和SDA引脚的PAL模式，简单来说就是设置引脚的功能模式为IIC模式
// 对于STM32F072来说，I2C1应该是AF1
#define I2C1_SCL_PAL_MODE 1
#define I2C1_SDA_PAL_MODE 1

// 指定使用IIC1接口的SCL和SDA引脚的速度，单位为Hz，这里设置为100kHz
#define F_SCL 100000UL


// Pointing Device设置，参考：
// https://docs.qmk.fm/features/pointing_device#common-configuration

// 指定TrackPoint的中断引脚，这里使用C13引脚
#define TRACKPOINT_INT_PIN  C13
// 指定使用IIC1接口
#define I2C_DRIVER I2CD1;

// 指定使用IIC1接口的SCL和SDA引脚
#define I2C1_SLC_PIN  B6
#define I2C1_SDA_PIN  B7

// 指定使用IIC1接口的SCL和SDA引脚的PAL模式，简单来说就是设置引脚的功能模式为IIC模式
#define I2C1_SCL_PAL_MODE 4
#define I2C1_SDA_PAL_MODE 4

// 指定使用IIC1接口的SCL和SDA引脚的速度，单位为Hz，这里设置为400kHz
#define F_SCL 400000UL

// 指定TrackPoint的中断引脚，这里使用C13引脚
#define TRACKPOINT_INT_PIN  C13
