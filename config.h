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
