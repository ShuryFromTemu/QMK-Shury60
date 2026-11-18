// 启用 IIC
I2C_DRIVER_REQUIRED = yes

// 启用指点设备
POINTING_DEVICE_ENABLE = yes
POINTING_DEVICE_DRIVER = custom
SRC += trackpoint.c