# PID Demo

这个Arduino的项目写的很随意，东拼西凑出来的，只是基本实现了功能而已，应该不会更新了

之后会做到rt-thread系统上

# 相关描述

相关硬件平台: Arduino UNO、MPU6050、sg90

依赖库: [I2Cdev](https://github.com/jrowberg/i2cdevlib)、[Timer](https://github.com/JChristensen/Timer)、[Fuzzy-like-PI-controller](https://github.com/afakharany93/Fuzzy-like-PI-controller)、Sevor

## 系统架构

![](/doc/img/design_of_sys.jpg)

![](/doc/img/fussy_pid_demo.jpg)

# 存在问题：

+ 在UNO的板上隔一段时间后会死机，暂时不知道原因，也懒得去查了
+ mpu6050的fifo有时会overflow
