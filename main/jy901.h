#ifndef __JY901_H__
#define __JY901_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/i2c.h"

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define CMD_START 0x51

#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define ESP_SLAVE_ADDR CONFIG_I2C_IMU_ADDRESS                 /*!< ESP32 slave address, you can set any 7bit value */
#define WRITE_BIT I2C_MASTER_WRITE                            /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                              /*!< I2C master read */
#define ACK_CHECK_EN 0x1                                      /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                                     /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                                           /*!< I2C ack value */
#define NACK_VAL 0x1

#define DATA_LENGTH 74 /*!< Data buffer length of test buffer */

typedef struct
{
    //      MM;         0x32  分钟：MM；
    //      SS;         0x32  秒：SS
    //      MS;         0x33  毫秒：MS=((MSH<<8)|MSL)
    //      AX;         0x34  X轴加速度：ax=((AxH<<8)|AxL)/32768*16g(g 为重力加速度，可取 9.8m/s^2)
    //      AY;         0x35  Y轴加速度：ay=((AyH<<8)|AyL)/32768*16g(g 为重力加速度，可取 9.8m/s^2)
    //      AZ;         0x36  Z轴加速度：az=((AzH<<8)|AzL)/32768*16g(g 为重力加速度，可取 9.8m/s^2)
    //      GX;         0x37  X轴角速度：wx=((wxH<<8)|wxL)/32768*2000(°/s)
    //      GY;         0x38  Y轴角速度：wy=((wyH<<8)|wyL)/32768*2000(°/s)
    //      GZ;         0x39  Z轴角速度：wz=((wzH<<8)|wzL)/32768*2000(°/s)
    //      Roll;       0x3d  滚转角（x 轴）：Roll=((RollH<<8)|RollL)/32768*180(°)
    //      Pitch;      0x3e  俯仰角（y 轴）：Pitch=((PitchH<<8)|PitchL)/32768*180(°)
    //      Yaw;        0x3f  偏航角（z 轴）：Yaw=((YawH<<8)|YawL)/32768*180(°)
    //      TEMP;       0x40  温度：T=((TH<<8)|TL) /100 ℃
    //      Q0;         0x51  四元数Q0：Q0=((Q0H<<8)|Q0L)/32768
    //      Q1;         0x52  四元数Q1：Q1=((Q1H<<8)|Q1L)/32768
    //      Q2;         0x53  四元数Q2：Q2=((Q2H<<8)|Q2L)/32768
    //      Q3;         0x54  四元数Q3：Q3=((Q3H<<8)|Q3L)/32768

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //|AxH|AxL|AyH|AyL|AzH|AzL|GxH|GxL|GyH|GyL|GzH|GzL|Q0H|Q0L|Q1H|Q1L|Q2H|Q2L|Q3H|Q3L|
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    uint8_t spatial_attitude[20];

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //|MM|SS|MSH|MSL|TH|TL|RollH|RollL|PitchH|PitchL|YawH|YawL|
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    uint8_t appendix_infor[12];
} IMU_Data;

esp_err_t read_imu_data(IMU_Data *imu_data_rd);

// esp_err_t write_imu_command(uint8_t *imu_command, size_t len);

#endif