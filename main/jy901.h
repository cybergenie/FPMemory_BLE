#ifndef __JY901_H__
#define __JY901_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/i2c.h"

#define CMD_START 0x51

#define ESP_SLAVE_ADDR CONFIG_I2C_IMU_ADDRESS /*!< ESP32 slave address, you can set any 7bit value */
#define WRITE_BIT I2C_MASTER_WRITE            /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ              /*!< I2C master read */
#define ACK_CHECK_EN 0x1                      /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                     /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                           /*!< I2C ack value */
#define NACK_VAL 0x1

esp_err_t i2c_master_read_imu(i2c_port_t i2c_num, uint8_t *data_rd, size_t size);

#endif