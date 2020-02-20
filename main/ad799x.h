#ifndef __AD799X_H__
#define __AD799X_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#include "esp_system.h"
#include "driver/i2c.h"

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 16

#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define GPIO_AD799X_CONVST CONFIG_GPIO_AD799X_CONVST
#define ESP_SLAVE_ADDR CONFIG_I2C_ADC_ADDRESS                 /*!< ESP32 slave address, you can set any 7bit value */
#define WRITE_BIT I2C_MASTER_WRITE                            /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                              /*!< I2C master read */
#define ACK_CHECK_EN 0x1                                      /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                                     /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                                           /*!< I2C ack value */
#define NACK_VAL 0x1


typedef struct
{
    uint8_t data_sensor_1[2];
    uint8_t data_sensor_2[2];
    uint8_t data_sensor_3[2];
    uint8_t data_sensor_4[2];
    uint8_t data_sensor_5[2];
    uint8_t data_sensor_6[2];
    uint8_t data_sensor_7[2];
    uint8_t data_sensor_8[2];

} ADC_Data;


typedef struct 
{
    // bool convst;
    uint8_t addr_result_geg[2];
    uint8_t result_config[2];
    uint8_t addr_config_reg[2];
    
} ad7998_config_t;

ad7998_config_t ad7998_config = {
    // .convst = false,
    .addr_result_geg = {0x00,0x00},     //
    .result_config = {0x0F,0xF8},
    .addr_config_reg = {0x00,0x02},
};


esp_err_t ad799x_power_up(void);

#endif