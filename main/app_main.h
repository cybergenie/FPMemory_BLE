/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SUPPORT_IMU

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define DATA_LENGTH 74                   /*!< Data buffer length of test buffer */
#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

// #define CMD_START 0x51

// #define ESP_SLAVE_ADDR CONFIG_I2C_IMU_ADDRESS /*!< ESP32 slave address, you can set any 7bit value */
// #define WRITE_BIT I2C_MASTER_WRITE            /*!< I2C master write */
// #define READ_BIT I2C_MASTER_READ              /*!< I2C master read */
// #define ACK_CHECK_EN 0x1                      /*!< I2C master will check ack from slave*/
// #define ACK_CHECK_DIS 0x0                     /*!< I2C master will not check ack from slave */
// #define ACK_VAL 0x0                           /*!< I2C ack value */
// #define NACK_VAL 0x1

/*
 * DEFINES
 ****************************************************************************************
 */
//#define SUPPORT_HEARTBEAT
//#define SPP_DEBUG_MODE

#define i2c_sprintf(s,...)         sprintf((char*)(s), ##__VA_ARGS__)
#define I2C_DATA_MAX_LEN           (512)
#define I2C_CMD_MAX_LEN            (20)
#define I2C_STATUS_MAX_LEN         (20)
#define I2C_DATA_BUFF_MAX_LEN      (2*1024)

#define IMU_WAKE_UP                 0x22

/* Attributes State Machine */
enum
{
    I2C_IDX_SVC,

    I2C_DATA_RECV_CHAR,
    I2C_DATA_RECV_VAL,

    I2C_DATA_NOTIFY_CHAR,
    I2C_DATA_NTY_VAL,
    I2C_DATA_NTF_CFG,

    I2C_COMMAND_CHAR,
    I2C_COMMAND_VAL,

    I2C_STATUS_CHAR,
    I2C_STATUS_VAL,
    I2C_STATUS_CFG,

#ifdef SUPPORT_IMU
    IMU_DATA_NOTIFY_CHAR,
    IMU_DATA_NTY_VAL,
    IMU_DATA_NTF_CFG,
#endif

    I2C_IDX_NB,
};
