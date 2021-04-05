/* cmd_i2ctools.c

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "argtable3/argtable3.h"
#include "driver/i2c.h"
#include "esp_console.h"
#include "esp_log.h"

#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define WRITE_BIT I2C_MASTER_WRITE  /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ    /*!< I2C master read */
#define ACK_CHECK_EN 0x1            /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0           /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                 /*!< I2C ack value */
#define NACK_VAL 0x1                /*!< I2C nack value */
#define I2C_SLAVE_ADDRESS CONFIG_I2C_SLAVE_ADDRESS  
#define I2C_MASTER_SCL CONFIG_I2C_MASTER_SCL
#define I2C_MASTER_SDA CONFIG_I2C_MASTER_SDA

static const char *TAG = "cmd_i2ctools";

static gpio_num_t i2c_gpio_sda = 18;
static gpio_num_t i2c_gpio_scl = 19;
static uint32_t i2c_frequency = 100000;
static i2c_port_t i2c_port = I2C_NUM_0;


static esp_err_t i2c_get_port(int port, i2c_port_t *i2c_port)
{
    if (port >= I2C_NUM_MAX) {
        ESP_LOGE(TAG, "Wrong port number: %d", port);
        return ESP_FAIL;
    }
    switch (port) {
    case 0:
        *i2c_port = I2C_NUM_0;
        break;
    case 1:
        *i2c_port = I2C_NUM_1;
        break;
    default:
        *i2c_port = I2C_NUM_0;
        break;
    }
    return ESP_OK;
}

static esp_err_t i2c_master_driver_initialize(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = i2c_gpio_sda,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = i2c_gpio_scl,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = i2c_frequency
    };
    return i2c_param_config(i2c_port, &conf);
}

static struct {
    struct arg_int *chip_address;
    struct arg_int *size;
    struct arg_end *end;
} i2cdump_args;

static int do_i2cdump_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&i2cdump_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, i2cdump_args.end, argv[0]);
        return 0;
    }

    /* Check chip address: "-c" option */
    int chip_addr = i2cdump_args.chip_address->ival[0];
    /* Check read size: "-s" option */
    int size = 1;
    if (i2cdump_args.size->count) {
        size = i2cdump_args.size->ival[0];
    }
    if (size != 1 && size != 2 && size != 4) {
        ESP_LOGE(TAG, "Wrong read size. Only support 1,2,4");
        return 1;
    }
    i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    i2c_master_driver_initialize();
    uint8_t data_addr;
    uint8_t data[4];
    int32_t block[16];
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f"
           "    0123456789abcdef\r\n");
    for (int i = 0; i < 128; i += 16) {
        printf("%02x: ", i);
        for (int j = 0; j < 16; j += size) {
            fflush(stdout);
            data_addr = i + j;
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, chip_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
            i2c_master_write_byte(cmd, data_addr, ACK_CHECK_EN);
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, chip_addr << 1 | READ_BIT, ACK_CHECK_EN);
            if (size > 1) {
                i2c_master_read(cmd, data, size - 1, ACK_VAL);
            }
            i2c_master_read_byte(cmd, data + size - 1, NACK_VAL);
            i2c_master_stop(cmd);
            esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 50 / portTICK_RATE_MS);
            i2c_cmd_link_delete(cmd);
            if (ret == ESP_OK) {
                for (int k = 0; k < size; k++) {
                    printf("%02x ", data[k]);
                    block[j + k] = data[k];
                }
            } else {
                for (int k = 0; k < size; k++) {
                    printf("XX ");
                    block[j + k] = -1;
                }
            }
        }
        printf("   ");
        for (int k = 0; k < 16; k++) {
            if (block[k] < 0) {
                printf("X");
            }
            if ((block[k] & 0xff) == 0x00 || (block[k] & 0xff) == 0xff) {
                printf(".");
            } else if ((block[k] & 0xff) < 32 || (block[k] & 0xff) >= 127) {
                printf("?");
            } else {
                printf("%c", block[k] & 0xff);
            }
        }
        printf("\r\n");
    }
    i2c_driver_delete(i2c_port);
    return 0;
}

static void register_i2cdump(void)
{
    i2cdump_args.chip_address = arg_int1("c", "chip", "<chip_addr>", "Specify the address of the chip on that bus");
    i2cdump_args.size = arg_int0("s", "size", "<size>", "Specify the size of each read");
    i2cdump_args.end = arg_end(1);
    const esp_console_cmd_t i2cdump_cmd = {
        .command = "i2cdump",
        .help = "Examine registers visible through the I2C bus",
        .hint = NULL,
        .func = &do_i2cdump_cmd,
        .argtable = &i2cdump_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&i2cdump_cmd));
}

void register_i2ctools(void)
{
    register_i2cconfig();
    register_i2cdectect();
    register_i2cget();
    register_i2cset();
    register_i2cdump();
}
