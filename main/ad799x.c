#include "ad799x.h"

ad799x_config_t ad799x_config = {
    // .convst = false,
    .addr_result_geg = {0x00,0x00},     //
    .result_config = {0x0F,0xF8},
    .addr_config_reg = {0x00,0x02},
};

static void ad799x_convst_gpio_init(void)
{
    //定义一个gpio_config类型的结构体，下面的都算对其进行的配置
    gpio_config_t io_conf;
    //禁用中断
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //设置为输出模式
    io_conf.mode = GPIO_MODE_OUTPUT;
    //你想设置的引脚
    io_conf.pin_bit_mask = (1 << GPIO_AD799X_CONVST);
    //关闭下拉模式
    io_conf.pull_down_en = 0;
    //禁用牵引模式
    io_conf.pull_up_en = 0;
    //配置GPIO与给定的设置。
    gpio_config(&io_conf);
}

//GPIO4高电平持续5ms，启动ADC
esp_err_t ad799x_power_up(void)
{
    esp_err_t ret;
    ad799x_convst_gpio_init();
    ret = gpio_set_level(GPIO_AD799X_CONVST, 1);
    if (ret == ESP_OK)
    {
        vTaskDelay(5 / portTICK_PERIOD_MS);
        ret = gpio_set_level(GPIO_AD799X_CONVST, 0);
    }
    return ret;
}

static esp_err_t i2c_master_read_adc(i2c_port_t i2c_num, uint8_t *data_rd, size_t size)
{
    int ret;
    if (size == 0)
    {
        return ESP_OK;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    //----------------
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ESP_SLAVE_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, ad799x_config.addr_config_reg, 2, ACK_CHECK_EN);
    i2c_master_write(cmd, ad799x_config.result_config, 2, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write(cmd, ad799x_config.addr_result_geg, 2, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1)
    {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);

    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

esp_err_t read_adc_data(ADC_Data *adc_data_rd)
{
    esp_err_t ret;
    uint8_t data_rd[DATA_LENGTH]={};

    ret = ad799x_power_up();

    if(ret == ESP_OK)
    {
        ret = i2c_master_read_adc(I2C_MASTER_NUM,data_rd,sizeof(data_rd));

        memcpy(adc_data_rd->data_sensor_1,data_rd,2);
        memcpy(adc_data_rd->data_sensor_2,data_rd+2,2);
        memcpy(adc_data_rd->data_sensor_3,data_rd+4,2);
        memcpy(adc_data_rd->data_sensor_4,data_rd+6,2);
        memcpy(adc_data_rd->data_sensor_5,data_rd+8,2);
        memcpy(adc_data_rd->data_sensor_6,data_rd+10,2);
        memcpy(adc_data_rd->data_sensor_7,data_rd+12,2);
        memcpy(adc_data_rd->data_sensor_8,data_rd+14,2);

    }
    return ret;

}
