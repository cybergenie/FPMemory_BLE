#include "ad799x.h"

static void ad799x_convst_gpio_init(void)
{
    //定义一个gpio_config类型的结构体，下面的都算对其进行的配置
    gpio_config_t io_conf;
    //禁用中断
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //设置为输出模式
    io_conf.mode = GPIO_MODE_OUTPUT;
    //你想设置的引脚
    io_conf.pin_bit_mask = (1 << GPIO_AD7998_CONVST);
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
    ad7998_convst_gpio_init();
    ret = gpio_set_level(GPIO_AD7998_CONVST, 1);    
    if (ret == ESP_OK)
    {
        vTaskDelay(5 / portTICK_PERIOD_MS);
        ret = gpio_set_level(GPIO_AD7998_CONVST, 0);
    }    
    return ret;
}
