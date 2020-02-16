#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#include "esp_system.h"

#define GPIO_AD7998_CONVST CONFIG_GPIO_AD7998_CONVST



typedef struct 
{
    bool convst;
} ad7998_config_t;


esp_err_t ad7998_enable(void);