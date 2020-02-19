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

#define GPIO_AD7998_CONVST CONFIG_GPIO_AD7998_CONVST



typedef struct 
{
    bool convst;
} ad7998_config_t;


esp_err_t ad799x_power_up(void);

#endif