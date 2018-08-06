/*
 * adc_voltage.h
 *
 *  Created on: 2018Äê2ÔÂ20ÈÕ
 *      Author: Lee
 */

#ifndef MAIN_ADC_VOLTAGE_H_
#define MAIN_ADC_VOLTAGE_H_

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_system.h"
#include "esp_adc_cal.h"
#include "esp_err.h"

void adc_voltage(uint32_t voltage[]);


#endif /* MAIN_ADC_VOLTAGE_H_ */
