#include "adc_voltage.h"

/*Note: Different ESP32 modules may have different reference voltages varying from
 * 1000mV to 1200mV. Use #define GET_VREF to route v_ref to a GPIO
 */
#define V_REF   1100

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;
static const adc_channel_t channel[8] = {
		ADC1_CHANNEL_4,		//IO 32
		ADC1_CHANNEL_5,		//IO 33
		ADC1_CHANNEL_6,		//IO 34
		ADC1_CHANNEL_7,		//IO 36
		ADC2_CHANNEL_5,		//IO 12
		ADC2_CHANNEL_6,		//IO 14
		ADC2_CHANNEL_7,		//IO 27
		ADC2_CHANNEL_9		//IO 26
};

static void check_efuse()
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}


void adc_voltage(uint32_t voltage[])
{

	//Check if Two Point or Vref are burned into eFuse
	check_efuse();

	//Characterize ADC
	adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
	esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, V_REF, adc_chars);


	for(int i = 0; i < 8; i++)
	{
		uint32_t adc_reading = 0;
		if(i<4)
		{
			adc1_config_width(ADC_WIDTH_BIT_12);
			adc1_config_channel_atten(channel[i], atten);
			adc_reading=adc1_get_raw((adc1_channel_t)channel[i]);
		}
		else
		{
			adc2_config_channel_atten((adc2_channel_t)channel[i], atten);
			int raw = 0;
			adc_reading =adc2_get_raw((adc2_channel_t)channel[i], ADC_WIDTH_BIT_12, &raw);
			adc_reading = raw;
		}
		voltage[i] =esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
	}

}
