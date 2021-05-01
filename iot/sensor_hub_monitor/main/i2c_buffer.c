#include <stdio.h>
#include "esp_log.h"
#include "iot_sensor_hub.h"
#include "i2c_buffer.h"

static const char *TAG = "I2C_Buffer";

void i2c_buffer_push(struct i2c_buffer_t *p, sensor_data_t sensor_data)
{
	if(p->size >= MAX_BUFFER_SIZE){
		p->i2c_buffer_err = ESP_FAIL;
        ESP_LOGE(TAG,"push failed! I2C BUFFER is full.");		
		return ;
	}
	struct i2c_buffer_node *p_node = (struct i2c_buffer_node *)malloc(sizeof(struct i2c_buffer_node));

	// initial p_node
	p_node->sensor_data = sensor_data;
	p_node->next = NULL;
	p_node->prior = p->ptail;

	// add p_node to i2c buffer
	if(p->size == 0){
		p->phead = p_node;
	}
	else{
		p->ptail->next = p_node;
	}
	p->ptail = p_node;
	p->size++;
	p->i2c_buffer_err = ESP_OK;	
}

sensor_data_t i2c_buffer_pop(struct i2c_buffer_t *p)
{
	if(p->size <= 0){
        ESP_LOGE(TAG,"pop failed! i2c buffer is empty.");		
		p->i2c_buffer_err = ESP_FAIL;
		return;
	}
	sensor_data_t ret = p->phead->sensor_data;
	if(p->size == 1){			// last member in i2c buffer
		free(p->phead);
		p->phead = NULL;
		p->ptail = NULL;
	}
	else{
		p->phead = p->phead->next;
		free(p->phead->prior);
		p->phead->prior = NULL;
	}
	p->size--;
	p->i2c_buffer_err = ESP_OK;
	return ret;
}

unsigned int i2c_buffer_get_size(struct i2c_buffer_t *p)
{
	p->i2c_buffer_err = ESP_OK;
	return p->size;
}

void i2c_buffer_print(struct i2c_buffer_t *p ,int32_t id)
{
	if(p->size <= 0){
        ESP_LOGI(TAG,"NULL");		
		p->i2c_buffer_err = ESP_FAIL;
		return ;
	}
	struct i2c_buffer_node *ptmp = p->phead;
	unsigned int i = 0;
	for(i = 0; i < p->get_size(p); i++){        
        sensor_data_t sensor_data = ptmp->sensor_data;
        sensor_type_t sensor_type = (sensor_type_t)((sensor_data.sensor_id) >> 4 & SENSOR_ID_MASK);
        if (sensor_type >= SENSOR_TYPE_MAX) {        
            ESP_LOGE(TAG, "sensor_id invalid, id=%d", sensor_data.sensor_id);        
            return;    
        }    
        switch (id) {        
            case SENSOR_STARTED:            
                ESP_LOGI(TAG, "Timestamp = %llu - %s SENSOR_STARTED",                     
                    sensor_data.timestamp,                     
                    SENSOR_TYPE_STRING[sensor_type]);            
                break;        
            case SENSOR_STOPED:            
                ESP_LOGI(TAG, "Timestamp = %llu - %s SENSOR_STOPED",                     
                    sensor_data.timestamp,                     
                    SENSOR_TYPE_STRING[sensor_type]);            
            break;        
            case SENSOR_HUMI_DATA_READY:            
                ESP_LOGI(TAG, "Timestamp = %llu - SENSOR_HUMI_DATA_READY - "                     
                    "humiture=%.2f",                     
                    sensor_data.timestamp,                     
                    sensor_data.humidity);            
            break;        
            case SENSOR_TEMP_DATA_READY:            
                ESP_LOGI(TAG, "Timestamp = %llu - SENSOR_TEMP_DATA_READY - "                     
                    "temperature=%.2f\n",                     
                    sensor_data.timestamp,                     
                    sensor_data.temperature);            
            break;        
            case SENSOR_ACCE_DATA_READY:            
                ESP_LOGI(TAG, "Timestamp = %llu - SENSOR_ACCE_DATA_READY - "
                "acce_x=%.2f, acce_y=%.2f, acce_z=%.2f\n",
                sensor_data.timestamp,
                sensor_data.acce.x, sensor_data.acce.y, sensor_data.acce.z);            
            break;        
            case SENSOR_GYRO_DATA_READY:            
                ESP_LOGI(TAG, "Timestamp = %llu - SENSOR_GYRO_DATA_READY - "
                    "gyro_x=%.2f, gyro_y=%.2f, gyro_z=%.2f\n",
                    sensor_data.timestamp,
                    sensor_data.gyro.x, sensor_data.gyro.y, sensor_data.gyro.z);            
            break;        
            case SENSOR_LIGHT_DATA_READY:            
                ESP_LOGI(TAG, "Timestamp = %llu - SENSOR_LIGHT_DATA_READY - "
                    "light=%.2f",
                    sensor_data.timestamp,
                    sensor_data.light);            
            break;        
            case SENSOR_RGBW_DATA_READY:            
                ESP_LOGI(TAG, "Timestamp = %llu - SENSOR_RGBW_DATA_READY - "
                    "r=%.2f, g=%.2f, b=%.2f, w=%.2f\n",
                    sensor_data.timestamp,
                    sensor_data.rgbw.r, sensor_data.rgbw.r, sensor_data.rgbw.b, sensor_data.rgbw.w);            
            break;        
            case SENSOR_UV_DATA_READY:            
                ESP_LOGI(TAG, "Timestamp = %llu - SENSOR_UV_DATA_READY - "
                    "uv=%.2f, uva=%.2f, uvb=%.2f\n",
                    sensor_data.timestamp,
                    sensor_data.uv.uv, sensor_data.uv.uva, sensor_data.uv.uvb);            
            break;        
            default:            
                ESP_LOGI(TAG, "Timestamp = %llu - event id = %d", sensor_data.timestamp, id);            
            break;    
        }
		ptmp = ptmp->next;
	}
	p->i2c_buffer_err = ESP_OK;
}

esp_err_t i2c_buffer_is_err(struct i2c_buffer_t *p)
{
	return p->i2c_buffer_err;
}

void i2c_buffer_clear(struct i2c_buffer_t *p)
{
	if(p->size <= 0){
		p->i2c_buffer_err = ESP_OK;
		return ;
	}
	while(--(p->size)){				// clear p->depth-1 member
		p->ptail = p->ptail->prior;
		free(p->ptail->next);
		p->ptail->next = NULL;
	}

	// clear last member
	free(p->ptail);
	free(p->phead);
	p->ptail = NULL;
	p->phead = NULL;
	p->i2c_buffer_err = ESP_OK;
}

struct i2c_buffer_t *i2c_buffer_create(void)
{
	struct i2c_buffer_t *p = (struct i2c_buffer_t *)malloc(sizeof(struct i2c_buffer_t));

	// initial i2c buffer member
	p->size 	= 0;
	p->i2c_buffer_err = ESP_OK;
	p->phead 	= NULL;
	p->ptail 	= NULL;
	p->is_err 	= i2c_buffer_is_err;
	p->pop 		= i2c_buffer_pop;
	p->print 	= i2c_buffer_print;
	p->push 	= i2c_buffer_push;
	p->clear 	= i2c_buffer_clear;
	p->get_size = i2c_buffer_get_size;

	return p;
}









