#ifndef _I2C_BUFFER_H_
#define _I2C_BUFFER_H_

#include "esp_err.h"
#include "sensor_type.h"

#define MAX_BUFFER_SIZE 2000u

struct i2c_buffer_node
{
    sensor_data_t*  sensor_data;
    struct buffer_node* prior;
    struct buffer_node* next;
};

struct i2c_buffer_t
{
    unsigned int size;
    esp_err_t i2c_buffer_err;

    struct i2c_buffer_node *phead;
    struct i2c_buffer_node *ptail;

    void(*push)(struct i2c_buffer_t *p, int num);		// push a numb to FIFO
	int(*pop)(struct i2c_buffer_t *p);				// pop a numb from FIFO
	void(*print)(struct i2c_buffer_t *p);				// print all numb in FIFO
	unsigned int(*get_size)(struct i2c_buffer_t *p);	// get free size of FIFO
	int(*is_err)(struct i2c_buffer_t *p);				// judge FIFO err happened or not
	void(*clear)(struct i2c_buffer_t *p);

};

void i2c_buffer_push(struct i2c_buffer_t *p, sensor_data_t sensor_data);
sensor_data_t*	i2c_buffer_pop(struct i2c_buffer_t *p);
void i2c_buffer_print(struct i2c_buffer_t *p,int32_t id);
int	i2c_buffer_is_err(struct i2c_buffer_t *p);
void i2c_buffer_clear(struct i2c_buffer_t *p);
unsigned int i2c_buffer_get_size(struct i2c_buffer_t *p);

struct i2c_buffer_t *i2c_buffer_create(void);

#endif /* _IOT_BOARD_H_ */