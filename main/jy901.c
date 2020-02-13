
//**************************
//维特智能9轴惯性传感器JY901驱动：
//I2C地址为0x50，从0x30开始读取寄存器数据
//******************************

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "jy901.h"

#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

static esp_err_t i2c_master_read_imu(i2c_port_t i2c_num, uint8_t *data_rd, size_t size)
{
    int ret;
    if (size == 0)
    {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    //----------------
    esp_err_t temp = i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ESP_SLAVE_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x30, ACK_CHECK_EN);
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

// esp_err_t write_imu_command(uint8_t *imu_command,size_t len)
// {
//     int ret;   
//     i2c_port_t i2c_num = I2C_MASTER_NUM;
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     //----------------
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, ESP_SLAVE_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
//     i2c_master_write(cmd, imu_command, len,ACK_CHECK_EN);
//     i2c_master_stop(cmd);
//     ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
//     i2c_cmd_link_delete(cmd);    
//     return ret;
// }

esp_err_t read_imu_data(IMU_Data *imu_data_rd)
{    
    uint8_t data_rd[DATA_LENGTH]={};
    esp_err_t ret = i2c_master_read_imu(I2C_MASTER_NUM, data_rd, sizeof(data_rd));

    //0x34  X轴加速度：ax=((AxH<<8)|AxL)/32768*16g(g 为重力加速度，可取 9.8m/s^2)
    //0x35  Y轴加速度：ay=((AyH<<8)|AyL)/32768*16g(g 为重力加速度，可取 9.8m/s^2)
    //0x36  Z轴加速度：az=((AzH<<8)|AzL)/32768*16g(g 为重力加速度，可取 9.8m/s^2)
    memcpy(imu_data_rd->spatial_attitude,data_rd+8,6); 

    //0x37  X轴角速度：wx=((wxH<<8)|wxL)/32768*2000(°/s)
    //0x38  Y轴角速度：wy=((wyH<<8)|wyL)/32768*2000(°/s)
    //0x39  Z轴角速度：wz=((wzH<<8)|wzL)/32768*2000(°/s)
    memcpy(imu_data_rd->spatial_attitude+6,data_rd+14,6);

    //0x51  四元数Q0：Q0=((Q0H<<8)|Q0L)/32768
    //0x52  四元数Q1：Q1=((Q1H<<8)|Q1L)/32768
    //0x53  四元数Q2：Q2=((Q2H<<8)|Q2L)/32768
    //0x54  四元数Q3：Q3=((Q3H<<8)|Q3L)/32768
    memcpy(imu_data_rd->spatial_attitude+12,data_rd+66,8);

    //0x32  分钟：MM；
    //0x32  秒：SS
    //0x33  毫秒：MS=((MSH<<8)|MSL)
    memcpy(imu_data_rd->appendix_infor,data_rd+4,4);

    //0x40  温度：T=((TH<<8)|TL) /100 ℃
    memcpy(imu_data_rd->appendix_infor+4,data_rd+32,2);

    //0x3d  滚转角（x 轴）：Roll=((RollH<<8)|RollL)/32768*180(°)
    //0x3e  俯仰角（y 轴）：Pitch=((PitchH<<8)|PitchL)/32768*180(°)
    //0x3f  偏航角（z 轴）：Yaw=((YawH<<8)|YawL)/32768*180(°)    
    memcpy(imu_data_rd->appendix_infor+6,data_rd+26,6);

    return ret;
}