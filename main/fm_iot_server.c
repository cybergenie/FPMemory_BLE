/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "string.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"

#include "board.h"
#include "iot_sensor_hub.h"

#include "fm_iot_server.h"

#define GATTS_TABLE_TAG  "FM_SENSOR_SERVER"

#define SENSOR_PERIOD CONFIG_SENSOR_EXAMPLE_PERIOD
#define SENSOR_BUFFER_SIZE CONFIG_SENSOR_BUFFER_SIZE


#define TAG "ble Monitor"

static void sensor_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{        
    sensor_data_t *sensor_data = (sensor_data_t *)event_data;
    sensor_type_t sensor_type = (sensor_type_t)((sensor_data->sensor_id) >> 4 & SENSOR_ID_MASK);

    if (sensor_type >= SENSOR_TYPE_MAX) {
        ESP_LOGE(TAG, "sensor_id invalid, id=%d", sensor_data->sensor_id);
        return;
    }    
        if( xQueueSend( sensor_data_queue, sensor_data, 0 ) != pdPASS )
        {
            ESP_LOGE(TAG, "the sensor data buffer is full!");
        }
    switch (id) {
        case SENSOR_STARTED:
        {                    
            ESP_LOGI(TAG, "Timestamp = %llu - %s SENSOR_STARTED",
                     sensor_data->timestamp,
                     SENSOR_TYPE_STRING[sensor_type]);
            break;
        }
        case SENSOR_STOPED:
            ESP_LOGI(TAG, "Timestamp = %llu - %s SENSOR_STOPED",
                     sensor_data->timestamp,
                     SENSOR_TYPE_STRING[sensor_type]);
            break;
        case SENSOR_HUMI_DATA_READY:
            ESP_LOGI(TAG, "Timestamp = %llu - SENSOR_HUMI_DATA_READY - "
                     "humiture=%.2f",
                     sensor_data->timestamp,
                     sensor_data->humidity);
            break;
        case SENSOR_TEMP_DATA_READY:
            ESP_LOGI(TAG, "Timestamp = %llu - SENSOR_TEMP_DATA_READY - "
                     "temperature=%.2f\n",
                     sensor_data->timestamp,
                     sensor_data->temperature);
            break;
        case SENSOR_ACCE_DATA_READY:
            ESP_LOGI(TAG, "Timestamp = %llu - SENSOR_ACCE_DATA_READY - "
                     "acce_x=%.2f, acce_y=%.2f, acce_z=%.2f\n",
                     sensor_data->timestamp,
                     sensor_data->acce.x, sensor_data->acce.y, sensor_data->acce.z);
            break;
        case SENSOR_GYRO_DATA_READY:
            ESP_LOGI(TAG, "Timestamp = %llu - SENSOR_GYRO_DATA_READY - "
                     "gyro_x=%.2f, gyro_y=%.2f, gyro_z=%.2f\n",
                     sensor_data->timestamp,
                     sensor_data->gyro.x, sensor_data->gyro.y, sensor_data->gyro.z);
            break;
        case SENSOR_LIGHT_DATA_READY:
            ESP_LOGI(TAG, "Timestamp = %llu - SENSOR_LIGHT_DATA_READY - "
                     "light=%.2f",
                     sensor_data->timestamp,
                     sensor_data->light);
            break;
        case SENSOR_RGBW_DATA_READY:
            ESP_LOGI(TAG, "Timestamp = %llu - SENSOR_RGBW_DATA_READY - "
                     "r=%.2f, g=%.2f, b=%.2f, w=%.2f\n",
                     sensor_data->timestamp,
                     sensor_data->rgbw.r, sensor_data->rgbw.r, sensor_data->rgbw.b, sensor_data->rgbw.w);
            break;
        case SENSOR_UV_DATA_READY:
            ESP_LOGI(TAG, "Timestamp = %llu - SENSOR_UV_DATA_READY - "
                     "uv=%.2f, uva=%.2f, uvb=%.2f\n",
                     sensor_data->timestamp,
                     sensor_data->uv.uv, sensor_data->uv.uva, sensor_data->uv.uvb);
            break;
        default:
            ESP_LOGI(TAG, "Timestamp = %llu - event id = %d", sensor_data->timestamp, id);
            break;
    }
}



void sensor_acq_task(void)
{
    /*initialize board for peripheral auto-init with default parameters,
    you can use menuconfig to choose a target board*/
    esp_err_t err = iot_board_init();
    if (err != ESP_OK) {
        goto error_loop;
    }

    /*get the i2c0 bus handle with a resource ID*/
    bus_handle_t i2c0_bus_handle = (bus_handle_t)iot_board_get_handle(BOARD_I2C0_ID);
    if (i2c0_bus_handle == NULL) {
        goto error_loop;
    }
    
    /*register handler with NULL specific typeID, thus all events posted to sensor_loop will be handled*/
    ESP_ERROR_CHECK(iot_sensor_handler_register_with_type(NULL_ID, NULL_ID, sensor_event_handler, NULL));

    /*create sensors based on sensor scan result*/
    sensor_info_t* sensor_infos[10];
    sensor_handle_t sensor_handle[10] = {NULL};
    sensor_config_t sensor_config = {
        .bus = i2c0_bus_handle, /*which bus sensors will connect to*/
        .mode = MODE_POLLING, /*data acquire mode*/
        .min_delay = SENSOR_PERIOD /*data acquire period*/
    };
    int num = iot_sensor_scan(i2c0_bus_handle, sensor_infos, 10); /*scan for valid sensors based on active i2c address*/
    ESP_LOGI(TAG,"number of sensors:%d",num);
    for (size_t i = 0; i < num && i<10; i++) {

        if (ESP_OK != iot_sensor_create(sensor_infos[i]->sensor_id, &sensor_config, &sensor_handle[i])) { /*create a sensor with specific sensor_id and configurations*/
            goto error_loop;
        }

        iot_sensor_start(sensor_handle[i]); /*start a sensor, data ready events will be posted once data acquired successfully*/
        ESP_LOGI(TAG,"%s (%s) created",sensor_infos[i]->name, sensor_infos[i]->desc);
    }

    while (1) {
        vTaskDelay(1000);
    }

error_loop:
    ESP_LOGE(TAG, "ERRO");
    while (1) {
        vTaskDelay(1000);
    }
}



void i2c_task(void *pvParameters)
{ 
    sensor_data_t  lReceivedValue;
    uint8_t * temp = NULL;
    portBASE_TYPE xStatus;
    for (;;) 
    {
         temp = (uint8_t *)malloc(sizeof(lReceivedValue));
         if(temp == NULL){
            ESP_LOGE(GATTS_TABLE_TAG, "%s malloc.1 failed\n", __func__);
            break;
        }
        xStatus= xQueueReceive(sensor_data_queue, temp, (portTickType)portMAX_DELAY);
         if (xStatus==pdPASS)
         {
             esp_ble_gatts_send_indicate(sensor_gatts_if, sensor_conn_id, sensor_handle_table[SENSOR_IDX_SENSOR_DATA_NTY_VAL],sizeof(lReceivedValue), temp, false);
             ESP_LOGE(TAG, "the sensor data is %x",*temp);             
         }
         else
         {
             ESP_LOGE(TAG, "the sensor data is empty");
         }

    }
    vTaskDelete(NULL);

}

static void sensor_task_init(void)
{
    sensor_data_queue = xQueueCreate(SENSOR_BUFFER_SIZE, sizeof(sensor_data_t));
    xTaskCreatePinnedToCore(i2c_task, "i2c_task", 2048, "task2",2, NULL,1);
    sensor_acq_task();
      
    //xTaskCreate(ble_demo_task, "ble_demo_task", 2048, "task1",4, NULL);
    
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;
    ESP_LOGE(GATTS_TABLE_TAG, "GAP_EVT, event %d\n", event);

    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&sensor_adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TABLE_TAG, "Advertising start failed: %s\n", esp_err_to_name(err));
        }
        break;
    default:
        break;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *) param;
    

    ESP_LOGI(GATTS_TABLE_TAG, "event = %x\n",event);
    switch (event) {
    	case ESP_GATTS_REG_EVT:
    	    ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
        	esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);

        	ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
        	esp_ble_gap_config_adv_data_raw((uint8_t *)sensor_adv_data, sizeof(sensor_adv_data));

        	ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
        	esp_ble_gatts_create_attr_tab(sensor_gatt_db, gatts_if, SENSOR_IDX_NB, SENSOR_SVC_INST_ID);
       	break;
    	case ESP_GATTS_READ_EVT:            
       	    break;
    	case ESP_GATTS_WRITE_EVT:
      	 	break;    	
    	case ESP_GATTS_EXEC_WRITE_EVT:    	    
    	    break;    	
    	case ESP_GATTS_MTU_EVT:
    	    sensor_mtu_size = p_data->mtu.mtu;
    	    break;
    	case ESP_GATTS_CONF_EVT:
    	    break;
    	case ESP_GATTS_UNREG_EVT:
        	break;
    	case ESP_GATTS_DELETE_EVT:
        	break;
    	case ESP_GATTS_START_EVT:
        	break;
    	case ESP_GATTS_STOP_EVT:
        	break;
    	case ESP_GATTS_CONNECT_EVT:
    	    sensor_conn_id = p_data->connect.conn_id;
    	    sensor_gatts_if = gatts_if;
    	    is_connected = true;
    	    memcpy(&sensor_remote_bda,&p_data->connect.remote_bda,sizeof(esp_bd_addr_t));
        	break;
    	case ESP_GATTS_DISCONNECT_EVT:
    	    is_connected = false;
    	    enable_data_ntf = false;
    	    esp_ble_gap_start_advertising(&sensor_adv_params);
    	    break;
    	case ESP_GATTS_OPEN_EVT:
    	    break;
    	case ESP_GATTS_CANCEL_OPEN_EVT:
    	    break;
    	case ESP_GATTS_CLOSE_EVT:
    	    break;
    	case ESP_GATTS_LISTEN_EVT:
    	    break;
    	case ESP_GATTS_CONGEST_EVT:
    	    break;
    	case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
    	    ESP_LOGI(GATTS_TABLE_TAG, "The number handle =%x\n",param->add_attr_tab.num_handle);
    	    if (param->add_attr_tab.status != ESP_GATT_OK){
    	        ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
    	    }
    	    else if (param->add_attr_tab.num_handle != SENSOR_IDX_NB){
    	        ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table abnormally, num_handle (%d) doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, SENSOR_IDX_NB);
    	    }
    	    else {
    	        memcpy(sensor_handle_table, param->add_attr_tab.handles, sizeof(sensor_handle_table));
    	        esp_ble_gatts_start_service(sensor_handle_table[SENSOR_IDX_SVC]);
    	    }
    	    break;
    	}
    	default:
    	    break;
    }
}


static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_TABLE_TAG, "EVT %d, gatts if %d\n", event, gatts_if);

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            sensor_profile_tab[SENSOR_PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGI(GATTS_TABLE_TAG, "Reg app failed, app_id %04x, status %d\n",param->reg.app_id, param->reg.status);
            return;
        }
    }

    do {
        int idx;
        for (idx = 0; idx < SENSOR_PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == sensor_profile_tab[idx].gatts_if) {
                if (sensor_profile_tab[idx].gatts_cb) {
                    sensor_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void app_main(void)
{
    esp_err_t ret;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(GATTS_TABLE_TAG, "%s init bluetooth\n", __func__);
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(ESP_SENSOR_APP_ID);
    
    sensor_task_init();

    return;
}
