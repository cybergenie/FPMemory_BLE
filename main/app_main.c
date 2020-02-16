//
//   This example code is in the Public Domain (or CC0 licensed, at your option.)
//
//   Unless required by applicable law or agreed to in writing, this
//   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
//  CONDITIONS OF ANY KIND, either express or implied.
//

//****************************************************************************
//
// This demo showcases creating a GATT database using a predefined attribute table.
// It acts as a GATT server and can send adv data, be connected by client.
// Run the gatt_client demo, the client demo will automatically connect to the gatt_server_service_table demo.
// Client demo will enable GATT server's notify after connection. The two devices will then exchange
// data.
//
//****************************************************************************

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/i2c.h"
#include "jy901.h"
#include "ad7998.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "app_main.h"
#include "esp_gatt_common_api.h"

#define GATTS_TABLE_TAG "GATTS_TABLE"
#define I2C_TAG "I2C"

#define PROFILE_NUM 1
#define PROFILE_APP_IDX 0
#define ESP_APP_ID 0x55
#define SAMPLE_DEVICE_NAME "ESP_GATTS"
#define SVC_INST_ID 0

/* The max length of characteristic value. When the GATT client performs a write or prepare write operation,
*  the data length must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX. 
*/
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 500
#define PREPARE_BUF_MAX_SIZE 1024
#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))
// #define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */

#define ADV_CONFIG_FLAG (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

SemaphoreHandle_t print_mux = NULL;
static uint8_t adv_config_done = 0;
static uint16_t i2c_mtu_size = 23;
static uint16_t i2c_conn_id = 0xffff;
static esp_gatt_if_t i2c_gatts_if = 0xff;
static bool enable_data_ntf = false;
static bool is_connected = false;

uint16_t i2c_handle_table[I2C_IDX_NB];

typedef struct spp_receive_data_node
{
    int32_t len;
    uint8_t *node_buff;
    struct spp_receive_data_node *next_node;
} spp_receive_data_node_t;

static spp_receive_data_node_t *temp_spp_recv_data_node_p1 = NULL;
static spp_receive_data_node_t *temp_spp_recv_data_node_p2 = NULL;

typedef struct spp_receive_data_buff
{
    int32_t node_num;
    int32_t buff_size;
    spp_receive_data_node_t *first_node;
} spp_receive_data_buff_t;

static spp_receive_data_buff_t SppRecvDataBuff = {
    .node_num = 0,
    .buff_size = 0,
    .first_node = NULL};

#define CONFIG_SET_RAW_ADV_DATA
#ifdef CONFIG_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
    /* flags */
    0x02, 0x01, 0x06,
    /* tx power*/
    0x02, 0x0a, 0xeb,
    /* service uuid */
    0x03, 0x03, 0xFF, 0x00,
    /* device name */
    0x0f, 0x09, 'E', 'S', 'P', '_', 'G', 'A', 'T', 'T', 'S', '_', 'D', 'E', 'M', 'O'};
static uint8_t raw_scan_rsp_data[] = {
    /* flags */
    0x02, 0x01, 0x06,
    /* tx power */
    0x02, 0x0a, 0xeb,
    /* service uuid */
    0x03, 0x03, 0xFF, 0x00};

#else
static uint8_t service_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb,
    0x34,
    0x9b,
    0x5f,
    0x80,
    0x00,
    0x00,
    0x80,
    0x00,
    0x10,
    0x00,
    0x00,
    0xFF,
    0x00,
    0x00,
    0x00,
};

/* The length of adv data must be less than 31 bytes */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0,       //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //test_manufacturer,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,       //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
#endif /* CONFIG_SET_RAW_ADV_DATA */

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst
{
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst heart_rate_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

static xQueueHandle cmd_cmd_queue = NULL;

#ifdef SUPPORT_IMU
//static xQueueHandle cmd_heartbeat_queue = NULL;
//static uint8_t heartbeat_s[9] = {'E', 's', 'p', 'r', 'e', 's', 's', 'i', 'f'};
static bool enable_imu_ntf = false;
//static uint8_t heartbeat_count_num = 0;
#endif

/// SPP Service
static const uint16_t i2c_service_uuid = 0xABF0;
/// Characteristic UUID
#define ESP_GATT_UUID_I2C_DATA_RECEIVE 0xABF1
#define ESP_GATT_UUID_I2C_DATA_NOTIFY 0xABF2
#define ESP_GATT_UUID_I2C_COMMAND_RECEIVE 0xABF3
#define ESP_GATT_UUID_I2C_COMMAND_NOTIFY 0xABF4

#ifdef SUPPORT_IMU
#define ESP_GATT_UUID_SPP_HEARTBEAT 0xABF5
#endif

//static const uint8_t heart_measurement_ccc[2] = {0x00, 0x00};
//static const uint8_t char_value[4] = {0x11, 0x22, 0x33, 0x44};

/*
 *  SPP PROFILE ATTRIBUTES
 ****************************************************************************************
 */

#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_READ;

#ifdef SUPPORT_IMU
static const uint8_t char_prop_read_write_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
#endif

///I2C Service - data receive characteristic, read&write without response
static const uint16_t i2c_data_receive_uuid = ESP_GATT_UUID_I2C_DATA_RECEIVE;
static const uint8_t i2c_data_receive_val[20] = {0x00};

///I2C Service - data notify characteristic, notify&read
static const uint16_t i2c_data_notify_uuid = ESP_GATT_UUID_I2C_DATA_NOTIFY;
static const uint8_t i2c_data_notify_val[20] = {0x00};
static const uint8_t i2c_data_notify_ccc[2] = {0x00, 0x00};

///I2C Service - command characteristic, read&write without response
static const uint16_t i2c_command_uuid = ESP_GATT_UUID_I2C_COMMAND_RECEIVE;
static const uint8_t i2c_command_val[10] = {0x00};

///I2C Service - status characteristic, notify&read
static const uint16_t i2c_status_uuid = ESP_GATT_UUID_I2C_COMMAND_NOTIFY;
static const uint8_t i2c_status_val[10] = {0x00};
static const uint8_t i2c_status_ccc[2] = {0x00, 0x00};

#ifdef SUPPORT_IMU
///SPP Server - Heart beat characteristic, notify&write&read
static const uint16_t i2c_imu_uuid = ESP_GATT_UUID_SPP_HEARTBEAT;
static const uint8_t i2c_imu_val[2] = {0x00, 0x00};
static const uint8_t i2c_imu_ccc[2] = {0x00, 0x00};
#endif

/* Full Database Description - Used to add attributes into the database */
static const esp_gatts_attr_db_t gatt_db[I2C_IDX_NB] =
    {
        // Service Declaration
        [I2C_IDX_SVC] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ, sizeof(i2c_service_uuid), sizeof(i2c_service_uuid), (uint8_t *)&i2c_service_uuid}},
        //I2C -  data receive characteristic Declaration
        [I2C_DATA_RECV_CHAR] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

        //SPP -  data receive characteristic Value
        [I2C_DATA_RECV_VAL] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&i2c_data_receive_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, I2C_DATA_MAX_LEN, sizeof(i2c_data_receive_val), (uint8_t *)i2c_data_receive_val}},

        //I2C -  data notify characteristic Declaration
        [I2C_DATA_NOTIFY_CHAR] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

        //I2C -  data notify characteristic Value
        [I2C_DATA_NTY_VAL] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&i2c_data_notify_uuid, ESP_GATT_PERM_READ, I2C_DATA_MAX_LEN, sizeof(i2c_data_notify_val), (uint8_t *)i2c_data_notify_val}},

        //I2C -  data notify characteristic - Client Characteristic Configuration Descriptor
        [I2C_DATA_NTF_CFG] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(i2c_data_notify_ccc), (uint8_t *)i2c_data_notify_ccc}},

        //I2C -  command characteristic Declaration
        [I2C_COMMAND_CHAR] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

        //I2C -  command characteristic Value
        [I2C_COMMAND_VAL] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&i2c_command_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, I2C_CMD_MAX_LEN, sizeof(i2c_command_val), (uint8_t *)i2c_command_val}},

        //I2C -  status characteristic Declaration
        [I2C_STATUS_CHAR] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

        //SPP -  status characteristic Value
        [I2C_STATUS_VAL] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&i2c_status_uuid, ESP_GATT_PERM_READ, I2C_STATUS_MAX_LEN, sizeof(i2c_status_val), (uint8_t *)i2c_status_val}},

        //I2C -  status characteristic - Client Characteristic Configuration Descriptor
        [I2C_STATUS_CFG] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(i2c_status_ccc), (uint8_t *)i2c_status_ccc}},

#ifdef SUPPORT_IMU
        //SPP -  Heart beat characteristic Declaration
        [IMU_DATA_NOTIFY_CHAR] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

        //SPP -  Heart beat characteristic Value
        [IMU_DATA_NTY_VAL] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&i2c_imu_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(i2c_imu_val), sizeof(i2c_imu_val), (uint8_t *)i2c_imu_val}},

        //SPP -  Heart beat characteristic - Client Characteristic Configuration Descriptor
        [IMU_DATA_NTF_CFG] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(i2c_data_notify_ccc), (uint8_t *)i2c_imu_ccc}},
#endif

};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
#ifdef CONFIG_SET_RAW_ADV_DATA
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#else
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#endif
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        /* advertising start complete event to indicate advertising start successfully or failed */
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "advertising start failed");
        }
        else
        {
            ESP_LOGI(GATTS_TABLE_TAG, "advertising start successfully");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "Advertising stop failed");
        }
        else
        {
            ESP_LOGI(GATTS_TABLE_TAG, "Stop adv successfully\n");
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                 param->update_conn_params.status,
                 param->update_conn_params.min_int,
                 param->update_conn_params.max_int,
                 param->update_conn_params.conn_int,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

static uint8_t find_char_and_desr_index(uint16_t handle)
{
    uint8_t error = 0xff;

    for (int i = 0; i < I2C_IDX_NB; i++)
    {
        if (handle == i2c_handle_table[i])
        {
            return i;
        }
    }

    return error;
}

static bool store_wr_buffer(esp_ble_gatts_cb_param_t *p_data)
{
    temp_spp_recv_data_node_p1 = (spp_receive_data_node_t *)malloc(sizeof(spp_receive_data_node_t));

    if (temp_spp_recv_data_node_p1 == NULL)
    {
        ESP_LOGI(GATTS_TABLE_TAG, "malloc error %s %d\n", __func__, __LINE__);
        return false;
    }
    if (temp_spp_recv_data_node_p2 != NULL)
    {
        temp_spp_recv_data_node_p2->next_node = temp_spp_recv_data_node_p1;
    }
    temp_spp_recv_data_node_p1->len = p_data->write.len;
    SppRecvDataBuff.buff_size += p_data->write.len;
    temp_spp_recv_data_node_p1->next_node = NULL;
    temp_spp_recv_data_node_p1->node_buff = (uint8_t *)malloc(p_data->write.len);
    temp_spp_recv_data_node_p2 = temp_spp_recv_data_node_p1;
    memcpy(temp_spp_recv_data_node_p1->node_buff, p_data->write.value, p_data->write.len);
    if (SppRecvDataBuff.node_num == 0)
    {
        SppRecvDataBuff.first_node = temp_spp_recv_data_node_p1;
        SppRecvDataBuff.node_num++;
    }
    else
    {
        SppRecvDataBuff.node_num++;
    }

    return true;
}

static void free_write_buffer(void)
{
    temp_spp_recv_data_node_p1 = SppRecvDataBuff.first_node;

    while (temp_spp_recv_data_node_p1 != NULL)
    {
        temp_spp_recv_data_node_p2 = temp_spp_recv_data_node_p1->next_node;
        free(temp_spp_recv_data_node_p1->node_buff);
        free(temp_spp_recv_data_node_p1);
        temp_spp_recv_data_node_p1 = temp_spp_recv_data_node_p2;
    }

    SppRecvDataBuff.node_num = 0;
    SppRecvDataBuff.buff_size = 0;
    SppRecvDataBuff.first_node = NULL;
}

static void print_write_buffer(void)
{
    temp_spp_recv_data_node_p1 = SppRecvDataBuff.first_node;

    while (temp_spp_recv_data_node_p1 != NULL)
    {
        //uart_write_bytes(UART_NUM_0, (char *)(temp_spp_recv_data_node_p1->node_buff), temp_spp_recv_data_node_p1->len);
        esp_log_buffer_hex(GATTS_TABLE_TAG, temp_spp_recv_data_node_p1->node_buff, temp_spp_recv_data_node_p1->len);
        temp_spp_recv_data_node_p1 = temp_spp_recv_data_node_p1->next_node;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    uint8_t res = 0xff;
    switch (event)
    {
    //建立连接之前，第一步注册 ESP_GATTS_REG_EVT
    case ESP_GATTS_REG_EVT:
    {
        //设置蓝牙设备名称
        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);
        if (set_dev_name_ret)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "set device name failed, error code = %x", set_dev_name_ret);
        }
#ifdef CONFIG_SET_RAW_ADV_DATA
        //配置广播数据
        esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
        if (raw_adv_ret)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
        }
        adv_config_done |= ADV_CONFIG_FLAG;
        esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
        if (raw_scan_ret)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
        }
        adv_config_done |= SCAN_RSP_CONFIG_FLAG;
#else
        //config adv data
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
        }
        adv_config_done |= ADV_CONFIG_FLAG;
        //config scan response data
        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "config scan response data failed, error code = %x", ret);
        }
        adv_config_done |= SCAN_RSP_CONFIG_FLAG;
#endif
        //创建特性列表，启动ESP_GATTS_CREAT_ATTR_TAB_EVT事件
        esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, I2C_IDX_NB, SVC_INST_ID);
        if (create_attr_ret)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "create attr table failed, error code = %x", create_attr_ret);
        }
    }
    break;
    case ESP_GATTS_READ_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_READ_EVT");
        break;
    case ESP_GATTS_WRITE_EVT:
        res = find_char_and_desr_index(param->write.handle);
        if (!param->write.is_prep)
        {
            // the data length of gattc write  must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
            ESP_LOGI(GATTS_TABLE_TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle, param->write.len);
            esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);

            if (res == IMU_DATA_NTY_VAL)
            {
                uint8_t *spp_cmd_buff = NULL;
                spp_cmd_buff = (uint8_t *)malloc((i2c_mtu_size - 3) * sizeof(uint8_t));
                if (spp_cmd_buff == NULL)
                {
                    ESP_LOGE(GATTS_TABLE_TAG, "%s malloc failed\n", __func__);
                    break;
                }
                memset(spp_cmd_buff, 0x0, (i2c_mtu_size - 3));
                memcpy(spp_cmd_buff, param->write.value, param->write.len);
                xQueueSend(cmd_cmd_queue, &spp_cmd_buff, 10 / portTICK_PERIOD_MS);
            }
            else if (res == I2C_DATA_NTF_CFG)
            {
                ESP_LOGE(GATTS_TABLE_TAG, "I2C_DATA_NTF_CFG: %02x \n", param->write.value[0]);
                if ((param->write.len == 2) && (param->write.value[0] == 0x01) && (param->write.value[1] == 0x00))
                {
                    //开启监听
                    enable_data_ntf = true;
                }
                else if ((param->write.len == 2) && (param->write.value[0] == 0x00) && (param->write.value[1] == 0x00))
                {
                    //关闭监听
                    enable_data_ntf = false;
                }
            }
#ifdef SUPPORT_IMU
            else if (res == IMU_DATA_NTF_CFG)
            {
                ESP_LOGE(GATTS_TABLE_TAG, "IMU_DATA_NTF_CFG: %02x \n", param->write.value[0]);
                if ((param->write.len == 2) && (param->write.value[0] == 0x01) && (param->write.value[1] == 0x00))
                {
                    //开启监听
                    enable_imu_ntf = true;
                }
                else if ((param->write.len == 2) && (param->write.value[0] == 0x00) && (param->write.value[1] == 0x00))
                {
                    //关闭监听
                    enable_imu_ntf = false;
                }
            }
#endif

            /* send response when param->write.need_rsp is true*/
            if (param->write.need_rsp)
            {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
            }
        }
        else
        {
            /* handle prepare write */
            store_wr_buffer(param);
            //example_prepare_write_event_env(gatts_if, &prepare_write_env, param);
        }
        break;
    case ESP_GATTS_EXEC_WRITE_EVT:
        // the length of gattc prepare write data must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
        if (param->exec_write.exec_write_flag)
        {
            print_write_buffer();
            free_write_buffer();
        }
        break;
    case ESP_GATTS_MTU_EVT:
        //当连接上以后client以后，产生ESP_GATTS_MTU_EVT事件，回调连接ID和MTU大小
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);

        i2c_conn_id = param->connect.conn_id;
        i2c_gatts_if = gatts_if;
        is_connected = true;

        esp_log_buffer_hex(GATTS_TABLE_TAG, param->connect.remote_bda, 6);
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
        conn_params.latency = 0;
        conn_params.max_int = 0x20; // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x10; // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 400;  // timeout = 400*10ms = 4000ms
        //start sent the update connection parameters to the peer device.
        esp_ble_gap_update_conn_params(&conn_params);
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
        is_connected = false;
        enable_data_ntf = false;
#ifdef SUPPORT_IMU
        enable_imu_ntf = false;
#endif
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
    {
        //建立连接之前，第二步创建服务和调用回调函数 ESP_GATTS_CREAT_ATTR_TAB_EVT
        if (param->add_attr_tab.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
        }
        else if (param->add_attr_tab.num_handle != I2C_IDX_NB)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to HRS_IDX_NB(%d)",
                     param->add_attr_tab.num_handle, I2C_IDX_NB);
        }
        else
        {
            ESP_LOGI(GATTS_TABLE_TAG, "create attribute table successfully, the number handle = %d\n", param->add_attr_tab.num_handle);
            memcpy(i2c_handle_table, param->add_attr_tab.handles, sizeof(i2c_handle_table));
            esp_ble_gatts_start_service(i2c_handle_table[I2C_IDX_SVC]); //启动服务
        }
        break;
    }
    case ESP_GATTS_STOP_EVT:
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    case ESP_GATTS_UNREG_EVT:
    case ESP_GATTS_DELETE_EVT:
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            heart_rate_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        }
        else
        {
            ESP_LOGE(GATTS_TABLE_TAG, "reg app failed, app_id %04x, status %d",
                     param->reg.app_id,
                     param->reg.status);
            return;
        }
    }
    do
    {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++)
        {
            /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == heart_rate_profile_tab[idx].gatts_if)
            {
                if (heart_rate_profile_tab[idx].gatts_cb)
                {
                    heart_rate_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

//********************I2C*****************************

static void disp_buf(uint8_t *buf, int len)
{
    int i;
    for (i = 0; i < len; i++)
    {
        printf("%02x ", buf[i]);
        if ((i + 1) % 16 == 0)
        {
            printf("\n");
        }
    }
    printf("\n");
}

static void i2c_task(void *arg)
{
    //int i = 0;
    int ret;
    uint32_t task_idx = (uint32_t)arg;
    IMU_Data imu_data_rd;
    int cnt = 0;
    while (1)
    {
        ESP_LOGI(I2C_TAG, "TASK[%d] test cnt: %d", task_idx, cnt++);
        //---------------------------------------------------

        xSemaphoreTake(print_mux, portMAX_DELAY);

        ret = read_imu_data(&imu_data_rd);

        if (ret == ESP_ERR_TIMEOUT)
        {
            ESP_LOGE(I2C_TAG, "I2C Timeout");
        }
        else if (ret == ESP_OK)
        {
            printf("*******************\n");
            printf("TASK[%d]  MASTER READ FROM SLAVE\n", task_idx);
            disp_buf(imu_data_rd.spatial_attitude, sizeof(imu_data_rd.spatial_attitude));
            disp_buf(imu_data_rd.appendix_infor, sizeof(imu_data_rd.appendix_infor));
            if (is_connected == true)
            {

                if (enable_imu_ntf == true)
                {
                    esp_ble_gatts_send_indicate(i2c_gatts_if, i2c_conn_id, i2c_handle_table[IMU_DATA_NTY_VAL], sizeof(imu_data_rd.appendix_infor), imu_data_rd.appendix_infor, false);
                    esp_ble_gatts_send_indicate(i2c_gatts_if, i2c_conn_id, i2c_handle_table[IMU_DATA_NTY_VAL], sizeof(imu_data_rd.spatial_attitude), imu_data_rd.spatial_attitude, false);
                }
                else
                {
                    ESP_LOGW(I2C_TAG, "TASK[%d] %s: the ble notify is closed.\n",
                             task_idx, esp_err_to_name(ret));
                }
            }
            else
            {
                ESP_LOGW(I2C_TAG, "TASK[%d] %s: the ble is not connected.\n",
                         task_idx, esp_err_to_name(ret));
            }
        }
        else
        {
            ESP_LOGW(I2C_TAG, "TASK[%d] %s: Master read slave error, IO not connected...\n",
                     task_idx, esp_err_to_name(ret));
        }
        xSemaphoreGive(print_mux);
        vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS * (task_idx + 1)) / portTICK_RATE_MS);
    }
    vSemaphoreDelete(print_mux);
    vTaskDelete(NULL);
}

static esp_err_t i2c_master_init(void)
{
    esp_err_t ret;
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);

    ret = i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);

    if (ret == ESP_OK)
    {
        xTaskCreate(i2c_task, "i2c_task_0", 1024 * 2, (void *)0, 10, NULL);
    }
    else
    {
        ESP_LOGE(I2C_TAG, "i2c driver installed failed, error code = %x", ret);
    }

    return ret;
}

void i2c_cmd_task(void *arg)
{
    uint8_t *cmd_id;

    for (;;)
    {
        vTaskDelay(50 / portTICK_PERIOD_MS);
        if (xQueueReceive(cmd_cmd_queue, &cmd_id, portMAX_DELAY))
        {
            uint8_t IMU_COMMAND = cmd_id[0];
            // esp_log_buffer_char(I2C_TAG, (char *)(cmd_id), strlen((char *)cmd_id));
            ESP_LOGE(GATTS_TABLE_TAG, "cmd_ids: %02x", IMU_COMMAND);
            ad7998_enable();

            // switch (IMU_COMMAND)
            // {
            // case IMU_WAKE_UP:
            // {
            //     esp_err_t ret;
            //     uint8_t imu_command[3] = {0x22, 0x01, 0x00};
            //     // ret = write_imu_command(imu_command, sizeof(imu_command));
            //     if (ret == ESP_OK)
            //     {
            //         ESP_LOGI(I2C_TAG, "IMU wake up sucessfully.");
            //     }
            //     else
            //     {
            //         ESP_LOGE(I2C_TAG, "IMU wake up failed.");
            //     }
            //     break;
            // }

            // default:
            //     break;
            // }            
            free(cmd_id);
        }
    }
    vTaskDelete(NULL);
}

static void i2c_task_init(void)
{
    i2c_master_init();
    cmd_cmd_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(i2c_cmd_task, "i2c_cmd_task", 2048, NULL, 10, NULL);
}

void app_main(void)
{
    esp_err_t ret;

    /* Initialize NVS. */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "gatts register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "gap register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(ESP_APP_ID);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    print_mux = xSemaphoreCreateMutex();
    i2c_task_init();
}
