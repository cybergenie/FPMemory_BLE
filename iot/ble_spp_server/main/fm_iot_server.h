/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * DEFINES
 ****************************************************************************************
 */
//#define SUPPORT_HEARTBEAT
//#define SENSOR_DEBUG_MODE

#define sensor_sprintf(s,...)         sprintf((char*)(s), ##__VA_ARGS__)
#define SENSOR_DATA_MAX_LEN           (512)
#define SENSOR_CMD_MAX_LEN            (20)
#define SENSOR_STATUS_MAX_LEN         (20)
#define SENSOR_DATA_BUFF_MAX_LEN      (2*1024)
///Attributes State Machine
enum{
    SENSOR_IDX_SVC,

    SENSOR_IDX_SENSOR_DATA_RECV_CHAR,
    SENSOR_IDX_SENSOR_DATA_RECV_VAL,

    SENSOR_IDX_SENSOR_DATA_NOTIFY_CHAR,
    SENSOR_IDX_SENSOR_DATA_NTY_VAL,
    SENSOR_IDX_SENSOR_DATA_NTF_CFG,

    SENSOR_IDX_SENSOR_COMMAND_CHAR,
    SENSOR_IDX_SENSOR_COMMAND_VAL,

    SENSOR_IDX_SENSOR_STATUS_CHAR,
    SENSOR_IDX_SENSOR_STATUS_VAL,
    SENSOR_IDX_SENSOR_STATUS_CFG,

    SENSOR_IDX_NB,
};

#define SENSOR_PROFILE_NUM             1
#define SENSOR_PROFILE_APP_IDX         0
#define ESP_SENSOR_APP_ID              0x56
#define SAMPLE_DEVICE_NAME          "ESP_SENSOR_SERVER"    //The Device Name Characteristics in GAP
#define SENSOR_SVC_INST_ID	            0

/// SENSOR Service
static const uint16_t sensor_service_uuid = 0xABF0;
/// Characteristic UUID
#define ESP_GATT_UUID_SENSOR_DATA_RECEIVE      0xABF1
#define ESP_GATT_UUID_SENSOR_DATA_NOTIFY       0xABF2
#define ESP_GATT_UUID_SENSOR_COMMAND_RECEIVE   0xABF3
#define ESP_GATT_UUID_SENSOR_COMMAND_NOTIFY    0xABF4

/*
 *  SENSOR PROFILE ATTRIBUTES
 ****************************************************************************************
 */

#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE_NR|ESP_GATT_CHAR_PROP_BIT_READ;



///SENSOR Service - data receive characteristic, read&write without response
static const uint16_t sensor_data_receive_uuid = ESP_GATT_UUID_SENSOR_DATA_RECEIVE;
static const uint8_t  sensor_data_receive_val[20] = {0x00};

///SENSOR Service - data notify characteristic, notify&read
static const uint16_t sensor_data_notify_uuid = ESP_GATT_UUID_SENSOR_DATA_NOTIFY;
static const uint8_t  sensor_data_notify_val[20] = {0x00};
static const uint8_t  sensor_data_notify_ccc[2] = {0x00, 0x00};

///SENSOR Service - command characteristic, read&write without response
static const uint16_t sensor_command_uuid = ESP_GATT_UUID_SENSOR_COMMAND_RECEIVE;
static const uint8_t  sensor_command_val[10] = {0x00};

///SENSOR Service - status characteristic, notify&read
static const uint16_t sensor_status_uuid = ESP_GATT_UUID_SENSOR_COMMAND_NOTIFY;
static const uint8_t  sensor_status_val[10] = {0x00};
static const uint8_t  sensor_status_ccc[2] = {0x00, 0x00};

static uint16_t sensor_mtu_size = 23;
static uint16_t sensor_conn_id = 0xffff;
static esp_gatt_if_t sensor_gatts_if = 0xff;

static xQueueHandle sensor_data_queue = NULL; //传感器数据缓存队列


static bool enable_data_ntf = false;
static bool is_connected = false;
static esp_bd_addr_t sensor_remote_bda = {0x0,};

static uint16_t sensor_handle_table[SENSOR_IDX_NB];

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

struct gatts_profile_inst {
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

static esp_ble_adv_params_t sensor_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst sensor_profile_tab[SENSOR_PROFILE_NUM] = {
    [SENSOR_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

///Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t sensor_gatt_db[SENSOR_IDX_NB] =
{
    //SENSOR -  Service Declaration
    [SENSOR_IDX_SVC]                      	=
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
    sizeof(sensor_service_uuid), sizeof(sensor_service_uuid), (uint8_t *)&sensor_service_uuid}},

    //SENSOR -  data receive characteristic Declaration
    [SENSOR_IDX_SENSOR_DATA_RECV_CHAR]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    //SENSOR -  data receive characteristic Value
    [SENSOR_IDX_SENSOR_DATA_RECV_VAL]             	=
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&sensor_data_receive_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    SENSOR_DATA_MAX_LEN,sizeof(sensor_data_receive_val), (uint8_t *)sensor_data_receive_val}},

    //SENSOR -  data notify characteristic Declaration
    [SENSOR_IDX_SENSOR_DATA_NOTIFY_CHAR]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    //SENSOR -  data notify characteristic Value
    [SENSOR_IDX_SENSOR_DATA_NTY_VAL]   =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&sensor_data_notify_uuid, ESP_GATT_PERM_READ,
    SENSOR_DATA_MAX_LEN, sizeof(sensor_data_notify_val), (uint8_t *)sensor_data_notify_val}},

    //SENSOR -  data notify characteristic - Client Characteristic Configuration Descriptor
    [SENSOR_IDX_SENSOR_DATA_NTF_CFG]         =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),sizeof(sensor_data_notify_ccc), (uint8_t *)sensor_data_notify_ccc}},

    //SENSOR -  command characteristic Declaration
    [SENSOR_IDX_SENSOR_COMMAND_CHAR]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    //SENSOR -  command characteristic Value
    [SENSOR_IDX_SENSOR_COMMAND_VAL]                 =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&sensor_command_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    SENSOR_CMD_MAX_LEN,sizeof(sensor_command_val), (uint8_t *)sensor_command_val}},

    //SENSOR -  status characteristic Declaration
    [SENSOR_IDX_SENSOR_STATUS_CHAR]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    //SENSOR -  status characteristic Value
    [SENSOR_IDX_SENSOR_STATUS_VAL]                 =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&sensor_status_uuid, ESP_GATT_PERM_READ,
    SENSOR_STATUS_MAX_LEN,sizeof(sensor_status_val), (uint8_t *)sensor_status_val}},

    //SENSOR -  status characteristic - Client Characteristic Configuration Descriptor
    [SENSOR_IDX_SENSOR_STATUS_CFG]         =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),sizeof(sensor_status_ccc), (uint8_t *)sensor_status_ccc}},

};

static const uint8_t sensor_adv_data[25] = {
    /* Flags */
    0x02,0x01,0x06,
    /* Complete List of 16-bit Service Class UUIDs */
    0x03,0x03,0xF0,0xAB,
    /* Complete Local Name in advertising */
    0x0F,0x09, 'E', 'S', 'P', '_', 'S', 'E', 'N','O','R', '_', 'S', 'E', 'R','V', 'E', 'R'
};



