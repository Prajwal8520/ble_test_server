#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"

#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "example_ble_sec_gatts_demo.h"
#include "driver/gpio.h"

#define GATTS_TABLE_TAG "SEC_GATTS_DEMO"
// #define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40
#define PREPARE_BUF_MAX_SIZE 1024

static float smoothed_rssi = -1.0;
#define ALPHA 0.4

static int connection_id = -1;             // -1 indicates no active connection
static esp_bd_addr_t connected_device_bda; // Store the address of the connected device
//  static int conn_id_save;

// Static variables to store read/write data
#define GATTS_CHAR_VAL_LEN_MAX 0x40
static uint8_t read_char_value[GATTS_CHAR_VAL_LEN_MAX] = "Prajwal";
static uint8_t write_char_value[GATTS_CHAR_VAL_LEN_MAX] = {0};

#define HEART_PROFILE_NUM 1
#define HEART_PROFILE_APP_IDX 0
#define ESP_HEART_RATE_APP_ID 0x55

#define EXAMPLE_DEVICE_NAME "BLE_test_PS"
#define HEART_RATE_SVC_INST_ID 0

// #define GATTS_DEMO_CHAR_VAL_LEN_MAX               500

#define ADV_CONFIG_FLAG (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

typedef struct
{
    uint8_t *prepare_buf;
    int prepare_len;
} prepare_type_env_t;
static prepare_type_env_t prepare_write_env;

static uint8_t adv_config_done = 0;

static uint16_t heart_rate_handle_table[HRS_IDX_NB];

static uint8_t test_manufacturer[3] = {'E', 'S', 'P'};

// Function prototypes (declared before being used)
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void app_main(void);

static uint8_t sec_service_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    // first uuid, 16bit, [12],[13] is the value
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
    0x18,
    0x0D,
    0x00,
    0x00,
};

/*service and characteristics UUID define*/
static const uint8_t BIKE_SERVICE_UUID[16] = {0x20, 0xd8, 0x3e, 0x12, 0x54, 0x1f, 0x94, 0xad, 0x6c, 0x4b, 0x92, 0x81, 0x22, 0x35, 0xc8, 0x2c};
static const uint8_t SPEED_CHARACTERSTICS_UUID[16] = {0x10, 0xd8, 0x3e, 0x12, 0x54, 0x1f, 0x94, 0xad, 0x6c, 0x4b, 0x92, 0x81, 0x22, 0x35, 0xc8, 0x2c};
static const uint8_t GPS_LOCATION_CHARACTERSITICS_UUID[16] = {0x11, 0xd8, 0x3e, 0x12, 0x54, 0x1f, 0x94, 0xad, 0x6c, 0x4b, 0x92, 0x81, 0x22, 0x35, 0xc8, 0x2c};
static const uint8_t UPLOAD_POINT_CHARACTERISTICS_UUID[16] = {0x62, 0xd8, 0x3e, 0x12, 0x54, 0x1f, 0x94, 0xad, 0x6c, 0x4b, 0x92, 0x81, 0x22, 0x35, 0xc8, 0x2c};
/*service and characteristics UUID define*/

// Define your message or data
uint8_t custom_message[] = {'Y', 'A', 'T', 'R', 'I'};

// config adv data
static esp_ble_adv_data_t heart_rate_adv_config = {
    .set_scan_rsp = false,
    .include_txpower = true,
    .include_name = true,
    .min_interval = 0x0006, // slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, // slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = sizeof(custom_message),
    .p_manufacturer_data = custom_message, // Place your message here
    // .manufacturer_len = 0,       // TEST_MANUFACTURER_DATA_LEN,
    // .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(sec_service_uuid),
    .p_service_uuid = sec_service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// config scan response data
static esp_ble_adv_data_t heart_rate_scan_rsp_config = {
    .set_scan_rsp = true,
    .include_name = true,
    .manufacturer_len = sizeof(test_manufacturer),
    .p_manufacturer_data = test_manufacturer,
};

static esp_ble_adv_params_t heart_rate_adv_params = {
    .adv_int_min = 0x100,
    .adv_int_max = 0x100,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_RANDOM,
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

TaskHandle_t rssi_task_handle = NULL; // defined task handle

// Periodic task to read RSSI
void rssi_read_task(void *pvParameters)
{
    printf("rssi_read_task have been triggered \n");
    for (;;)
    {
        // int8_t current_rssi;

        // printf("this is rssi_read_task");
        //  Ensure there is a valid connection
        printf("connection_id is: %d\n", connection_id);
        if (connection_id != -1)
        {
            // Read the RSSI value of the connected device
            // esp_ble_gap_read_rssi(connected_device_bda);
            esp_ble_gap_read_rssi(connected_device_bda);
        }

        // Delay for a period (e.g., every 5 seconds)
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
} // Periodic task to read RSSI

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, // this part not found yet in yatri project-one-communication-dev\gatt_security_server
                                        esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst heart_rate_profile_tab[HEART_PROFILE_NUM] = {
    [HEART_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/*
 *  Heart Rate PROFILE ATTRIBUTES
 ****************************************************************************************
 */

/// Heart Rate Sensor Service
// static const uint16_t heart_rate_svc = ESP_GATT_UUID_HEART_RATE_SVC;

#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
// static const uint8_t char_prop_notify = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_write = ESP_GATT_CHAR_PROP_BIT_WRITE;
// static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE;

/// Heart Rate Sensor Service - Heart Rate Measurement Characteristic, notify
// static const uint16_t heart_rate_meas_uuid = ESP_GATT_HEART_RATE_MEAS;
static const uint8_t heart_measurement_ccc[2] = {0x00, 0x00};

// /// Heart Rate Sensor Service -Body Sensor Location characteristic, read
// static const uint16_t body_sensor_location_uuid = ESP_GATT_BODY_SENSOR_LOCATION;
// static const uint8_t body_sensor_loc_val[1] = {0x00};

// /// Heart Rate Sensor Service - Heart Rate Control Point characteristic, write&read
// static const uint16_t heart_rate_ctrl_point = ESP_GATT_HEART_RATE_CNTL_POINT;
// static const uint8_t heart_ctrl_point[1] = {0x00};

/// Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t heart_rate_gatt_db[HRS_IDX_NB] =
    {
        // Heart Rate Service Declaration
        [bike_service] =
            {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ, sizeof(uint16_t), sizeof(BIKE_SERVICE_UUID), (uint8_t *)&BIKE_SERVICE_UUID}},

        // // Speed Data
        [speed_characteristics_index] =
            {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

        // Speed Data Characteristic Value
        [speed_characteristics_value] =
            {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, (uint8_t *)&SPEED_CHARACTERSTICS_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, HRPS_HT_MEAS_MAX_LEN, 0, NULL}},

        // Speed Data Characteristic - Client Characteristic Configuration Descriptor
        [speed_characteristics_descriptor] =
            {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},

        // GPS_Data
        [gps_data_characteristics_index] =
            {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}}, // can be changed to char_prop_notify

        // GPS Data Characteristic Value
        [gps_data_characteristics_value] =
            {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, (uint8_t *)&GPS_LOCATION_CHARACTERSITICS_UUID, ESP_GATT_PERM_READ, HRPS_HT_MEAS_MAX_LEN, 0, NULL}},

        // GPS Data Characteristic Declaration
        [gps_data_characteristics_descriptor] =
            {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},

        // // CONTROL_POINT
        [control_point_characteristics_index] =
            {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write}},

        // CONTROL_POINT CHARACTERISTICS VALUE
        [control_point_characteristics_value] =
            {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, (uint8_t *)&UPLOAD_POINT_CHARACTERISTICS_UUID, ESP_GATT_PERM_WRITE, HRPS_HT_MEAS_MAX_LEN, 0, NULL}},

        // CONTROL_POINT Characteristic Declaration
        [control_point_characteristics_descriptor] =
            {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},

};

static char *esp_key_type_to_str(esp_ble_key_type_t key_type)
{
    char *key_str = NULL;
    switch (key_type)
    {
    case ESP_LE_KEY_NONE:
        key_str = "ESP_LE_KEY_NONE";
        break;
    case ESP_LE_KEY_PENC:
        key_str = "ESP_LE_KEY_PENC";
        break;
    case ESP_LE_KEY_PID:
        key_str = "ESP_LE_KEY_PID";
        break;
    case ESP_LE_KEY_PCSRK:
        key_str = "ESP_LE_KEY_PCSRK";
        break;
    case ESP_LE_KEY_PLK:
        key_str = "ESP_LE_KEY_PLK";
        break;
    case ESP_LE_KEY_LLK:
        key_str = "ESP_LE_KEY_LLK";
        break;
    case ESP_LE_KEY_LENC:
        key_str = "ESP_LE_KEY_LENC";
        break;
    case ESP_LE_KEY_LID:
        key_str = "ESP_LE_KEY_LID";
        break;
    case ESP_LE_KEY_LCSRK:
        key_str = "ESP_LE_KEY_LCSRK";
        break;
    default:
        key_str = "INVALID BLE KEY TYPE";
        break;
    }

    return key_str;
}

static char *esp_auth_req_to_str(esp_ble_auth_req_t auth_req)
{
    char *auth_str = NULL;
    switch (auth_req)
    {
    case ESP_LE_AUTH_NO_BOND:
        auth_str = "ESP_LE_AUTH_NO_BOND";
        break;
    case ESP_LE_AUTH_BOND:
        auth_str = "ESP_LE_AUTH_BOND";
        break;
    case ESP_LE_AUTH_REQ_MITM:
        auth_str = "ESP_LE_AUTH_REQ_MITM";
        break;
    case ESP_LE_AUTH_REQ_BOND_MITM:
        auth_str = "ESP_LE_AUTH_REQ_BOND_MITM";
        break;
    case ESP_LE_AUTH_REQ_SC_ONLY:
        auth_str = "ESP_LE_AUTH_REQ_SC_ONLY";
        break;
    case ESP_LE_AUTH_REQ_SC_BOND:
        auth_str = "ESP_LE_AUTH_REQ_SC_BOND";
        break;
    case ESP_LE_AUTH_REQ_SC_MITM:
        auth_str = "ESP_LE_AUTH_REQ_SC_MITM";
        break;
    case ESP_LE_AUTH_REQ_SC_MITM_BOND:
        auth_str = "ESP_LE_AUTH_REQ_SC_MITM_BOND";
        break;
    default:
        auth_str = "INVALID BLE AUTH REQ";
        break;
    }

    return auth_str;
}

static void show_bonded_devices(void)
{
    int dev_num = esp_ble_get_bond_device_num();

    esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    esp_ble_get_bond_device_list(&dev_num, dev_list);
    ESP_LOGI(GATTS_TABLE_TAG, "Bonded devices number : %d\n", dev_num);

    ESP_LOGI(GATTS_TABLE_TAG, "Bonded devices list : %d\n", dev_num);
    for (int i = 0; i < dev_num; i++)
    {
        esp_log_buffer_hex(GATTS_TABLE_TAG, (void *)dev_list[i].bd_addr, sizeof(esp_bd_addr_t));
    }

    free(dev_list);
}

static void __attribute__((unused)) remove_all_bonded_devices(void)
{
    int dev_num = esp_ble_get_bond_device_num();

    esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    esp_ble_get_bond_device_list(&dev_num, dev_list);
    for (int i = 0; i < dev_num; i++)
    {
        esp_ble_remove_bond_device(dev_list[i].bd_addr);
    }

    free(dev_list);
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    // ESP_LOGV(GATTS_TABLE_TAG, "GAP_EVT, event %d\n", event);
    printf("GAP_EVT, event %d\n", event);
    switch (event)
    {

    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:

        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT");
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        printf("value of adv_config_done after scan_rsp_config_flag is: %d \n", adv_config_done);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&heart_rate_adv_params);
        }
        break;

    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:

        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT");
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&heart_rate_adv_params);
        }
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:

        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_ADV_START_COMPLETE_EVT");
        // advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "advertising start failed, error status = %x", param->adv_start_cmpl.status);
            break;
        }
        ESP_LOGI(GATTS_TABLE_TAG, "advertising start success");
        break;

    case ESP_GAP_BLE_PASSKEY_REQ_EVT: /* passkey request event */

        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_PASSKEY_REQ_EVT");
        /* Call the following function to input the passkey which is displayed on the remote device */
        // esp_ble_passkey_reply(heart_rate_profile_tab[HEART_PROFILE_APP_IDX].remote_bda, true, 0x00);
        break;

    case ESP_GAP_BLE_OOB_REQ_EVT:
    {

        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_OOB_REQ_EVT");
        uint8_t tk[16] = {1}; // If you paired with OOB, both devices need to use the same tk
        esp_ble_oob_req_reply(param->ble_security.ble_req.bd_addr, tk, sizeof(tk));
        break;
    }

    case ESP_GAP_BLE_LOCAL_IR_EVT: /* BLE local IR event */

        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_LOCAL_IR_EVT");
        break;

    case ESP_GAP_BLE_LOCAL_ER_EVT: /* BLE local ER event */

        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_LOCAL_ER_EVT");
        break;

    case ESP_GAP_BLE_NC_REQ_EVT:

        /* The app will receive this evt when the IO has DisplayYesNO capability and the peer device IO also has DisplayYesNo capability.
        show the passkey number to the user to confirm it with the number displayed by peer device. */
        esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, true);
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_NC_REQ_EVT, the passkey Notify number:%d", param->ble_security.key_notif.passkey);
        break;

    case ESP_GAP_BLE_SEC_REQ_EVT:

        /* send the positive(true) security response to the peer device to accept the security request.
        If not accept the security request, should send the security response with negative(false) accept value*/
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;

    case ESP_GAP_BLE_PASSKEY_NOTIF_EVT: /// the app will receive this evt when the IO  has Output capability and the peer device IO has Input capability.
        /// show the passkey number to the user to input it in the peer device.

        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_PASSKEY_NOTIFY_EVT, The passkey Notify number:%06d", param->ble_security.key_notif.passkey);
        break;

    case ESP_GAP_BLE_KEY_EVT:

        // shows the ble key info share with peer device to the user.
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_KEY_EVT, key type = %s", esp_key_type_to_str(param->ble_security.ble_key.key_type));
        break;

    case ESP_GAP_BLE_AUTH_CMPL_EVT:
    {
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_AUTH_CMPL_EVT");
        esp_bd_addr_t bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTS_TABLE_TAG, "remote BD_ADDR: %08x%04x",
                 (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
                 (bd_addr[4] << 8) + bd_addr[5]);
        ESP_LOGI(GATTS_TABLE_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(GATTS_TABLE_TAG, "pair status = %s", param->ble_security.auth_cmpl.success ? "success" : "fail");
        if (!param->ble_security.auth_cmpl.success)
        {
            ESP_LOGI(GATTS_TABLE_TAG, "fail reason = 0x%x", param->ble_security.auth_cmpl.fail_reason);

            printf("BLE PART REACHED HERE due to failed auth_cmpl_event\n");

            esp_ble_gatts_close(3, 0);
        }
        else
        {
            ESP_LOGI(GATTS_TABLE_TAG, "auth mode = %s", esp_auth_req_to_str(param->ble_security.auth_cmpl.auth_mode));

            // Pairing Successful
            printf("BLE has been successful due to ble_auth_cmpl_evt success \n");

            show_bonded_devices();
            printf("connected_device_bda value is: %s \n", connected_device_bda);
            esp_ble_gap_read_rssi(connected_device_bda);
            // // Start the RSSI reading task
            if (rssi_task_handle == NULL)
            {
                printf("I am inside xtaskcreate");
                xTaskCreatePinnedToCore(rssi_read_task, "rssi_read_task", 2048, NULL, 5, &rssi_task_handle, 1); //&rssi_task_handle);
                // xTaskCreate(rssi_read_task, "rssi_read_task", 2048, NULL, 5, &rssi_task_handle);
            }
            // Start the RSSI reading task
            // printf("I am inside xtaskcreate \n");
            // xTaskCreatePinnedToCore(rssi_read_task, "rssi_read_task", 2048, NULL, 5, &rssi_task_handle, 1); //&rssi_task_handle);
        }
        break;
    }

    case ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT:
    {

        ESP_LOGD(GATTS_TABLE_TAG, "ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT status = %d", param->remove_bond_dev_cmpl.status);
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_REMOVE_BOND_DEV");
        ESP_LOGI(GATTS_TABLE_TAG, "-----ESP_GAP_BLE_REMOVE_BOND_DEV----");
        esp_log_buffer_hex(GATTS_TABLE_TAG, (void *)param->remove_bond_dev_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTS_TABLE_TAG, "------------------------------------");
        break;
    }

    case ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT:

        if (param->local_privacy_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "config local privacy failed, error status = %x", param->local_privacy_cmpl.status);
            break;
        }

        esp_err_t ret = esp_ble_gap_config_adv_data(&heart_rate_adv_config);
        if (ret)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
        }
        else
        {
            adv_config_done |= ADV_CONFIG_FLAG;
        }

        ret = esp_ble_gap_config_adv_data(&heart_rate_scan_rsp_config);
        if (ret)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
        }
        else
        {
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
        }

        break;

    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT");
        ESP_LOGI(GATTS_TABLE_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                 param->update_conn_params.status,
                 param->update_conn_params.min_int,
                 param->update_conn_params.max_int,
                 param->update_conn_params.conn_int,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
        break;

    case ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT: // Event where RSSI is returned

        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT");
        if (param->read_rssi_cmpl.status == ESP_BT_STATUS_SUCCESS)
        {
            // ESP_LOGI(GATTS_TABLE_TAG, "RSSI value is : %d", param->read_rssi_cmpl.rssi);
            int8_t current_rssi = param->read_rssi_cmpl.rssi;

            // making rssi value smooth starts here
            if (smoothed_rssi == -1.0)
            {
                smoothed_rssi = current_rssi; // first rssi reading
            }
            else
            {
                // apply the exponential moving filter
                smoothed_rssi = (ALPHA * current_rssi) + ((1 - ALPHA)) * smoothed_rssi;
            }

            ESP_LOGI(GATTS_TABLE_TAG, "Current RSSI: %d , Smoothed RSSI value is : %.2f", current_rssi, smoothed_rssi);
            if (smoothed_rssi >= -60)
            {
                gpio_set_level(GPIO_NUM_2, 1);
                printf("light on \n");
            }
            else
            {
                gpio_set_level(GPIO_NUM_2, 0);
                printf("light off \n");
            }
        }
        else
        {
            ESP_LOGE(GATTS_TABLE_TAG, "Failed to read RSSI");
        }
        break;
    default:
        break;
    }
}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_TABLE_TAG, "EXAMPLW_EXEC_WRITE_EVENT_ENV");

    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && prepare_write_env->prepare_buf)
    {
        esp_log_buffer_hex(GATTS_TABLE_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }
    else
    {
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf)
    {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

// Handle read event by sending data to app
//  void handle_read_event(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
//      ESP_LOGI(GATTS_TABLE_TAG, "Handling read event...");

//     esp_gatt_rsp_t rsp;
//     memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
//     rsp.attr_value.handle = param->read.handle;
//     rsp.attr_value.len = sizeof(read_char_value);  // Length of data to be sent
//     memcpy(rsp.attr_value.value, read_char_value, rsp.attr_value.len);

//     esp_err_t err = esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
//     if (err != ESP_OK) {
//         ESP_LOGE(GATTS_TABLE_TAG, "Error sending response: %s", esp_err_to_name(err));
//     }
// }

// Handle read event by sending the stored characteristic value to the client
void handle_read_event(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_gatt_rsp_t rsp;
    memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
    rsp.attr_value.handle = param->read.handle;
    rsp.attr_value.len = strlen((char *)read_char_value);              // Length of the data
    memcpy(rsp.attr_value.value, read_char_value, rsp.attr_value.len); // Data to send
    esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
}

// Function to convert byte array to text (string)
void byte_array_to_text(const uint8_t *byte_array, size_t len)
{
    // Create a buffer large enough to store the string and a null terminator
    char text_buffer[len + 1];

    // Copy the byte array into the text buffer
    memcpy(text_buffer, byte_array, len);

    // Add a null terminator to the string
    text_buffer[len] = '\0';

    // Print the resulting text
    printf("Converted text: %s\n", text_buffer);
}

// Handle write event by receiving the data from the client
void handle_write_event(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_TABLE_TAG, "I am inside handle_write_event");
    if (param->write.need_rsp)
    {
        esp_gatt_rsp_t rsp;
        // esp_ble_conn_update_params_t conn_params = {0}
        // esp_gatt_rsp_t write_char_value;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->write.handle;
        rsp.attr_value.len = param->write.len;
        // printf("incoming value is: %d", param);
        ESP_LOGI(GATTS_TABLE_TAG, "Incoming data is: ");
        esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, &rsp);

        // Store the received data in the `write_char_value`
        memcpy(write_char_value, param->write.value, param->write.len);
        printf("Received write data: ");
        for (int i = 0; i < param->write.len; i++)
        {
            printf("%02x ", param->write.value[i]); // Print each byte as a two-digit hexadecimal number
        }
        printf("\n");
        byte_array_to_text(param->write.value, param->write.len);
    }

    // printf("Data stored in write_char_value: ");
    // for (int i = 0; i < param->write.len; i++)
    // {
    //     printf("%02x ", write_char_value[i]); // Print each byte as a two-digit hexadecimal number
    // }
    // printf("\n");

    // Sending notification to the client
    const char *notify_data = "Message received"; // Data to send as notification
    uint16_t data_len = strlen(notify_data);      // Length of the notification data

    // Use the correct characteristic handle for notification
    uint16_t char_handle = heart_rate_handle_table[speed_characteristics_value]; // Access correct handle

    // Send notification to the client
    esp_err_t err = esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, char_handle, data_len, (uint8_t *)notify_data, false);

    if (err != ESP_OK)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "Error sending notification: %s", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI(GATTS_TABLE_TAG, "Notification sent successfully: %s", notify_data);
    }
}

// void BLE_notify_data(char *value, int characteristic_value, char *value_source)
// {
//     printf("NOTIFICATION SOURCE\n%s\nDATA\n%s\n" RST, value_source, value);
//     // printf(BLE_STATUS "DATA\n%s\n" RST, value);
//     esp_ble_gatts_send_indicate(3, 0, heart_rate_handle_table[characteristic_value], strlen(value), (uint8_t *)value, false);
// }
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    // ESP_LOGV(GATTS_TABLE_TAG, "event = %x\n", event);
    printf("gatts_profile_event_handler event: %x\n", event);
    switch (event)
    {

    case ESP_GATTS_REG_EVT:

        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_REG_EVT, status: %d, app_id: %d\n", param->reg.status, param->reg.app_id);

        esp_ble_gap_set_device_name(EXAMPLE_DEVICE_NAME);
        printf("Device name is: %s\n", EXAMPLE_DEVICE_NAME);
        // generate a resolvable random address
        esp_ble_gap_config_local_privacy(true);
        esp_ble_gatts_create_attr_tab(heart_rate_gatt_db, gatts_if,
                                      HRS_IDX_NB, HEART_RATE_SVC_INST_ID);
        break;

    case ESP_GATTS_WRITE_EVT:

        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_WRITE_EVT, write value:");
        esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
        handle_write_event(gatts_if, param); // Call the function to handle write
        break;

    case ESP_GATTS_READ_EVT:
        // ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_READ_EVT");
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_READ_EVT, conn_id: %d, trans_id: %d, handle: %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
        handle_read_event(gatts_if, param);
        // ESP_LOGI(GATTS_TABLE_TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
        // esp_gatt_rsp_t rsp;
        // memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        // rsp.attr_value.handle = param->read.handle;
        // rsp.attr_value.len = 4;

        // //Hier muss je nach verwendetem Handle ausgelesen werden
        // rsp.attr_value.value[0] = 0x16;
        // rsp.attr_value.value[1] = 0x54;
        // rsp.attr_value.value[2] = 0x32;
        // rsp.attr_value.value[3] = 0x61;
        // esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
        // ESP_GATT_OK, &rsp);
        break;

    case ESP_GATTS_EXEC_WRITE_EVT:

        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
        example_exec_write_event_env(&prepare_write_env, param);
        break;

    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_MTU_EVT");
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_MTU_EVT, MTU size = %d", param->mtu.mtu);
        break;

    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONF_EVT");
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONF_EVT, status = %d \n", param->conf.status);
        break;

    case ESP_GATTS_UNREG_EVT:

        break;

    case ESP_GATTS_DELETE_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DELETE_EVT");
        break;

    case ESP_GATTS_START_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_START_EVT");
        esp_ble_gap_start_advertising(&heart_rate_adv_params); // advertise once again
        break;

    case ESP_GATTS_STOP_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_STOP_EVT");
        break;

    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT");
        esp_ble_conn_update_params_t conn_params = {
            .latency = 0,
            .max_int = 0x30,
            .min_int = 0x10,
            .timeout = 400};
        // /* start security connect with peer device when receive the connect event sent by the master */
        esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        memcpy(connected_device_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));

        // // Start the RSSI reading task
        // if (rssi_task_handle == NULL)
        // {
        //     printf("I am inside xtaskcreate");
        //     xTaskCreate(rssi_read_task, "rssi_read_task", 2048, NULL, 5, &rssi_task_handle);
        // }

        /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
        conn_params.latency = 0;
        conn_params.max_int = 0x30; // max_int = 0x30*1.25ms = 40ms
        conn_params.min_int = 0x10; // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 400;  // timeout = 400*10ms = 4000ms
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONN_EVT, conn_id: %d\n", param->connect.conn_id);
        connection_id = param->connect.conn_id;
        // ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
        //          param->connect.conn_id,
        //          param->connect.remote_bda[0],
        //          param->connect.remote_bda[1],
        //          param->connect.remote_bda[2],
        //          param->connect.remote_bda[3],
        //          param->connect.remote_bda[4],
        //          param->connect.remote_bda[5]);
        // heart_rate_profile_tab[ESP_HEART_RATE_APP_ID].conn_id = param->connect.conn_id;

        // start sent the update connection parameters to the peer device.
        esp_ble_gap_update_conn_params(&conn_params);
        // Request RSSI of the connected device
        // esp_ble_gap_read_rssi(param->connect.remote_bda);
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason: 0x%x", param->disconnect.reason);

        // Stop RSSI reading task
        if (rssi_task_handle != NULL)
        {
            vTaskDelete(rssi_task_handle);
            rssi_task_handle = NULL;
        }

        // Clear the connection ID or mark the connection as invalid
        connection_id = -1; // Assuming connection_id is -1 when there is no connection
                            // Optionally restart advertising
        esp_ble_gap_start_advertising(&heart_rate_adv_params);
        break;

    case ESP_GATTS_OPEN_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_OPEN_EVT");
        break;

    case ESP_GATTS_CANCEL_OPEN_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CANCLE_OPEN_EVT");
        break;

    case ESP_GATTS_CLOSE_EVT:
        printf("BLE STATUS :: DEVICE IS DISCONNECTED.");
        break;

    case ESP_GATTS_LISTEN_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_LISTEN_EVT");
        break;

    case ESP_GATTS_CONGEST_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONGEST_EVT");
        break;

    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
    {
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CREAT_ATTR_TAB_EVT, The number handle = %x", param->add_attr_tab.num_handle);
        if (param->create.status == ESP_GATT_OK)
        {
            if (param->add_attr_tab.num_handle == HRS_IDX_NB)
            {
                memcpy(heart_rate_handle_table, param->add_attr_tab.handles,
                       sizeof(heart_rate_handle_table));
                esp_ble_gatts_start_service(heart_rate_handle_table[bike_service]);
                printf("BLE_STATUS :: STARTED SERVICE SUCCESSFULLY HERE :: \n");
            }
            else
            {
                ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table abnormally, num_handle (%d) doesn't equal to HRS_IDX_NB(%d)",
                         param->add_attr_tab.num_handle, HRS_IDX_NB);
            }
        }
        else
        {
            ESP_LOGE(GATTS_TABLE_TAG, " Create attribute table failed, error code = %x", param->create.status);
        }
        break;
    }

    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            heart_rate_profile_tab[HEART_PROFILE_APP_IDX].gatts_if = gatts_if;
        }
        else
        {
            ESP_LOGI(GATTS_TABLE_TAG, "Reg app failed, app_id %04x, status %d\n",
                     param->reg.app_id,
                     param->reg.status);
            return;
        }
    }

    do
    {
        int idx;
        for (idx = 0; idx < HEART_PROFILE_NUM; idx++)
        {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                gatts_if == heart_rate_profile_tab[idx].gatts_if)
            {
                if (heart_rate_profile_tab[idx].gatts_cb)
                {
                    heart_rate_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void app_main(void)
{

    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);

    esp_err_t ret;

    // Initialize NVS.
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
        ESP_LOGE(GATTS_TABLE_TAG, "%s init controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    else
    {
        ESP_LOGI(GATTS_TABLE_TAG, "controller initalized");
    }
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    else
    {
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_BT_MODE_BLE_ENABLED");
    }

    ESP_LOGI(GATTS_TABLE_TAG, "%s init bluetooth", __func__);
    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    else
    {
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_BLUEDROID_INITALIZED");
    }
    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    else
    {
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_BLUEDROID_ENABLED");
    }

    // esp_bd_addr_t random_addr[6] = {0xC0, 0xDE, 0xFA, 0xCE, 0xDE, 0xAD}; // Example random address
    uint8_t random_addr[6] = {0xC0, 0xDE, 0xFA, 0xCE, 0xDE, 0xAD}; // Example random address

    ret = esp_ble_gap_set_rand_addr(random_addr);
    if (ret == ESP_OK)
    {
        // ESP_LOGI("BLE", "Random address set successfully.");
        ESP_LOGI("BLE", "Random address set successfully: %02X:%02X:%02X:%02X:%02X:%02X",
                 random_addr[0], random_addr[1], random_addr[2],
                 random_addr[3], random_addr[4], random_addr[5]);
    }
    else
    {
        ESP_LOGE("BLE", "Failed to set random address: %s", esp_err_to_name(ret));
    }
    ret = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9); // to set advertizing power...
    if (ret != ESP_OK)
    {
        ESP_LOGE("TX POWER", "Error setting TX power: %s", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(GATTS_TABLE_TAG, "esp_ble_tx_power_set to P9...");
    }
    ret = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, ESP_PWR_LVL_P9);
    if (ret)
    {
        ESP_LOGE("TX POWER", "Error setting TX power: %s", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(GATTS_TABLE_TAG, "esp_ble_tx_power_set to P9..."); // to set connection power...
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "gatts register error, error code = %x", ret);
        return;
    }
    else
    {
        ESP_LOGI(GATTS_TABLE_TAG, "GATTS_EVENT_HANDLER CALLED");
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "gap register error, error code = %x", ret);
        return;
    }
    else
    {
        ESP_LOGI(GATTS_TABLE_TAG, "GAP_EVENT_HANDLER CALLED");
    }
    printf("esp_ble_gatts_app_register called ESP_GATTS_REG_EVT \n");
    ret = esp_ble_gatts_app_register(ESP_HEART_RATE_APP_ID);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "gatts app register error, error code = %x", ret);
        return;
    }
    else
    {
        ESP_LOGI(GATTS_TABLE_TAG, "PROFILE REGISTERED");
    }

    /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND; // bonding with peer device after authentication
    esp_ble_io_cap_t iocap = ESP_IO_CAP_OUT;                    // ESP_IO_CAP_NONE    ;           //set the IO capability to No output No input
    uint8_t key_size = 16;                                      // the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    // set static passkey
    uint32_t passkey = rand();
    uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;
    uint8_t oob_support = ESP_BLE_OOB_DISABLE;
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob_support, sizeof(uint8_t));
    /* If your BLE device acts as a Slave, the init_key means you hope which types of key of the master should distribute to you,
    and the response key means which key you can distribute to the master;
    If your BLE device acts as a master, the response key means you hope which types of key of the slave should distribute to you,
    and the init key means which key you can distribute to the slave. */
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    /* Just show how to clear all the bonded devices
     * Delay 30s, clear all the bonded devices
     *
     * vTaskDelay(30000 / portTICK_PERIOD_MS);
     * remove_all_bonded_devices();
     */
}