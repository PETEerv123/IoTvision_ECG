/*
@name: Ta Thuan
@brief: ECG_Device intergrated Electrocardiagram & Accelar
@Version Firmware : IoTVsion V2.0
*/

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "math.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_task_wdt.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_err.h"

/*Private Library*/
#include "esp_timer.h"
#include "Max30003.h"
#include "Config.h"
#include "ADXL345.h"
// #include "Bluetooth.h"
/* BLE Library */
#include "esp_bt.h"
// #include "esp_gap_ble_api.h"
// #include "esp_gatts_api.h"
// #include "esp_bt_defs.h"
// #include "esp_bt_main.h"
// #include "esp_bt_device.h"
// #include "esp_gatt_common_api.h"
// #include "Bluetooth.h"
/* NimBLE stack APIs */
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
/* NimBLE GATT APIs */
#include "host/ble_gatt.h"
#include "services/gatt/ble_svc_gatt.h"
#include "host/ble_gap.h"
#include "services/gap/ble_svc_gap.h"

const char *Tag_ECG = "Process ECG Device";

SemaphoreHandle_t ECG_Electriocal_diagram;
QueueHandle_t ble_queue;
#define MaximumSample_ECG 50
#define NumberSample_Bat 100

#define BLE_GAP_APPEARANCE_GENERIC_TAG 0x0200
#define BLE_GAP_URI_PREFIX_HTTPS 0x17
#define BLE_GAP_LE_ROLE_PERIPHERAL 0x00

static uint8_t esp_uri[] = {BLE_GAP_URI_PREFIX_HTTPS, '/', '/', 'I', 'o', 'T', 'V', 'I', 'S', 'I', 'O', 'N', '.', 'c', 'o', 'm'}; // https://IoTVision.com
/**/
static uint8_t own_addr_type = 0;
static uint8_t addr_val[6] = {0};

static uint16_t heart_rate_chr_conn_handle = 0;
static bool heart_rate_chr_conn_handle_inited = false;
static bool heart_rate_ind_status = false;

uint8_t percBat = 0;
bool Flag_Sample = false;
float mag = 0;
uint8_t MACMasterDungDinhDang[6];
const char *MACMaster = "244CABF96330"; // 90380CA3B9DC 244CABF96330
Status_ECG_Handle ECG_Status;
typedef struct
{
    int32_t EcgWave[MaximumSample_ECG]; // 4 byte * 60 //  byte // 240
    int16_t HR;                         // 2 byte  0~256 bpm
    int16_t RR;                         // 2 byte
    int8_t Bat;                         // 1 byte // mức độ pin tiêu thụ 0-100%
    uint16_t Accel;                     // 2 byte
} __attribute__((packed)) ECGFIFO;
ECGFIFO FIFO;
Kalman1D_t kfMag;
MAX30003_handle_t MAX30003_handle;
esp_now_peer_info_t peerInfo; // khai báo để add thiết bị vào
volatile bool Trigger_Event = false;
const char *TAG = "BLuetooth";
static int gap_event_handler(struct ble_gap_event *event, void *arg);
static int heart_rate_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg);

static void start_advertising(void);
void send_ecg_ble_notification(ECGFIFO *_FIFO);
void adv_init(void);
void ble_store_config_init(void);
void gatt_svr_subscribe_cb(struct ble_gap_event *event);
static int16_t heart_rate_chr_val;
static uint16_t heart_rate_chr_val_handle;
/* Heart rate service */
static const ble_uuid16_t heart_rate_svc_uuid = BLE_UUID16_INIT(0x0259);
static const ble_uuid16_t heart_rate_chr_uuid = BLE_UUID16_INIT(0x2A37);

// static const ble_uuid128_t service_uuid =
//     BLE_UUID128_INIT(0xfb,0x34,0x9b,0x5f,0x80,0x00,0x00,0x80,0x00,0x10,0x00,0x00,0x59,0x02,0x00,0x00);

// static const ble_uuid128_t char_uuid =
//     BLE_UUID128_INIT(0xfb,0x34,0x9b,0x5f,0x80,0x00,0x00,0x80,0x00,0x10,0x00,0x00,0x37,0x2A,0x00,0x00);

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = &heart_rate_svc_uuid.u,
     .characteristics = (struct ble_gatt_chr_def[]){
         {
             .uuid = &heart_rate_chr_uuid.u,
             .access_cb = heart_rate_chr_access,                   // hàm call back xử lý mọi yêu cầu của client
             .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY, // cho phép sever xử lý gửi dữ liệu tới cho client
             .val_handle = &heart_rate_chr_val_handle,
         },
         {0}}},
    {0}};
static int heart_rate_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    /* Local variables */
    int rc;

    /* Handle access events */
    /* Note: Heart rate characteristic is read only */
    switch (ctxt->op)
    {

    /* Read characteristic event */
    case BLE_GATT_ACCESS_OP_READ_CHR:
        /* Verify connection handle */
        if (conn_handle != BLE_HS_CONN_HANDLE_NONE)
        {
            ESP_LOGI(TAG, "characteristic read; conn_handle=%d attr_handle=%d",
                     conn_handle, attr_handle);
        }
        else
        {
            ESP_LOGI(TAG, "characteristic read by nimble stack; attr_handle=%d",
                     attr_handle);
        }

        // /* Verify attribute handle */
        // if (attr_handle == heart_rate_chr_val_handle)
        // {
        //     /* Update access buffer value */
        //     heart_rate_chr_val = FIFO.HR;
        //     rc = os_mbuf_append(ctxt->om, &heart_rate_chr_val,
        //                         sizeof(heart_rate_chr_val));
        //     return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        // }
        goto error;

    /* Unknown event */
    default:
        goto error;
    }

error:
    ESP_LOGE(
        TAG,
        "unexpected access operation to heart rate characteristic, opcode: %d",
        ctxt->op);
    return BLE_ATT_ERR_UNLIKELY;
}

void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    /* Local variables */
    char buf[BLE_UUID_STR_LEN];

    /* Handle GATT attributes register events */
    switch (ctxt->op)
    {

    /* Service register event */
    case BLE_GATT_REGISTER_OP_SVC:
        ESP_LOGD(TAG, "registered service %s with handle=%d",
                 ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                 ctxt->svc.handle);
        break;

    /* Characteristic register event */
    case BLE_GATT_REGISTER_OP_CHR:
        ESP_LOGD(TAG,
                 "registering characteristic %s with "
                 "def_handle=%d val_handle=%d",
                 ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                 ctxt->chr.def_handle, ctxt->chr.val_handle);
        break;

    /* Descriptor register event */
    case BLE_GATT_REGISTER_OP_DSC:
        ESP_LOGD(TAG, "registering descriptor %s with handle=%d",
                 ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                 ctxt->dsc.handle);
        break;

    /* Unknown event */
    default:
        assert(0);
        break;
    }
}

static void on_stack_reset(int reason)
{
    /* On reset, print reset reason to console */
    ESP_LOGI(TAG, "nimble stack reset, reset reason: %d", reason);
}

static void on_stack_sync(void)
{
    /* On stack sync, do advertising initialization */
    adv_init();
}
#pragma region initial_Bluetooth_GATT
void Init_Bluetooth_GATT_Sever(void)
{
    static const char *GAP_TAG = "ECG Device";
    // esp_err_t ret;
    /* NimBLE stack initialization */
    nimble_port_init();
    /* Call NimBLE GAP initialization API */
    ble_svc_gap_init();
    /* Set GAP device name */
    ble_svc_gap_device_name_set(GAP_TAG);
    /* GATT server initialization */
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svr_svcs);
    ble_gatts_add_svcs(gatt_svr_svcs);
    /* NimBLE host configuration initialization */

    /* Set host callbacks */
    ble_hs_cfg.reset_cb = on_stack_reset;
    ble_hs_cfg.sync_cb = on_stack_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /* Store host configuration */
    ble_store_config_init();
}
#pragma endregion initial_Bluetooth_GATT
#pragma region BAT_CONFIG
void Bat_Config(void)
{
    const char tag[] = "ADC BAT";
    esp_err_t ret;

    // ret = adc1_config_width(ADC_WIDTH_BIT_12);
    // if (ret != ESP_OK)
    // {
    //     // printf("Cau hinh ADC BAT that bai ");
    //     ESP_LOGI(tag,"Cau hinh ADC BAT dải 4096 bit thât bại :%d ", ret);
    // }
    ret = adc1_config_channel_atten((gpio_num_t)PIN_BATTERY, ADC_ATTEN_DB_12);
    if (ret != ESP_OK)
    {
        // printf("Cau hinh ADC BAT dải 4096 bit thât bại ");
        ESP_LOGI(tag, "Cau hinh ADC BAT dải 4096 bit thât bại :%d ", ret);
    }
    if (ret == ESP_OK)
    {
        printf("Cau hinh BAT thanh cong\n ");
    }
}
#pragma endregion BAT_CONFIG
#pragma region Led_State
void Led_State_init(void)
{
    gpio_config_t Ledcfg = {
        .pin_bit_mask = (1ULL << led_pin_state),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&Ledcfg);
    gpio_set_level((gpio_num_t)led_pin_state, 0);
    // ESP_LOGI("LED State","khoi tao thanh cong Led State");
}
#pragma endregion Led_State
#pragma region WIFI&Esp_now_configure_chanel
static void _wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK(esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
#endif
}
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {};
void esp32_now_init(void)
{
    // tiết kiệm pin ở chế độ cao nhất
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MAX_MODEM));
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    /*trường hợp ở chế độ AP*/
#if CONFIG_ESPNOW_ENABLE_POWER_SAVE
    ESP_ERROR_CHECK(esp_now_set_wake_window(CONFIG_ESPNOW_WAKE_WINDOW));
    ESP_ERROR_CHECK(esp_wifi_connectionless_module_set_wake_interval(CONFIG_ESPNOW_WAKE_INTERVAL));
#endif

    for (int i = 0; i < 6; ++i)
    {
        sscanf(MACMaster + 2 * i, "%2hhx", &MACMasterDungDinhDang[i]);
    }
    memcpy(peerInfo.peer_addr, MACMasterDungDinhDang, 6);
    peerInfo.channel = 1; // current channel
    peerInfo.encrypt = false;
    esp_err_t status = esp_now_add_peer(&peerInfo);
    if (status == ESP_OK)
    {
        ESP_LOGI("ESP-NOW", "Add peer SUCCESS");
    }
    else
    {
        ESP_LOGE("ESP-NOW", "Add peer FAILED, err = %s", esp_err_to_name(status));
    }
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    printf("MAC GATEWAY Device: %02X:%02X:%02X:%02X:%02X:%02X\n",
           MACMasterDungDinhDang[0], MACMasterDungDinhDang[1], MACMasterDungDinhDang[2],
           MACMasterDungDinhDang[3], MACMasterDungDinhDang[4], MACMasterDungDinhDang[5]);
    printf("MAC Address Device: %02X:%02X:%02X:%02X:%02X:%02X\n",
           mac[0], mac[1], mac[2],
           mac[3], mac[4], mac[5]);
}
#pragma endregion WIFI &Esp_now_configure_chanel
#pragma region MAX30003_Status
void MAX300003_STATUS_Sample(Status_ECG_Handle *Status)
{
    uint8_t regReadBuff[4];
    esp_err_t err = max30003RegRead(STATUS, regReadBuff);
    if (err != ESP_OK)
    {
        ESP_LOGE("MAX30003 Status", "Read ERR: %02X", err);
        // return false;
    }

    uint32_t data = (regReadBuff[1] << 16) |
                    (regReadBuff[2] << 8) |
                    regReadBuff[3];

    Status->_EINT = (data & EINT) ? 1 : 0;
    Status->_EOVF = (data & EOVF) ? 1 : 0;
    Status->_FSTINT = (data & FSTINT) ? 1 : 0;
    Status->_DCLOFFINT = (data & DCLOFFINT) ? 1 : 0;
    Status->_LDOFF_PH = (data & LDOFF_PH) ? 1 : 0;
    Status->_LDOFF_PL = (data & LDOFF_PL) ? 1 : 0;
    Status->_LDOFF_NH = (data & LDOFF_NH) ? 1 : 0;
    Status->_LDOFF_NL = (data & LDOFF_NL) ? 1 : 0;
    // return true;
}
#pragma endregion

#pragma endregion MAX30003_Status

void IRAM_ATTR max30003_isr_handler(void *arg)
{

    // if (gpio_get_level((gpio_num_t)INT) == 0)
    // {
    // Trigger_Event = true;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(ECG_Electriocal_diagram, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken); // chuyển đổi ngữ cảnh tránh bị lỗi
    // } else Trigger_Event = false;
}
void ECG_Handle_t(void *pvParameters)
{
    // esp_task_wdt_delete(NULL); // xóa Watchdog
    static int i = 0;
    static int32_t raw = 0;
    volatile bool Flag_Chest = 0;
    static uint32_t HR_Avg = 0;
    static uint32_t RR_Avg = 0;
    for (;;)
    {
        if (xSemaphoreTake(ECG_Electriocal_diagram, portMAX_DELAY) == pdTRUE)
        // if(Trigger_Event == true)
        {
            gpio_intr_disable((gpio_num_t)INT); // Ngắt tạm thời
            MAX300003_STATUS_Sample(&ECG_Status);
            if (ECG_Status._EOVF)
            {
                printf("FIFO Over flow");
                max30003RegWrite(FIFO_RST, 0x0); /// resset FIFO
            }
            else
            {
                raw = MAX30003_getEcgSamples(); // valid sample
            }
            if (ECG_Status._DCLOFFINT)
                Flag_Chest = 1;
            Max30003_Info sensor = MAX30003_getHRandRR();
            HR_Avg += sensor.heartRate;
            RR_Avg += sensor.RR;
            static uint8_t cnt = 0;
            cnt++;
            if (cnt >= 50)
            {
                FIFO.HR = (int16_t)(HR_Avg / 50.0f); // nhip tim
                FIFO.RR = (int16_t)(RR_Avg / 50.0f);        // khoang cach giua 2 dinh song tinh ms
                cnt = 0;
                HR_Avg = 0 ;
                RR_Avg = 0 ;
            }
            if (Flag_Chest)
            {
                Flag_Chest = 0;
                raw = -2553;
                FIFO.HR = 0;
                FIFO.RR = 0;

                // ESP_LOGE(Tag_ECG, "Warning Chest Lead off\n");
            }
            FIFO.EcgWave[i] = raw;
            i++;
            // printf("%ld \n", FIFO.EcgWave[i]);
            if (i > MaximumSample_ECG - 1)
            {
                i = 0;
                // gửi dữ liệu chỉ khi pin đủ
                if (FIFO.Bat >= 1)
                {
                    esp_err_t res = esp_now_send(peerInfo.peer_addr, (uint8_t *)&FIFO, sizeof(FIFO));
                    if (res != ESP_OK)
                    {
                        ESP_LOGE(Tag_ECG, "esp_now_send error: %d", res);
                    }
                    xQueueSend(ble_queue, &FIFO, 0);
                }
                else
                {
                    ESP_LOGW(Tag_ECG, "Battery low, skip sending packet");
                }
            }
            // Trigger_Event = false;
            gpio_intr_enable((gpio_num_t)INT); // bat ngat
        }
    }
}
void Reading_BAT(void *pvParameters)
{
    // esp_adc_cal_characteristics_t adc_chars;
    // esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12,
    // 1100, &adc_chars);

    for (;;)
    {

        int adcValue = 0;
        for (int i = 0; i < NumberSample_Bat; i++)
        {
            adcValue += adc1_get_raw((gpio_num_t)PIN_BATTERY);
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }

        float voltage = ((float)adcValue / (float)(NumberSample_Bat)) * (3.2f / 4095.0f); // 3v3 đầu vào
        // // Tính VBAT dựa trên mạch chia áp
        float vbat = voltage * ((R1 + R2) / R2);
        percBat = (uint8_t)((vbat * 1000 - Bat_min_volt) * 100.0 / (Bat_max_volt - Bat_min_volt));
        percBat = constrain(percBat, 0, 100);
        FIFO.Bat = percBat;
        // #ifdef debug
        // printf("Voltage ADC: %.3f V\n", voltage);
        // printf("VBAT: %.3f V\n", vbat);
        // printf("Phần trăm pin: %d \n", percBat);
        // #endif
        // }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
void Acceleration_Handle(void *pvParameters)
{
    const float THRESH_Falling = 2.0; // 4g threshold
    float x, y, z;
    for (;;)
    {
        if (ADXL345_Update())
        {
            // printf("RAW: X=%d Y=%d Z=%d | g: %.3f %.3f %.3f\n",
            //        ADXL345_RawX(), ADXL345_RawY(), ADXL345_RawZ(),
            //        ADXL345_GetX(), ADXL345_GetY(), ADXL345_GetZ());

            x = ADXL345_GetX();
            y = ADXL345_GetY();
            z = ADXL345_GetZ();

            mag = sqrtf(x * x + y * y + z * z);

            FIFO.Accel = (uint16_t)(mag * 1000); // vector gia toc te nga
            if (mag > THRESH_Falling)
            {
                printf(">>> Falling DETECTED: |a| = %.2f g\n", mag);
                // gpio_set_level((gpio_num_t)led_pin_state,0);
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
void BLE_Protocol(void *pvParameters)
{
    ECGFIFO _BLE_FIFO;
    for (;;)
    {
        if (xQueueReceive(ble_queue, &_BLE_FIFO, portMAX_DELAY) == pdTRUE)
        {
            if (heart_rate_chr_conn_handle_inited)
            {
                send_ecg_ble_notification(&_BLE_FIFO);
            }
        }
    }
}
static void nimble_host_task(void *param)
{
    /* Task entry log */
    ESP_LOGI(TAG, "nimble host task has been started!");

    /* This function won't return until nimble_port_stop() is executed */
    nimble_port_run();

    /* Clean up at exit */
    vTaskDelete(NULL);
}
void app_main(void)
{
    printf("Date Updating : 11/8/2025 17:28:30 pm\n");
    printf("IoTVision ECG version 2 Design by Trung Thao\n");
    printf("Ta Thuan & Luong Bui Duy Duy Author's Firmware\n");
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    _wifi_init();
    esp32_now_init();
    Init_Bluetooth_GATT_Sever();
    static SPI_config _config = {
        .CS_pin = CS_PIN_MAX30003,
        .MISO_pin = SO,
        .MOSI_pin = SI,
        .SCK_pin = SCK,
        .INT_pin = INT // ngắt nếu không sài ngắt thì -1
    };

    if (Max30003_init(&MAX30003_handle, &_config) == ESP_OK)
    {
        printf("MAX30003 init success!\n");
    }
    max30003ReadInfo();
    max30003Begin();
    ECG_Electriocal_diagram = xSemaphoreCreateBinary();
    if (ECG_Electriocal_diagram == NULL)
    {
        printf("SEM CREATE FAIL\n");
        while (1)
            ;
    }
    max30003_attach_interrupt(max30003_isr_handler); // sử dung ngắt
    vTaskDelay(pdMS_TO_TICKS(100));
    Bat_Config();
    Led_State_init();

    I2C_Config Info = {
        .__SCL = SCL,
        .__SDA = SDA};
    ADXL345_init(Info);
    ADXL345writeRate(ADXL345_RATE_200HZ);
    ADXL345writeRange(ADXL345_RANGE_16G);
    ADXL_ReadDevice();
    ADXL345_START();
    printf("ADXL345 device id : 0x%2X\n", ADXL_ReadDevice());
    ADXL345_Update();
    ble_queue = xQueueCreate(10, sizeof(ECGFIFO));
    xTaskCreate(ECG_Handle_t,
                "Sample Raw ECG Device",
                10000,
                NULL,
                4,
                NULL);
    xTaskCreate(Reading_BAT,
                "Reading Bat",
                4048,
                NULL,
                2,
                NULL);
    xTaskCreate(Acceleration_Handle,
                "Sample Accel",
                4048,
                NULL,
                3,
                NULL);
    xTaskCreate(nimble_host_task,
                "NimBLE Host",
                4096,
                NULL,
                3,
                NULL);
    xTaskCreate(BLE_Protocol,
                "BLE Handle",
                4096,
                NULL,
                3,
                NULL);
    // printf("Woke up!\n");
    // while (1)
    // {
    // vTaskDelay(pdMS_TO_TICKS(500));
    //     // static int i = 0;

    //     // int32_t raw = MAX30003_getEcgSamples();
    //     // Max30003_Info sensor = MAX30003_getHRandRR();
    //     // FIFO.HR = sensor.heartRate;

    //     // FIFO.Accel = 0;
    //     // FIFO.RR = 0;
    //     // FIFO.EcgWave[i] = raw;

    //     // printf("%ld  %d \n", FIFO.EcgWave[i] ,MAX300003_STATUS_Sample());
    //     // i++;
    //     // if (i > MaximumSample_ECG -1)
    //     // {
    //     //     i = 0;
    //     //     esp_now_send(peerInfo.peer_addr, (uint8_t *)&FIFO, sizeof(FIFO));
    //     // }
    //     // vTaskDelay(23 / portTICK_PERIOD_MS);
    // }
}

/* Defines */
static void start_advertising(void)
{
    const char *name;
    int rc = 0;
    struct ble_hs_adv_fields adv_fields = {0};
    struct ble_hs_adv_fields rsp_fields = {0};
    struct ble_gap_adv_params adv_params = {0};

    /* Set advertising flags */
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    /* Set device name */
    name = ble_svc_gap_device_name();
    adv_fields.name = (uint8_t *)name;
    adv_fields.name_len = strlen(name);
    adv_fields.name_is_complete = 1;

    /* Set device tx power */
    adv_fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    adv_fields.tx_pwr_lvl_is_present = 1;

    /* Set device appearance */
    adv_fields.appearance = BLE_GAP_APPEARANCE_GENERIC_TAG;
    adv_fields.appearance_is_present = 1;

    /* Set device LE role */
    adv_fields.le_role = BLE_GAP_LE_ROLE_PERIPHERAL;
    adv_fields.le_role_is_present = 1;

    /* Set advertiement fields */
    rc = ble_gap_adv_set_fields(&adv_fields);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "failed to set advertising data, error code: %d", rc);
        return;
    }

    /* Set device address */
    rsp_fields.device_addr = addr_val;
    rsp_fields.device_addr_type = own_addr_type;
    rsp_fields.device_addr_is_present = 1;

    /* Set URI */
    rsp_fields.uri = esp_uri;
    rsp_fields.uri_len = sizeof(esp_uri);

    /* Set advertising interval */
    rsp_fields.adv_itvl = BLE_GAP_ADV_ITVL_MS(500);
    rsp_fields.adv_itvl_is_present = 1;

    /* Set scan response fields */
    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "failed to set scan response data, error code: %d", rc);
        return;
    }

    /* Set non-connetable and general discoverable mode to be a beacon */
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    /* Set advertising interval */
    adv_params.itvl_min = BLE_GAP_ADV_ITVL_MS(500);
    adv_params.itvl_max = BLE_GAP_ADV_ITVL_MS(800);

    /* Start advertising */
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params,
                           gap_event_handler, NULL);
    /*
     *   int ble_gap_adv_start(uint8_t own_addr_type, const ble_addr_t *direct_addr, int32_t duration_ms, const struct ble_gap_adv_params *adv_params, ble_gap_event_fn *cb, void *cb_arg)
     *    Start advertising
     *
     *    This function configures and start advertising procedure.
     *
     *    Parameters:
     *    own_addr_type – The type of address the stack should use for itself. Valid values are: - BLE_OWN_ADDR_PUBLIC - BLE_OWN_ADDR_RANDOM - BLE_OWN_ADDR_RPA_PUBLIC_DEFAULT - BLE_OWN_ADDR_RPA_RANDOM_DEFAULT
     *    direct_addr – The peer's address for directed advertising. This parameter shall be non-NULL if directed advertising is being used.
     *    duration_ms – The duration of the advertisement procedure. On expiration, the procedure ends and a BLE_GAP_EVENT_ADV_COMPLETE event is reported. Units are milliseconds. Specify BLE_HS_FOREVER for no expiration.
     *    adv_params – Additional arguments specifying the particulars of the advertising procedure.
     *    cb – The callback to associate with this advertising procedure. If advertising ends, the event is reported through this callback. If advertising results in a connection, the connection inherits this callback as its event-reporting mechanism.
     *    cb_arg – The optional argument to pass to the callback function.
     *
     *    Returns:
     *    0 on success, error code on failure.
     *
     *    ble_gap_adv_start
     */
    if (rc != 0)
    {
        ESP_LOGE(TAG, "failed to start advertising, error code: %d", rc);
        return;
    }
    ESP_LOGI(TAG, "advertising started!");
}
/* Public functions */
void adv_init(void)
{
    /* Local variables */
    int rc = 0;
    char addr_str[18] = {0};

    /* Make sure we have proper BT identity address set (random preferred) */
    rc = ble_hs_util_ensure_addr(0);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "device does not have any available bt address!");
        return;
    }

    /* Figure out BT address to use while advertising (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "failed to infer address type, error code: %d", rc);
        return;
    }

    /* Printing ADDR */
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL); //
    if (rc != 0)
    {
        ESP_LOGE(TAG, "failed to copy device address, error code: %d", rc);
        return;
    }
    /* Notice:
     *Parameters:
     *-id_addr_type – The type of identity address to retrieve. Valid values are: o BLE_ADDR_PUBLIC o BLE_ADDR_RANDOM
     *-out_id_addr – On success, the requested identity address is copied into this buffer. The buffer must be at least six bytes in size. Pass NULL if you do not require this information.
     *- out_is_nrpa – On success, the pointed-to value indicates whether the retrieved address is a non-resolvable private address. Pass NULL if you do not require this information.
     *- Returns:
     *0 on success; BLE_HS_EINVAL if an invalid address type was specified; BLE_HS_ENOADDR if the device does not have an identity address of the requested type; Other BLE host core code on error.
     */
    sprintf(addr_str, "%02X:%02X:%02X:%02X:%02X:%02X", addr_val[0], addr_val[1], addr_val[2], addr_val[3], addr_val[4], addr_val[5]);
    ESP_LOGI(TAG, "device address: %s", addr_str);

    /* Start advertising. */
    start_advertising();
}

/*
 *  GATT server subscribe event callback
 *      1. Update heart rate subscription status
 */

void gatt_svr_subscribe_cb(struct ble_gap_event *event)
{
    /* Check connection handle */
    if (event->subscribe.conn_handle != BLE_HS_CONN_HANDLE_NONE)
    {
        ESP_LOGI(TAG, "subscribe event; conn_handle=%d attr_handle=%d",
                 event->subscribe.conn_handle, event->subscribe.attr_handle);
    }
    else
    {
        ESP_LOGI(TAG, "subscribe by nimble stack; attr_handle=%d",
                 event->subscribe.attr_handle);
    }

    /* Check attribute handle */
    if (event->subscribe.attr_handle == heart_rate_chr_val_handle)
    {
        /* Update heart rate subscription status */
        heart_rate_chr_conn_handle = event->subscribe.conn_handle;
        heart_rate_chr_conn_handle_inited = true;
        heart_rate_ind_status = event->subscribe.cur_indicate;
    }
}
/*
 * NimBLE applies an event-driven model to keep GAP service going
 * gap_event_handler is a callback function registered when calling
 * ble_gap_adv_start API and called when a GAP event arrives
 */

static int gap_event_handler(struct ble_gap_event *event, void *arg)
{
    /* Local variables */
    int rc = 0;
    struct ble_gap_conn_desc desc;

    /* Handle different GAP event */
    switch (event->type)
    {

    /* Connect event */
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        ESP_LOGI(TAG, "connection %s; status=%d",
                 event->connect.status == 0 ? "established" : "failed",
                 event->connect.status);

        /* Connection succeeded */
        if (event->connect.status == 0)
        {
            /* Check connection handle */
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            if (rc != 0)
            {
                ESP_LOGE(TAG,
                         "failed to find connection by handle, error code: %d",
                         rc);
                return rc;
            }

            /* Print connection descriptor */
            // print_conn_desc(&desc);

            /* Try to update connection parameters */
            struct ble_gap_upd_params params = {.itvl_min = desc.conn_itvl,
                                                .itvl_max = desc.conn_itvl,
                                                .latency = 3,
                                                .supervision_timeout =
                                                    desc.supervision_timeout};
            rc = ble_gap_update_params(event->connect.conn_handle, &params);
            if (rc != 0)
            {
                ESP_LOGE(
                    TAG,
                    "failed to update connection parameters, error code: %d",
                    rc);
                return rc;
            }
        }
        /* Connection failed, restart advertising */
        else
        {
            start_advertising();
        }
        return rc;

    /* Disconnect event */
    case BLE_GAP_EVENT_DISCONNECT:
        /* A connection was terminated, print connection descriptor */
        ESP_LOGI(TAG, "disconnected from peer; reason=%d",
                 event->disconnect.reason);
        heart_rate_chr_conn_handle = 0;
        heart_rate_chr_conn_handle_inited = false;
        heart_rate_ind_status = false;
        /* Restart advertising */
        start_advertising();
        return rc;

    /* Connection parameters update event */
    case BLE_GAP_EVENT_CONN_UPDATE:
        /* The central has updated the connection parameters. */
        ESP_LOGI(TAG, "connection updated; status=%d",
                 event->conn_update.status);

        /* Print connection descriptor */
        rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
        if (rc != 0)
        {
            ESP_LOGE(TAG, "failed to find connection by handle, error code: %d",
                     rc);
            return rc;
        }
        // print_conn_desc(&desc);
        return rc;

    /* Advertising complete event */
    case BLE_GAP_EVENT_ADV_COMPLETE:
        /* Advertising completed, restart advertising */
        ESP_LOGI(TAG, "advertise complete; reason=%d",
                 event->adv_complete.reason);
        // start_advertising();
        return rc;

    /* Notification sent event */
    case BLE_GAP_EVENT_NOTIFY_TX:
        if ((event->notify_tx.status != 0) &&
            (event->notify_tx.status != BLE_HS_EDONE))
        {
            /* Print notification info on error */
            ESP_LOGI(TAG,
                     "notify event; conn_handle=%d attr_handle=%d "
                     "status=%d is_indication=%d",
                     event->notify_tx.conn_handle, event->notify_tx.attr_handle,
                     event->notify_tx.status, event->notify_tx.indication);
        }
        return rc;

    /* Subscribe event */
    case BLE_GAP_EVENT_SUBSCRIBE:
        /* Print subscription info to log */
        ESP_LOGI(TAG,
                 "subscribe event; conn_handle=%d attr_handle=%d "
                 "reason=%d prevn=%d curn=%d previ=%d curi=%d",
                 event->subscribe.conn_handle, event->subscribe.attr_handle,
                 event->subscribe.reason, event->subscribe.prev_notify,
                 event->subscribe.cur_notify, event->subscribe.prev_indicate,
                 event->subscribe.cur_indicate);

        /* GATT subscribe event callback */
        gatt_svr_subscribe_cb(event);
        return rc;

    /* MTU update event */
    case BLE_GAP_EVENT_MTU:
        /* Print MTU update info to log */
        ESP_LOGI(TAG, "mtu update event; conn_handle=%d cid=%d mtu=%d",
                 event->mtu.conn_handle, event->mtu.channel_id,
                 event->mtu.value);
        return rc;
    }

    return rc;
}
void send_ecg_ble_notification(ECGFIFO *_FIFO)
{
    if (!heart_rate_chr_conn_handle_inited || heart_rate_chr_conn_handle == 0)
    {
        return;
    }

    uint8_t packet[2 + 2 + 1 + 2 + (MaximumSample_ECG * 4)];
    int offset = 0;

    // 1. Heart Rate (2 bytes, little-endian)
    packet[offset++] = _FIFO->HR & 0xFF;
    packet[offset++] = (_FIFO->HR >> 8) & 0xFF;

    // 2. RR Interval (2 bytes, little-endian)
    packet[offset++] = _FIFO->RR & 0xFF;
    packet[offset++] = (_FIFO->RR >> 8) & 0xFF;

    // 3. Battery Level (1 byte)
    packet[offset++] = _FIFO->Bat;

    // 4. Accelerometer (2 bytes, little-endian)
    packet[offset++] = _FIFO->Accel & 0xFF;
    packet[offset++] = (_FIFO->Accel >> 8) & 0xFF;

    for (int i = 0; i < MaximumSample_ECG; i++)
    {
        packet[offset++] = (_FIFO->EcgWave[i] >> 0) & 0xFF;
        packet[offset++] = (_FIFO->EcgWave[i] >> 8) & 0xFF;
        packet[offset++] = (_FIFO->EcgWave[i] >> 16) & 0xFF;
        packet[offset++] = (_FIFO->EcgWave[i] >> 24) & 0xFF;
    }
    // Gửi notification
    struct os_mbuf *om = ble_hs_mbuf_from_flat(packet, offset);
    if (om == NULL)
    {
        ESP_LOGE(TAG, "Failed to create mbuf");
        return;
    }
    /*int ble_gatts_notify_custom(uint16_t conn_handle, uint16_t att_handle, struct os_mbuf *om)
    *Sends a "free-form" characteristic notification. This function consumes the supplied mbuf regardless of the outcome.
    *
    *Parameters:
    *conn_handle – The connection over which to execute the procedure.
    *att_handle – The attribute handle to indicate in the outgoing notification.
    *om – The value to write to the characteristic.

    *Returns:
    *0 on success; nonzero on failure.
    */
    int rc = ble_gatts_notify_custom(heart_rate_chr_conn_handle,
                                     heart_rate_chr_val_handle,
                                     om);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to send notification: %d", rc);
        os_mbuf_free_chain(om);
    }
    else
    {
        ESP_LOGI(TAG, "Sent ECG data via BLE: HR=%d, RR=%d, Bat=%d%%\n", FIFO.HR, FIFO.RR, FIFO.Bat);
    }
}