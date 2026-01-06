#include "Max30003.h"

static const char TAG[] = "MAX30003";

#pragma region MAX30003
struct MAX30003_context_t
{
    SPI_config cfg;                 ///< Configuration by the caller.
    spi_device_handle_t spi_handle; ///< SPI device handle
    // SemaphoreHandle_t INTB2Bsem;    ///< Semaphore for INTB and INT2B ISR
    ETAG ETAG;
    
};
static MAX30003_handle_t __MAX30003_handle_t = NULL;
esp_err_t Max30003_init(MAX30003_handle_t *out_handle, SPI_config *config_pin)
{
    esp_err_t err = ESP_OK;
    *out_handle = malloc(sizeof(struct MAX30003_context_t));
    if (*out_handle == NULL)
    {
        ESP_LOGE(TAG, "Khong cap phat duoc bo nho cho MAX30003");
        return ESP_ERR_NO_MEM;
    }
    __MAX30003_handle_t = (*out_handle);
    // Sao chép cấu hình
    memcpy(&(*out_handle)->cfg, config_pin, sizeof(SPI_config));
    SPIsetting _cfg = {
        ._clock = MAX30003_SPI_SPEED,
        ._bitOrder = SPI_MSBFIRST,
        ._dataMode = SPI_MODE0};
    SPI_init_bus(&(*out_handle)->cfg, 32);

    err = SPI_add_device((*out_handle)->cfg.CS_pin, &_cfg, &(*out_handle)->spi_handle);
    if (err != ESP_OK)
    {
        if ((*out_handle)->spi_handle)
        {
            spi_bus_remove_device((*out_handle)->spi_handle);
            (*out_handle)->spi_handle = NULL;
        }
        free(*out_handle); // giải phóng bộ nhớ
    }
    return err;
}
#pragma region MAX30003_read
esp_err_t max30003RegRead(uint8_t Reg_address, uint8_t *buff)
{
    esp_err_t err;
    uint8_t spiTxBuff[4] = {(Reg_address << 1) | RREG, 0, 0, 0};
    spi_transaction_t trans = {
        .length = 8 * 4, // 4 byte = 32 bit (1 byte lệnh + 3 byte data)
        .tx_buffer = spiTxBuff,
        .rx_buffer = buff,
    };
    // vTaskDelay(1/portTICK_PERIOD_MS);
    taskYIELD(); // nhuong CPU
    // spi_transaction_t *pTrans = &trans;
    // spi_device_queue_trans(ctx->spi, &trans, portMAX_DELAY);
    // esp_err_t err = spi_device_get_trans_result(ctx->spi, &pTrans, portMAX_DELAY);
    // err = spi_device_acquire_bus(__MAX30003_handle_t->spi_handle,portMAX_DELAY);  
    //chiem quyen dieu khien SPI BUS cho thiet bi khi tryen SPI  
    // if(err != ESP_OK) {
    //     ESP_LOGI(TAG,"Loi Device Acquire %d",err);
    //     return err;
    // }
    gpio_set_level(__MAX30003_handle_t->cfg.CS_pin, 0);
    err = spi_device_transmit(__MAX30003_handle_t->spi_handle, &trans);
    // err = spi_device_polling_transmit(__MAX30003_handle_t->spi_handle, &trans);
    gpio_set_level(__MAX30003_handle_t->cfg.CS_pin, 1);
    // spi_device_release_bus(__MAX30003_handle_t->spi_handle); // giải phong Bus de lan sau su dung truyen SPI

    return err;
}
#pragma endregion MAX30003_read
#pragma region MAX30003_write
esp_err_t max30003RegWrite(uint8_t WRITE_ADDRESS, uint32_t data)
{

    uint8_t TxBuff[4];
    // Byte 0: Command (ghi thanh ghi)
    TxBuff[0] = (WRITE_ADDRESS << 1) | WREG;
    // 3 byte data (24 bit)
    TxBuff[1] = (data >> 16) & 0xFF;
    TxBuff[2] = (data >> 8) & 0xFF;
    TxBuff[3] = data & 0xFF;

    spi_transaction_t trans = {
        .length = 32, // 4 byte = 32 bits
        .tx_buffer = TxBuff,
    };

    gpio_set_level(__MAX30003_handle_t->cfg.CS_pin, 0);
    esp_err_t err = spi_device_transmit(__MAX30003_handle_t->spi_handle, &trans);
    gpio_set_level(__MAX30003_handle_t->cfg.CS_pin, 1);
    return err;
}
#pragma endregion MAX30003_write
bool max30003ReadInfo(void)
{
    uint8_t readBuff[4];

    max30003RegRead(INFO, readBuff);
    // printf("INFO Reg Raw: %02X %02X %02X %02X\n",
    //    readBuff[0], readBuff[1], readBuff[2], readBuff[3]);
    if ((readBuff[1] & 0xf0) == 0x50)
    { // chỉ nhận tín hiệu mức cao trả giá trị nhận vào 0101rev[19,16]

        printf("max30003 is ready");
        printf("Rev ID : %d\n", (readBuff[1] & 0xF0));
        return true;
    }
    else
    {

        printf("max30003 read info error\n");
        return false;
    }
}
void max30003Begin(void)
{
  max30003SwReset(); // Khởi động lại thiết bị
  vTaskDelay(10 / portTICK_PERIOD_MS);
  CNFG_GEN_ GEN = {
    .ULP_LON   = Dis_ULP_Lead_On_Detection,
    .FMSTR     = GEN_FMSTR_32768_512Hz,
    ._EN_ECG   = EN_ECG,
    .DCLOFF    = EN_DCLOFF,
    .DCIPOL    = DCLOFF_IPOL,
    .DCIMAG    = DCLOFF_IMAG_100nA,
    .DCVTH     = DCLOFF_VTH_400mV,
    .EN_RBIAS_ = EN_RBIAS,
    .RBIASV    = EN_RBIASV_50M_Ohm,
    .RBIASP    = EN_RBIASP,
    .RBIASN    = EN_BIASN,
    .REG       = 0, // clear thanh ghi tránh dữ liệu rác vào
  };
  GEN.REG = GEN.ULP_LON | GEN.FMSTR | GEN._EN_ECG | GEN._EN_ECG | GEN.DCLOFF | GEN.DCIPOL | GEN.DCIMAG | GEN.DCVTH | GEN.EN_RBIAS_ | GEN.RBIASV | GEN.RBIASP | GEN.RBIASN;
  ESP_LOGI(TAG,"CNFG GEN : %08lX ",GEN.REG);
  max30003RegWrite(CNFG_GEN, GEN.REG); 
//   vTaskDelay(10/ portTICK_PERIOD_MS);
//   max30003RegWrite(MNGR_INT,0x780010);  
//   vTaskDelay(20 / portTICK_PERIOD_MS);
//   max30003RegWrite(MNGR_DYN, 0xFC000);  
//   vTaskDelay(20 / portTICK_PERIOD_MS);
  // Cấu hình hiệu chuẩn (Calibration) CNFG_CAL
  // Cài đặt tín hiệu chuẩn để kiểm tra đường tín hiệu
//   max30003RegWrite(CNFG_CAL, 0x720000);  //   Calibration Source Magnitude Selection (VMAG)  = 0.50mV  || Calibration Source Frequency 256Hz
  // Cấu hình MUX đầu vào CNFG_EMUX 0x304800 0x700800 600800
  // Kết nối đường tín hiệu ECG vào ngõ vào MUX
  CNFG_EMUX_ EMUX = {
    .POL      = Pol_Inverted,
    .OPENP    = EN_ECGP_CN_AFE,
    .OPENN    = EN_ECGN_CN_AFE,
    .CALP_SEL = In_VCALP_ECGP,
    .CALN_SEL = In_VCALN_ECGN, 
    .REG      = 0 ,
  };
  EMUX.REG = EMUX.POL | EMUX.OPENP | EMUX.OPENN | EMUX.CALN_SEL | EMUX.CALP_SEL; 
  ESP_LOGI(TAG,"CNFG EMUX : %08lX ",EMUX.REG);
  vTaskDelay(10 / portTICK_PERIOD_MS);
  max30003RegWrite(CNFG_EMUX, EMUX.REG);  // Chọn ngõ vào ECGP và ECGN 0x0B0000
//   vTaskDelay(20 / portTICK_PERIOD_MS);

  // Cấu hình ECG CNFG_ECG
  // Cài đặt tốc độ lấy mẫu và các bộ lọc để tối ưu hóa chất lượng tín hiệu 0x805000 , 837000 512 Hz, GAIN=160, DLPF=150 Hz , 0x836002 512 Hz, GAIN=160, DLPF = 40 Hz
  max30003RegWrite(CNFG_ECG, 0x835000);  //
  vTaskDelay(10 / portTICK_PERIOD_MS);
  // 512Hz, Gain=40, DLPF=40Hz, DHPF=0.5Hz //0x815000
  // Cấu hình CNFG_RTOR1
  max30003RegWrite(CNFG_RTOR1, 0x3FA300);  //0x5FC600 0x3fc600 0x3FA300
  vTaskDelay(10 / portTICK_PERIOD_MS);
  max30003RegWrite(EN_INT, 0x800403); //
  vTaskDelay(10 / portTICK_PERIOD_MS);
  max30003Synch();
}
void max30003SwReset(void)
{
    max30003RegWrite(SW_RST, 0x000000);
}

void max30003Synch(void)
{
    max30003RegWrite(SYNCH, 0x000000);
}
int32_t MAX30003_getEcgSamples(void)
{
    uint8_t regReadBuff[4];
    esp_err_t err = max30003RegRead(ECG_FIFO, regReadBuff);
    if(err != ESP_OK) {
        ESP_LOGI(TAG,"Loi ECG_Sample %d",err);
    }
    unsigned long data0 = (unsigned long)(regReadBuff[1]);
    data0 = data0 << 16;
    unsigned long data1 = (unsigned long)(regReadBuff[2]);
    data1 = data1 << 8;
    unsigned long data2 = (unsigned long)(regReadBuff[3]);
    // data2 = data2 >> 6;
    // data2 = data2 & 0x03;
    int32_t data = (int32_t)(data0 | data1 | data2);
    if (data & 0x800000) data |= 0xFF000000;

    // ESP_LOGI(TAG,"RAW :  %02X , %02X ECG RaW : %ld",regReadBuff[2],regReadBuff[3], data);
    // printf("RAW %d %ld\n",(regReadBuff[3] >> 3) & 0x07, data);
    return data;
}

Max30003_Info MAX30003_getHRandRR(void)
{
    Max30003_Info info;
    esp_err_t err;
    uint8_t regReadBuff[4];
    err = max30003RegRead(RTOR, regReadBuff);
    if(err != ESP_OK) {
        ESP_LOGI(TAG,"Loi HandleRR %d",err);
    }
    unsigned long RTOR_msb = (unsigned long)(regReadBuff[1]);
    unsigned char RTOR_lsb = (unsigned char)(regReadBuff[2]);
    unsigned long rtor = (RTOR_msb << 8 | RTOR_lsb);
    rtor = ((rtor >> 2) & 0x3fff);

    info.heartRate = (unsigned int)(60 / ((float)rtor * 0.0078125));

    info.RR = (unsigned int)rtor * (7.8125); // 8ms
    return info;
    // #ifndef DEBUG
    //  Serial.println(heartRate);
    // #endif
}
// Hàm attach interrupt cho MAX30003 với semaphore
void max30003_attach_interrupt(void (*isr_handler)(void*))
{
    // Tạo binary semaphore
    // __MAX30003_handle_t->INTB2Bsem = xSemaphoreCreateBinary();

    // Cấu hình chân INTB
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE, // INTB  cấu hình chân ở  trạng thái thay đổi
        .pin_bit_mask = (1ULL << __MAX30003_handle_t->cfg.INT_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);
    // Cài đặt ISR
    gpio_install_isr_service(0);
    gpio_isr_handler_add(__MAX30003_handle_t->cfg.INT_pin, isr_handler, NULL);
}
#pragma endregion MAX30003

#pragma region SPICLASS
    esp_err_t SPI_init_bus(SPI_config *config_pin, uint8_t max_transfer)
{

    esp_err_t ret;
    ESP_LOGI(TAG, "Initializing bus SPI...");
    spi_bus_config_t buscfg = {
        .miso_io_num = config_pin->MISO_pin,
        .mosi_io_num = config_pin->MOSI_pin,
        .sclk_io_num = config_pin->SCK_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = max_transfer,
    };
    // Initialize the SPI bus
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    return ret;
}
esp_err_t SPI_add_device(gpio_num_t _CS, SPIsetting *cfg, spi_device_handle_t *spi_handle)
{
    esp_err_t ret;
    ESP_LOGI(TAG, "Adding SPI device...");

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = cfg->_clock,
        .mode = cfg->_dataMode, // SPI_MODE0..3
        .command_bits = 0,
        .spics_io_num = -1,
        // .pre_cb = cs_low, // trước khi bắt đầu truyền dữ liệu, CS ở mức thấp
        // .post_cb = cs_high, // sau khi truyền dữ liệu, CS đưa lên mức cao
        .queue_size = 1,
        .input_delay_ns = 500,
        .flags = SPI_DEVICE_POSITIVE_CS,
    };

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, spi_handle);
    gpio_config_t cs_cfg = {
        .pin_bit_mask = (1ULL << _CS),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&cs_cfg);
    gpio_set_level(_CS, 1);
    return ret;
}

#pragma endregion SPICLASS