#include "ADXL345.h"

static const char TAG[] = "ADXL345";

#define OFSX 0x1E
#define OFSY 0x1F
#define OFSZ 0x20

#define DEVID 0x00 // read only

#define THRESH_TAP 0x1D
#define DUR 0x21
#define LATENT 0x22
#define WINDOW 0x23
#define THRESH_ACT 0x24
#define THRESH_INACT 0x25
#define TIME_INACT 0x26
#define ACT_INACT_CONTROLL 0x27
#define THRESH_FF 0x28
#define TIME_FF 0x29
#define TAP_AXES 0x2A
#define ACT_TAP_STATUS 0x2B
#define BW_RATE 0x2C
#define POWER_CTL 0x2D
#define INT_ENABLE 0x2E
#define INT_MAP 0x2F
#define DATA_FORMAT 0x31

#define FIFO_CTTL 0x38
#define FIFO_STATUS 0x39
/**
 * @brief : Tạ thuận
 * @Note :
 * ở dưới là thanh ghi để đọc
 */
#define DATAX0 0x32 //
#define DATAX1 0x33 //
#define DATAY0 0x34
#define DATAY1 0x35
#define DATAZ0 0x36
#define DATAZ1 0x37

#define kRatio2g (float)(2 * 2) / 1024.0f;
#define kRatio4g (float)(4 * 2) / 1024.0f;
#define kRatio8g (float)(8 * 2) / 1024.0f;
#define kRatio16g (float)(16 * 2) / 1024.0f;

// #define kRatio2g    0.00390625f   // 2/512
// #define kRatio4g    0.0078125f    // 4/512
// #define kRatio8g    0.015625f     // 8/512
// #define kRatio16g   0.03125f      // 16/512

static ADXL345_Handle *ADXL345_Handle_t = NULL;
static void DataFormatBits_init(void);
static void BwRateBits_init(void);
esp_err_t WriteRegister(i2c_master_dev_handle_t i2c_dev, uint8_t ResgisterAddress, uint8_t cmd)
{
	esp_err_t err;
	uint8_t Data[2] = {ResgisterAddress, cmd};
	err = i2c_master_transmit(i2c_dev, Data, sizeof(Data), -1);
	return err;
}
esp_err_t ReadRegister(i2c_master_dev_handle_t i2c_dev, uint8_t ResgisterAddress, uint8_t *buff, uint8_t numberbyte)
{
	// uint8_t buf[20] = {ResgisterAddress};
	esp_err_t err = ESP_OK;
	err = i2c_master_transmit_receive(i2c_dev, &ResgisterAddress, 1, buff, numberbyte, portMAX_DELAY);
	// i2c_master_transmit(i2c_dev, &ResgisterAddress, sizeof(ResgisterAddress), -1);
	// i2c_master_receive(i2c_dev,_DATA,numberByte,-1);
	return err;
}
void ADXL345_init(I2C_Config _I2C_Config)
{
	ADXL345_Handle *Out = malloc(sizeof(ADXL345_Handle));
	(*Out)._cfg = _I2C_Config;
	ADXL345_Handle_t = Out;
	i2c_master_bus_config_t i2c_mst_config = {
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.scl_io_num = _I2C_Config.__SCL,
		.sda_io_num = _I2C_Config.__SDA,
		.glitch_ignore_cnt = 7, // che do loc nhieu
		.flags = {
			.enable_internal_pullup = true}};
	i2c_master_bus_handle_t bus_handle;

	ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = ADXL345_ADDR,
		.scl_speed_hz = Default_I2C_Clock,
	};
	// vTaskDelay(pdTICKS_TO_MS(5));
	if (i2c_master_bus_add_device(bus_handle, &dev_cfg, &Out->dev_handle) != ESP_OK)
	{
		// Nếu add device thất bại, cleanup
		i2c_del_master_bus(bus_handle);
		free(Out);
		return;
	}
	ESP_LOGI(TAG,"Khoi tao I2C  thanh cong ");
	BwRateBits_init();
	DataFormatBits_init();


	// WriteRegister((*Out).dev_handle, POWER_CTL, 0);	   // đưa thanh ghi vào chế độ chờ
	// WriteRegister((*Out).dev_handle, POWER_CTL, 0x08); //  đưa Measure vào chế độ đo // 8 Hz

}

esp_err_t ADXL345_START(void)
{
	esp_err_t err = ESP_OK;
	uint8_t reg = 0x08;
	err = WriteRegister(ADXL345_Handle_t->dev_handle, POWER_CTL, reg); // START
	return err;
}
esp_err_t ADXL345_STOP(void)
{
	esp_err_t err = ESP_OK;
	uint8_t reg = 0x00;
	err = WriteRegister(ADXL345_Handle_t->dev_handle, POWER_CTL, reg); // đưa thanh ghi vào chế độ chờ
	return err;
}
esp_err_t ADXL345writeRate(uint8_t rate)
{
	esp_err_t err = ESP_OK;
	ADXL345_Handle_t->_BwRateBits.lowPower = 0;
	ADXL345_Handle_t->_BwRateBits.rate = rate & 0x0F;
	ADXL345_Handle_t->_BwRateBits._BwRateBitstoByte = BwRateBits_toByte;
	err = WriteRegister(ADXL345_Handle_t->dev_handle, BW_RATE, ADXL345_Handle_t->_BwRateBits._BwRateBitstoByte()); //  đưa Measure vào chế độ đo
	return err;
}
esp_err_t ADXL345writeRange(uint8_t range)
{
	esp_err_t err = ESP_OK;
	ADXL345_Handle_t->_dataFormatBits.range = range & 0x03;
	ADXL345_Handle_t->_dataFormatBits.DataFormatBitstoByte = DataFormatBits_toByte;
	err = WriteRegister(ADXL345_Handle_t->dev_handle, DATA_FORMAT, ADXL345_Handle_t->_dataFormatBits.DataFormatBitstoByte()); //  range
	return err;
}
uint8_t ADXL_ReadDevice(void)
{
	uint8_t dev = 0;
	ReadRegister(ADXL345_Handle_t->dev_handle, DEVID, &dev, 1);
	// ESP_LOGI(TAG, "Address Device : 0x%02X", dev);
	return dev;
}
bool ADXL345_Update(void)
{
	// ReadRegister(dev_handle,DATAX0, 6); // đọc tự động 6 thanh ghi 0x32-0x37
	static uint8_t _DATA[6];
	portYIELD();
	if (ReadRegister(ADXL345_Handle_t->dev_handle, DATAX0, _DATA, sizeof(_DATA)) == ESP_OK)
	{
		ADXL345_Handle_t->_Sample._xyz[0] = (_DATA[1] << 8 | _DATA[0]);
		ADXL345_Handle_t->_Sample._xyz[1] = (_DATA[3] << 8 | _DATA[2]);
		ADXL345_Handle_t->_Sample._xyz[2] = (_DATA[5] << 8 | _DATA[4]);
		return true;
	}
	else
	{
		return false;
	}
}
int16_t ADXL345_RawX(void)
{
	return ADXL345_Handle_t->_Sample._xyz[0];
}
int16_t ADXL345_RawY(void)
{
	return ADXL345_Handle_t->_Sample._xyz[1];
}
int16_t ADXL345_RawZ(void)
{
	return ADXL345_Handle_t->_Sample._xyz[2];
}
float ADXL345_GetX(void)
{
	return ADXL345_convertToSI(ADXL345_Handle_t->_Sample._xyz[0]);
}

float ADXL345_GetY(void)
{
	return ADXL345_convertToSI(ADXL345_Handle_t->_Sample._xyz[1]);
}

float ADXL345_GetZ(void)
{
	return ADXL345_convertToSI(ADXL345_Handle_t->_Sample._xyz[2]);
}

float ADXL345_convertToSI(int16_t rawValue) {
  switch (ADXL345_Handle_t->_dataFormatBits.range) {
    case ADXL345_RANGE_2G:
      return rawValue * kRatio2g;

    case ADXL345_RANGE_4G:
      return rawValue * kRatio4g;

    case ADXL345_RANGE_8G:
      return rawValue * kRatio8g;

    case ADXL345_RANGE_16G:
      return rawValue * kRatio16g;

    default:
      return 0;
  }
}
uint8_t DataFormatBits_toByte(void)
{
	uint8_t bits = 0;
	bits |= ADXL345_Handle_t->_dataFormatBits.selfTest << 7;
	bits |= ADXL345_Handle_t->_dataFormatBits.spi << 6;
	bits |= ADXL345_Handle_t->_dataFormatBits.intInvert << 5;
	bits |= ADXL345_Handle_t->_dataFormatBits.fullRes << 3;
	bits |= ADXL345_Handle_t->_dataFormatBits.justify << 2;
	bits |= ADXL345_Handle_t->_dataFormatBits.range;
	// ESP_LOGI(TAG,"Dataformat: %2X\n",bits);
	return bits;
}
uint8_t BwRateBits_toByte(void)
{
	uint8_t bits = 0;
	bits |= ADXL345_Handle_t->_BwRateBits.lowPower << 4;
	bits |= ADXL345_Handle_t->_BwRateBits.rate;
	// ESP_LOGI(TAG,"bwRatebits: %2X\n",bits);
	return bits;
}
static void BwRateBits_init(void){
	ADXL345_Handle_t->_BwRateBits.lowPower = 0;
	ADXL345_Handle_t->_BwRateBits.rate     = 0;
}
static void DataFormatBits_init(void){
	ADXL345_Handle_t->_dataFormatBits.selfTest  = 0 ;
	ADXL345_Handle_t->_dataFormatBits.fullRes   = 0 ;
	ADXL345_Handle_t->_dataFormatBits.intInvert = 0 ;
	ADXL345_Handle_t->_dataFormatBits.justify   = 0 ;
	ADXL345_Handle_t->_dataFormatBits.range     = 0 ;
	ADXL345_Handle_t->_dataFormatBits.spi       = 0 ;
}
// void ADXL_GETXYZ_g(int16_t *x, int16_t *y, int16_t *z, float *gx, float *gy,
// 		float *gz) {
// 	ADXL_GETXYZ(x,y,z); // truyền địa chỉ vào
// 	*gx = *x * 0.0078; // nếu có Gán bién địa chỉ Gx thì mới trả giá trị về
// 	*gy = *y * 0.0078;
// 	*gz = *z * 0.0078;
// }
/*Example Code:
 *  I2C_begin(&hi2c1); // truyền địa chỉ I2C vào
//	Address = scan_I2C(); // code quét địa chỉ I2C
	ADXL345_init(); // khởi tạo lại ADXL345
	ADXL_GETXYZ(&x,&y,&z); // truyền tham số int16 x , y ,z vào
	ADXL_GETXYZ_g(&x,&y,&z,&gx,&gy&,gz); nếu cần sài gia tốc
 */
