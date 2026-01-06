#ifndef _ADXL345_H

#define _ADXL345_H

/*
@Name Author : Ta Thuan
@Program in 29/11/2025 6:05:5 PM
@Library ADXL345 : supported by Github
*/
#ifdef __cplusplus
 extern "C" {
#endif

#include "driver/i2c.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#define ADXL345_ADDR       0x53

#define Default_I2C_Clock  200000 //200k Hz



// Data Rate
#define ADXL345_RATE_3200HZ   0x0F    // 3200 Hz
#define ADXL345_RATE_1600HZ   0x0E    // 1600 Hz
#define ADXL345_RATE_800HZ    0x0D    // 800 Hz
#define ADXL345_RATE_400HZ    0x0C    // 400 Hz
#define ADXL345_RATE_200HZ    0x0B    // 200 Hz
#define ADXL345_RATE_100HZ    0x0A    // 100 Hz
#define ADXL345_RATE_50HZ     0x09    // 50 Hz
#define ADXL345_RATE_25HZ     0x08    // 25 Hz
#define ADXL345_RATE_12_5HZ   0x07    // 12.5 Hz
#define ADXL345_RATE_6_25HZ   0x06    // 6.25 Hz
#define ADXL345_RATE_3_13HZ   0x05    // 3.13 Hz
#define ADXL345_RATE_1_56HZ   0x04    // 1.56 Hz
#define ADXL345_RATE_0_78HZ   0x03    // 0.78 Hz
#define ADXL345_RATE_0_39HZ   0x02    // 0.39 Hz
#define ADXL345_RATE_0_20HZ   0x01    // 0.20 Hz
#define ADXL345_RATE_0_10HZ   0x00    // 0.10 Hz

// Range
#define ADXL345_RANGE_2G      0x00    // +-2 g
#define ADXL345_RANGE_4G      0x01    // +-4 g
#define ADXL345_RANGE_8G      0x02    // +-8 g
#define ADXL345_RANGE_16G     0x03    // +-16 g

typedef struct ADXL345
{
  gpio_num_t __SCL;
  gpio_num_t __SDA;
}I2C_Config;

typedef struct{
  int16_t _xyz[3];
  float gx,gy,gz;
}ADXL345_Sample;

typedef struct _dataFormatBits{
  uint8_t selfTest;   // D7
  uint8_t spi;        // D6
  uint8_t intInvert;  // D5
  uint8_t fullRes;    // D3
  uint8_t justify;    // D2
  uint8_t range ;      // D1 - D0
  uint8_t (*DataFormatBitstoByte)(void);
}dataFormatBits_t;

typedef struct _BwRateBits{
  uint8_t lowPower;   // D4
  uint8_t rate;       // D3 - D0
  uint8_t (*_BwRateBitstoByte)(void);
}BwRateBits_t;

typedef struct _PowerCtlBits{
  uint8_t link;       // D5
  uint8_t autoSleep;  // D4
  uint8_t measure;    // D3
  uint8_t sleep;      // D2
  uint8_t wakeup;     // D1 - D0
  uint8_t (*_PowerCtlBitstoByte)(struct _PowerCtlBits *self);
}PowerCtlBits_t;


typedef struct{
  i2c_master_dev_handle_t dev_handle;
  I2C_Config _cfg;
  ADXL345_Sample _Sample;
  dataFormatBits_t _dataFormatBits;
  BwRateBits_t    _BwRateBits;
  PowerCtlBits_t  _PowerCtlBits_t;
}ADXL345_Handle;


// int8_t scan_I2C(void);
void ADXL345_init(I2C_Config _I2C_Config);

esp_err_t WriteRegister(i2c_master_dev_handle_t i2c_dev,uint8_t ResgisterAddress,uint8_t cmd);
esp_err_t ReadRegister(i2c_master_dev_handle_t i2c_dev,uint8_t ResgisterAddress ,uint8_t *buff , uint8_t numberbyte);
float ADXL345_convertToSI(int16_t rawValue);
int16_t ADXL345_RawX(void);
int16_t ADXL345_RawY(void);
int16_t ADXL345_RawZ(void);
float ADXL345_GetX(void);
float ADXL345_GetY(void);
float ADXL345_GetZ(void);
esp_err_t ADXL345_START(void);
esp_err_t ADXL345_STOP(void);
esp_err_t ADXL345writeRate(uint8_t rate);
esp_err_t ADXL345writeRange(uint8_t range);
uint8_t ADXL_ReadDevice(void);
bool ADXL345_Update(void);
// void ADXL_GETXYZ(int16_t *x ,int16_t *y , int16_t *z);
// void ADXL_GETXYZ_g(int16_t *x, int16_t *y, int16_t *z, float *gx, float *gy,float *gz);
uint8_t DataFormatBits_toByte(void);
uint8_t BwRateBits_toByte(void);

#ifdef __cplusplus
}
#endif


#endif
