#ifndef _Max30003_H
#define _Max30003_H

#include "stdio.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "string.h"
/*SPI CLASS*/
#pragma region SPICLASS
#define MAX30003_SPI_SPEED 3000000

#define SPI_LSBFIRST 0
#define SPI_MSBFIRST 1

#define SPI_MODE0 0
#define SPI_MODE1 1
#define SPI_MODE2 2
#define SPI_MODE3 3


typedef struct
{
  uint32_t _clock;
  uint8_t _bitOrder;
  uint8_t _dataMode;
} SPIsetting;
typedef struct
{
  gpio_num_t SCK_pin;
  gpio_num_t MISO_pin;
  gpio_num_t MOSI_pin;
  gpio_num_t CS_pin; // chip slect
  gpio_num_t INT_pin; // chân ngắt
} SPI_config;
esp_err_t SPI_init_bus(SPI_config *config_pin, uint8_t max_transfer);
esp_err_t SPI_add_device(gpio_num_t _CS, SPIsetting *cfg, spi_device_handle_t *spi_handle);
/*End SPI CLASS*/
#pragma endregion SPICLASS

/* MAX Class*/
#pragma region MAX30003
/*
  @brief register class in side Max30003
  @ Used to access and configure register depending on the application need
*/
#define WREG 0x00
#define RREG 0x01

#define STATUS 0x01
#define EN_INT 0x02
#define EN_INT2 0x03
#define MNGR_INT 0x04
#define MNGR_DYN 0x05
#define SW_RST 0x08
#define SYNCH 0x09
#define FIFO_RST 0x0A
#define INFO 0x0F
#define CNFG_GEN 0x10
#define CNFG_CAL 0x12
#define CNFG_EMUX 0x14
#define CNFG_ECG 0x15
#define CNFG_RTOR1 0x1D
#define CNFG_RTOR2 0x1E
#define ECG_FIFO_BURST 0x20
#define ECG_FIFO 0x21
#define RTOR 0x25

#define CLK_PIN 6
#define RTOR_INTR_MASK 0x04
#define FILTER_WINDOW 5

#define Reg_DCLOFF 0x10

#pragma region CNFG_GEN
/*
@brief register CNFG_GEN 
*/
#define Dis_ULP_Lead_On_Detection (0 << 23) // D[23:22]
#define EN_ULP_Lead_On_Detection  (1 << 22)
/*
@brief FMSTR
Master Clock Frequency. Selects the Master Clock Frequency (FMSTR), and Timing 
Resolution (TRES), which also determines the ECG and CAL timing characteristics.  D[21:20]
* 00 =  FMSTR = 32768Hz,  TRES = 15.26µs  (512Hz ECG progressions)
* 01 =  FMSTR = 32000Hz,  TRES = 15.63µs  (500Hz ECG progressions)
* 10 =  FMSTR = 32000Hz,  TRES = 15.63µs  (200Hz ECG progressions)
* 11 =  FMSTR = 31968.78Hz,  TRES = 15.64µs (199.8049Hz ECG progressions)
*/
#define GEN_FMSTR_32768_512Hz   (0 << 20)
#define GEN_FMSTR_32000_500Hz   (1 << 20)
#define GEN_FMSTR_32000_200Hz   (2 << 20)
#define GEN_FMSTR_31968_199Hz   (3 << 20)
/*
@brief ECG Channel Enable
Note: The ECG channel must be enabled to allow R-to-R operation
*/
#define EN_ECG  (1 << 19)
/*
@brief  DC Lead-Off Detection Enable
* 00 = DC Lead-Off Detection disabled 
* 01 = DCLOFF Detection applied to the ECGP/N pin
* 10 = Reserved. Do not use
* 11 = Reserved. Do not use.
DC Method, requires active selected channel, enables DCLOFF interrupt  
and status bit behavior.
 Uses current sources and comparator thresholds set below.
*/
#define EN_DCLOFF    (1 << 12)
/*
@brief  DC Lead-Off Current Polarity (if current sources are enabled/connected)
* 0 = ECGP - Pullup   ECGN – Pulldown
* 1 = ECGP - Pulldown  ECGN – Pullup
*/
#define DCLOFF_IPOL  (1 << 11)
/*
@brief DC Lead-Off Current Magnitude Selection
* 000 =  0nA (Disable and Disconnect Current Sources)
* 001 =  5nA
* 010 =  10nA
* 011 =  20nA
* 100 =  50nA
* 101 = 100nA
* 110 = Reserved. Do not use.
* 111 = Reserved. Do not use.
*/
#define DCLOFF_IMAG_Disable (0 << 8)
#define DCLOFF_IMAG_5nA     (1 << 8)
#define DCLOFF_IMAG_10nA    (2 << 8)
#define DCLOFF_IMAG_20nA    (3 << 8)
#define DCLOFF_IMAG_50nA    (4 << 8)
#define DCLOFF_IMAG_100nA   (5 << 8) 
/*
@brief  DC Lead-Off Voltage Threshold Selection 
* 00 = VMID ± 300mV
* 01 = VMID ± 400mV
* 10 = VMID ± 450mV
* 11 = VMID ± 500mV
*/
#define DCLOFF_VTH_300mV  (0 << 6)
#define DCLOFF_VTH_400mV  (1 << 6)
#define DCLOFF_VTH_450mV  (2 << 6)
#define DCLOFF_VTH_500mV  (3 << 6)
/*
@brief  Enable and Select Resistive Lead Bias Mode 
* 00 = Resistive Bias disabled 
* 01 = ECG Resistive Bias enabled if EN_ECG is also enabled
* 10 = Reserved. Do not use.
* 11 = Reserved. Do not use.
If EN_ECG is not asserted at the same time as prior to EN_RBIAS[1:0] being set to 
01, then EN_RBIAS[1:0] will remain set to 00.
*/
#define EN_RBIAS   (1 << 4) 
/*
@brief Resistive Bias Mode Value Selection 
* 00 = RBIAS = 50MΩ
* 01 = RBIAS = 100MΩ
* 10 = RBIAS = 200MΩ
* 11 = Reserved. Do not use.
*/
#define EN_RBIASV_50M_Ohm  (0 << 2)
#define EN_RBIAS_100M_Ohm  (1 << 2)
#define EN_RBIAS_200M_Ohm  (2 << 2)
/*
@brief  Enables Resistive Bias on Positive Input
* 0 = ECGP is not resistively connected to VMID
* 1 = ECGP is connected to VMID through a resistor (selected by RBIASV).
*/
#define EN_RBIASP    (1 << 1)
/*
@brief Enables Resistive Bias on Negative Input
* 0 = ECGN is not resistively connected to VMID
* 1 = ECGN is connected to VMID through a resistor (selected by RBIASV).
*/
#define EN_BIASN     (1 << 0)
typedef struct MAX30003_GEN_t{
  uint32_t ULP_LON;
  uint32_t FMSTR;
  uint32_t _EN_ECG;
  uint32_t DCLOFF;
  uint32_t DCIPOL;
  uint32_t DCIMAG;
  uint32_t DCVTH;
  uint32_t EN_RBIAS_;
  uint32_t RBIASV;
  uint32_t RBIASP;
  uint32_t RBIASN;
  uint32_t  REG;
}CNFG_GEN_;
#pragma endregion CNFG_GEN
/*************************************************************
 *                 MAX30003 FMSTR TABLE
 *
 *   FMSTR[1:0]   fMSTR(Hz)     ECG(sps)     RTOR_RES(ms)     CAL_RES(us)
 *   ---------------------------------------------------------------------
 *       00       32768 Hz       512 sps       7.8125 ms        30.52 us
 *
 *       01       32000 Hz       256 sps       8.0000 ms        31.25 us
 *
 *       10       32000 Hz       128 sps       8.0000 ms        31.25 us
 *
 *       11       31968.78 Hz    ~200 sps      8.0080 ms        31.28 us
 *
 * NOTES:
 *- Sample rate phụ thuộc trực tiếp vào FMSTR.
 *- RTOR timing resolution thay đổi theo tần số master.
 *- CAL_RES gần như cố định ~31 µs.
 ************************************************************
 */
#pragma region CNFG_CAL
/*
@brief  Calibration Source (VCALP and VCALN) Enable
* 0 = Calibration sources and modes disabled
* 1 = Calibration sources and modes enabled
*/

#define EN_VCAL  (1 << 22)
/*
@brief Calibration Source Mode Selection 
* 0 = Unipolar, sources swing between VMID ± VMAG and VMID
* 1 = Bipolar, sources swing between VMID + VMAG and VMID - VMAG
*/ 
#define Bipolar  (1 << 21)
#define Unipolar (0 << 21)
/*
@brief Calibration Source Magnitude Selection (VMAG)
* 0 = 0.25mV
* 1 = 0.50mV
*/
#define VMAG_025mV (0 << 20)
#define VMAG_050mV (1 << 20)
/*
******************************************************************************
@brief Calibration Source Frequency Selection (FCAL)
* 000 = FMSTR/128     (Approximately 256Hz)
* 001 = FMSTR /512    (Approximately 64Hz)
* 010 = FMSTR /2048   (Approximately 16Hz)
* 011 = FMSTR /8192   (Approximately 4Hz)
* 100 = FMSTR /215    (Approximately 1Hz)
* 101 = FMSTR /217    (Approximately 1/4Hz)
* 110 = FMSTR /219    (Approximately 1/16Hz)
* 111 = FMSTR /221    (Approximately 1/64Hz)
******************************************************************************
* Note : Actual frequencies are determined by FMSTR selection (see CNFG_GEN for  
* details), approximate frequencies are based on a 32768Hz clock (FMSTR[2:0] = 000). 
* TCAL = 1/FCAL.
*/
#define FCAL_256Hz   (0 << 12)
#define FCAL_64Hz    (1 << 12)
#define FCAL_16Hz    (2 << 12)
#define FCAL_4Hz     (3 << 12)
#define FCAL_1Hz     (4 << 12)
#define FCAL_1_4Hz   (5 << 12)
#define FCAL_1_16Hz  (6 << 12)
#define FCAL_1_64Hz  (7 << 12)
/*
@brief  Calibration Source Duty Cycle Mode Selection
* 0 = Use CAL_THIGH to select time high for VCALP and VCALN
* 1 = THIGH = 50% (CAL_THIGH[10:0] are ignored)
*/
#define CAL_THIGH    (0 << 11) 
#define CAL_THIGH_50 (1 << 11)
/*
@brief Calibration Source Time High Selection
@Note If FIFTY = 1, tHIGH = 50% (and THIGH[10:0] are ignored)
*  otherwise THIGH = THIGH[10:0] x CAL_RES
*  CAL_RES is determined by FMSTR selection (see CNFG_GEN for details);  
*  for example, if FMSTR[2:0] = 000,CAL_RES = 30.52µs.
*/
typedef struct Register_CAL{
 uint32_t _EN_VCAL;
 uint32_t VMODE;
 uint32_t VMAG;
 uint32_t FCAL;
 uint32_t FIFTY;
 uint32_t THIGH;
 uint32_t REG;
}CNFG_CAL_;
#pragma endregion 
#pragma region CNFG_EMUX
/*
@brief ECG Input Polarity Selection
* 0 = Non-inverted
* 1 = Inverted
*/
#define Pol_Inverted        (0 << 23)
#define Pol_NON_Inverted    (1 << 23)
/*
@brief Open the ECGP Input Switch (most often used for testing and calibration studies)
* 0 = ECGP is internally connected to the ECG AFE Channel
* 1 = ECGP is internally isolated from  the ECG AFE Channel
*/
#define EN_ECGP_CN_AFE       (0 << 21)
#define EN_ECGP_ISO_AFE      (1 << 21)
/*
@brief  Open the ECGN Input Switch (most often used for testing and calibration studies)
* 0 = ECGN is internally connected to the ECG AFE Channel
* 1 = ECGN is internally isolated from  the ECG AFE Channel
*/
#define EN_ECGN_CN_AFE       (0 << 20)
#define EN_ECGN_ISO_AFE      (1 << 20)
/*
@brief ECGP Calibration Selection
* 00 = No calibration signal applied
* 01 = Input is connected to VMID
* 10 = Input is connected to VCALP (only available if CAL_EN_VCAL = 1)
* 11 = Input is connected to VCALN (only available if CAL_EN_VCAL = 1)
*/
#define No_Cal_ECGP              (0 << 18)
#define In_VMID_ECGP             (1 << 18)
#define In_VCALP_ECGP            (2 << 18)
#define In_VCALN_ECGP            (3 << 18)
/*
@brief ECGN Calibration Selection
* 00 = No calibration signal applied
* 01 = Input is connected to VMID
* 10 = Input is connected to VCALP (only available if CAL_EN_VCAL = 1)
* 11 = Input is connected to VCALN (only available if CAL_EN_VCAL = 1)
*/
#define No_Cal_ECGN              (0 << 16)
#define In_VMID_ECGN             (1 << 16)
#define In_VCALP_ECGN            (2 << 16)
#define In_VCALN_ECGN            (3 << 16)
typedef struct Register_EMUX{
  uint32_t POL;
  uint32_t OPENP;
  uint32_t OPENN;
  uint32_t CALP_SEL;
  uint32_t CALN_SEL;
  uint32_t REG;
}CNFG_EMUX_;

#pragma endregion CNFG_EMUX

/**
  * @brief      Set Up Sampling Rate
  *             If successful, return true , else release pointer and return fail
  * @return
  *    - ESP_OK: succeed
  *    - others: failed
  */
typedef enum
{
  SAMPLINGRATE_128 = 128,
  SAMPLINGRATE_256 = 256,
  SAMPLINGRATE_512 = 512
} sampRate;
typedef struct {
  uint32_t RR, heartRate;
}Max30003_Info;

typedef struct {
  uint8_t status;
}ETAG;


typedef struct MAX30003_context_t *MAX30003_handle_t;


/**
  * @brief      MAX300003 Begin must be call this function to malloc pointer other  way can't be used
  *             If successful, return true , else release pointer and return fail
  * @return
  *    - ESP_OK: succeed
  *    - others: failed
  */
esp_err_t Max30003_init(MAX30003_handle_t *out_handle, SPI_config *config_pin);
/**
  * @brief      MAx3003 RegWrite like as the name .It's used for write for slave 
  
  *
  * @param      Uint32_t Data which used for write for slave
  *
  * @return
  *    - ESP_OK: succeed
  *    - others: failed
  */
esp_err_t max30003RegWrite( uint8_t WRITE_ADDRESS, uint32_t data);
/**
  * @brief      MAx3003 RegRead like as the name .It's used for read Data from slave 
  
  *
  * @param      Uint8_t  Data receive which is 4 byte return .It's must the pointer point in to buff array 
  * @Ex :   buff[4] = {0}
  *         max30003RegRead(Reg_Addr,buff)
  *         buff[1] << 16; // skip byte 0 cause it remain 32-24 byte which can not use 
  *         buff[2] << 8;
  *         buff[3] 
  * @return
  *    - ESP_OK: succeed
  *    - others: failed
  */
esp_err_t max30003RegRead(uint8_t Reg_address, uint8_t *buff);


void max30003Begin(void);
void max30003SwReset(void);
void max30003Synch(void);


bool max30003ReadInfo(void);
int32_t MAX30003_getEcgSamples(void);
Max30003_Info MAX30003_getHRandRR(void);
void max30003_attach_interrupt(void (*isr_handler)(void*));
void Max30003_ISRgive( BaseType_t *xHigherPriorityTaskWoken);
#pragma endregion MAX30003

#endif