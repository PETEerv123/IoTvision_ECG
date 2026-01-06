#ifndef _Config_H
#define _Config_H

#ifdef _cplusplus
    extern "C"{
#endif

#define SO 5
#define SI 7
#define SCK 6
#define INT 8
#define CS_PIN_MAX30003 10 // Chân CS của MAX30003

#define SCL 0
#define SDA 1

#define PIN_BATTERY 4  // 
#define R1 200000.0    // 220000 Ohm
#define R2 100000.0    // 100000 Ohm

#define Bat_min_volt 3300
#define Bat_max_volt 4200
#define NutBoot 9

#define led_pin_state 3
#define constrain(x,min, max) ((x < min) ? min : (x > max) ? max : x) 

typedef struct {
    float x;   // giá trị ước lượng
    float P;   // sai số ước lượng
    float Q;   // nhiễu hệ thống (process noise)
    float R;   // nhiễu đo (measurement noise)
} Kalman1D_t;
#define EINT        (1U << 23)
#define EOVF        (1U << 22)
#define FSTINT      (1U << 21)
#define DCLOFFINT   (1U << 20)
#define LDOFF_PH    (1U << 3)
#define LDOFF_PL    (1U << 2)
#define LDOFF_NH    (1U << 1)
#define LDOFF_NL    (1U << 0)

typedef struct {
    uint8_t _EINT;
    uint8_t _EOVF;
    uint8_t _FSTINT;
    uint8_t _DCLOFFINT;
    uint8_t _LDOFF_PH;
    uint8_t _LDOFF_PL;
    uint8_t _LDOFF_NH;
    uint8_t _LDOFF_NL;
} Status_ECG_Handle;


void Bat_Config(void);
void Led_State_init(void);
void esp32_now_init(void); 
static void _wifi_init(void);
void MAX300003_STATUS_Sample(Status_ECG_Handle* Status);
void Init_Bluetooth_GATT_Sever(void);


#ifdef _cplusplus
    }
#endif



#endif