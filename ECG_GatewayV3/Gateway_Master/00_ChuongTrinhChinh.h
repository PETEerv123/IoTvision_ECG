#ifndef _ChuongTrinhChinh_h
#define _ChuongTrinhChinh_h

#include "Gateway_Master.h"  // Có define debug để bật/tắt các debug ra Serial.
#include <Arduino.h>
#include <Arduino_JSON.h>  // Thư viện xử lý dữ liệu kiểu JSON
#include <Wire.h>          // Để kết nối I2C với mô-đun RTC (thời gian thực),
                           // mô-đun đọc cảm biến nhiệt độ & độ ẩm SHT3x.



void KhoiTao(void);
void ChayChuongTrinhChinh(void);
;
void ThucThiTacVuTheoCODE(void);



#endif
