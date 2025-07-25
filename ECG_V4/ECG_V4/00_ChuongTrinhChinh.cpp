#include <EEPROM.h>
#include "00_ChuongTrinhChinh.h"
#include "01_Max30003.h"  // Thư viện MAX30003
#include "02_POSTGET.h"
#include "03_ThongSoBoard.h"
#include "04_ESPNOW.h"
#include "05_WIFI.h"
#include "06_Flags.h"

// #include "esp_sleep.h"
// extern "C" {
//   #include "driver/uart.h"
// }

_MAX30003 _max30003(CS_PIN_MAX30003);  // ECG
WIFI _WiFi;                            // Kết nối WiFi cho board.
Flags _Flags;                          // Cờ định thời gian thực thi cách lệnh.
ESPNOW _ESPNow;                        // Kết nối ESP-NOW giữa các ESP với nhau
POSTGET _POSTGET;                      // Các hàm thực thi POST - GET giữa device và serve.
ThongSoBoard _ThongSoBoard;            // Thông số cài đặt cho board lưu trong EEPROM.

#define myEvent
SemaphoreHandle_t EventHandle;

struct ECGFIFO {
  int32_t EcgWave;  // 4 byte
  int16_t HR;       // 2 byte  0~256 bpm
  int16_t RR;       // 2 byte
  int8_t Bat;       // 1 byte // mức độ pin tiêu thụ 0-100%
  int8_t Accel;     // 1 byte ~~ 0-256 tính độ phần trăm té ngã 0-100%
}; 


bool StatusConnection = false;
String ID_Mac_Board;  // Số ID của ESP32, đây là số IMEI của board.
uint8_t _GiuBootDePhatAP = 0;
unsigned long long Esp32delaytime = 0;
int delaytime = 2;
int8_t percBat, Final_VBat = 100;
bool rtorIntrFlag = false;

#ifdef myEvent
void ECGTask(void* pvParameters) {
  uint8_t i = 0;
  // uint32_t start = 0;
  ECGFIFO FIFO[20];
  for (;;) {
    if (xSemaphoreTake(EventHandle, portMAX_DELAY) == pdPASS) {
      if (_WiFi.DaBatAP != 1) {
        // if (_max30003.STATUS_RR()) {  // nếu phát hiện song R
        // if (millis() - Esp32delaytime >= delaytime)  //test whether the period has elapsed
        // {
        // if (rtorIntrFlag && _max30003.STATUS_RR() == 1) {
        // rtorIntrFlag = false;
        // Esp32delaytime = millis();
        if (_max30003.STATUS_RR()) {
          _max30003.Nhipdapbpm();
          FIFO[i].EcgWave = int32_t(_max30003.EcgWave());
          FIFO[i].HR = int16_t(_max30003.HR);
          FIFO[i].RR = int16_t(_max30003.RR);
          FIFO[i].Bat = percBat;
#ifdef debug
          Serial.println(_max30003.EcgWave());
#endif
          i++;
          if (i >= 20 ) {
            // start = millis();
            esp_now_send(_ESPNow.MACMasterDungDinhDang, (uint8_t*)&FIFO, sizeof(ECGFIFO)* 20);
            i = 0;
          }
        }
      }
      _Flags.TurnONFlags();
      ThucThiTacVuTheoFLAG();
      _Flags.TurnOFFFlags();
    }
  }
}
#endif
void rtorInterruptHndlr() {
#ifdef myEvent
  // if (millis() - Esp32delaytime >= delaytime) {  // thêm cơ chế delay tránh cho chương trình nhảy vào Watchdog
    xSemaphoreGiveFromISR(EventHandle, NULL);    // giống lệnh XSemaphoreGive nhưng an toàn hơn giúp đảm bảo hàm ECGTask sẽ chạy tránh tình trạng nhảy vào watchdog
    // Esp32delaytime = millis();
  // }
#endif
#ifndef myEvent
  rtorIntrFlag = true;
#endif
}
void KhoiTao() {
  Serial.begin(115200);
  analogReadResolution(12);

  pinMode(NutBoot, INPUT);
  pinMode(INT, INPUT_PULLUP);
  // Khởi tạo bộ nhớ ROM của ESP32
  EEPROM.begin(512);
  delay(10);

  // ID_Mac_Board = "ABCDEF";
  _ThongSoBoard.KhoiTao();  // lấy mac từ EPROM được lưu từ 0 -> MAc length

  strncpy(_ESPNow.MACMaster, _ThongSoBoard.MACMaster, sizeof(_ThongSoBoard.MACMaster));
  _ESPNow.khoitao_GuiFIFO(OnDataSent);
  ID_Mac_Board = _WiFi.LaySoMAC();
#ifdef debug
  Serial.print("Mac Esp-nologo: ");
  Serial.println(ID_Mac_Board);
  Serial.print("_ESPNow.MACMaster: ");
  Serial.println(_ESPNow.MACMaster);
#endif
  pinMode(CS_PIN_MAX30003, OUTPUT);
  digitalWrite(CS_PIN_MAX30003, HIGH);  // Vô hiệu hóa MAX30003 lúc đầu

  // Khởi tạo giao tiếp SPI
  SPI.begin(SCK, SO, SI, CS_PIN_MAX30003);
  SPI.setBitOrder(MSBFIRST);   //quy định thứ tự các bit được truyền và nhận: ưu tiên trọng số cao nhất
  SPI.setDataMode(SPI_MODE0);  // Mode 0 lấy bit ở cạnh lên
  _max30003.Khoitao();

#ifdef debug
  Serial.printf("Pin lúc đầu : %d\n", Final_VBat);
#endif
#ifdef myEvent
  EventHandle = xSemaphoreCreateBinary();  // tạo Semaphore nhị phân để tạo biến 0 và 1 , khi có ngắt xảy ra nó sẽ thực thi chương trình ngắt và đưa biến lên 1 và chạy chương trình chính
  xTaskCreate(
    ECGTask, "Task Đọc sóng và gửi ESP NOW", 4096  // Stack size
    ,
    NULL  // Pass reference to a variable describing the task number
    ,
    2  // Low priority
    ,
    NULL  // Task handle is not used here - simply pass NULL
  );
#endif
  attachInterrupt(digitalPinToInterrupt(INT), rtorInterruptHndlr, CHANGE);  // khởi tạo ngắt
  // xSemaphoreGive(EventHandle);
#ifdef debug
  Serial.print("Khởi tạo ngắt thành công");
#endif
}
void ChayChuongTrinhChinh() {
#ifndef myEvent
  if (_WiFi.DaBatAP != 1) {
    // if (_max30003.STATUS_RR()) {                   // nếu phát hiện song R
    // if (millis() - Esp32delaytime >= delaytime)  //test whether the period has elapsed
    // {
    if (rtorIntrFlag && _max30003.STATUS_RR() == 1) {
      rtorIntrFlag = false;
      // Esp32delaytime = millis();
      // if (_max30003.STATUS_RR()) {
      _max30003.Nhipdapbpm();
      FIFO.EcgWave = int32_t(_max30003.EcgWave());
      FIFO.HR = int16_t(_max30003.HR);
      FIFO.RR = int16_t(_max30003.RR);
      FIFO.Bat = percBat;
#ifdef debug
      Serial.println(_max30003.EcgWave());
#endif
      esp_now_send(_ESPNow.MACMasterDungDinhDang, (uint8_t*)&FIFO, sizeof(FIFO));
    }
    // }
    // }
    // _Flags.TurnONFlags();
    // ThucThiTacVuTheoFLAG();
    // _Flags.TurnOFFFlags();
  }
#endif
}
void ThucThiTacVuTheoFLAG(void) {
#pragma region ThucThiTacVuTheoFLAG
#pragma region Flag100ms
#ifdef _Flag_100ms
  if (_Flags.Flag.t100ms) {
    if (digitalRead(NutBoot) == LOW) {
      _GiuBootDePhatAP++;
      Serial.print("NHAN GIU LAN THU: ");
      Serial.println(_GiuBootDePhatAP);
    }
  }
#endif
#pragma endregion Flag100ms
#pragma region Flag500ms
#ifdef _Flag_500ms
  if (_Flags.Flag.t500ms) {
    if (_GiuBootDePhatAP > 10) {
      _WiFi.ThietLapAP();
    }
    ReadBat();
  }
#endif
#pragma endregion Flag500ms
}
#pragma endregion ThucThiTacVuTheoFLAG
#pragma region đọc điện áp Pin
void ReadBat(void) {
  int adcValue = analogRead(PIN_BATTERY);
  float voltage = adcValue * (3.3 / 4095.0);  // 3v3 đầu vào
  // Tính VBAT dựa trên mạch chia áp
  float vbat = voltage * ((R1 + R2) / R2);
  percBat = int8_t((vbat - 3.3) * 100.0 / (4.2 - 3.2));
  if (percBat < Final_VBat) {  // nếu điện áp Pin lúc trước nhỏ hơn điện áp Pin lúc sau // ví dụ 59 > 60
    Final_VBat = percBat;      // thì điện áp lúc sau sẽ bằng điện áp Pin lúc trước     // 60 = 59 ->  hiển thị 59
  } else if (percBat > Final_VBat && _max30003.STATUS_RR() != 1) {
    Final_VBat = percBat;
  }
  // nếu điện áp Pin lúc trước lớn hơn điện áp Pin lúc sau  // ví dụ 60 > 59 // hiển thị 60 ,trong trường hợp sạc thì nó sẽ kiểm tra DCLOFF , nếu kh có DClOFF thì nó trong quá trình sạc
  percBat = constrain(Final_VBat, 0, 100);
}
#pragma endregion dọc điện áp Pin
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    StatusConnection = true;
  } else {
    StatusConnection = false;
  }
}
// static int nap(unsigned int timeout_sec, int uart) {

//   if (!timeout_sec) {
//     esp_sleep_enable_uart_wakeup(uart);
//     uart_set_wakeup_threshold(uart, 1);
//   } else {
//     esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_UART);
//     esp_sleep_enable_timer_wakeup(1000000UL * timeout_sec);
//   }
//   esp_light_sleep_start();
// #ifdef debug
//   Serial.print("Resuming..");
// #endif
//   return 0;
// }