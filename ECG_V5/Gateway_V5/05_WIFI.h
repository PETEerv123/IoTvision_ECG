#ifndef _WiFi_h
#define _WiFi_h

#include "Gateway_V5.h"  // Có define debug để bật/tắt các debug ra Serial.
#include <Arduino.h>       // Để khai báo kiểu String

//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
//============================ Begin: classWiFi =====================================================//
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
#pragma region classWiFi
class WIFI {
public:
  bool DaBatAP;
  static bool DaLuuMACVaoEEPROM;
  
private:
  String AP_SSID = "Gateway@2025";
  String AP_PASS = "IoTvision@2025";

public:
  String LaySoMAC(void);
  void ThietLapAP(void);
  static void ServerON(void);

private:
  static void TrangChu(void);
  static void StyleCSS(void);
  static void CodeJS(void);
  static void LuuMACMasterVaoEEPROM(void);
};
#pragma endregion classWiFi
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
//============================ End: classWiFi =======================================================//
//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM

#endif
