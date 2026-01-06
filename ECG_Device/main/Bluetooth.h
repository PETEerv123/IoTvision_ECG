#ifndef _Bluetooth_Private_
#define _Bluetooth_Private_

#ifdef _cplusplus
    extern "C"{
#endif

#include "stdio.h"
/* NimBLE GAP APIs */
#include "host/ble_gap.h"
#include "services/gap/ble_svc_gap.h"


/* Defines */
#define BLE_GAP_APPEARANCE_GENERIC_TAG 0x0200
#define BLE_GAP_URI_PREFIX_HTTPS 0x17
#define BLE_GAP_LE_ROLE_PERIPHERAL 0x00

static void nimble_host_config_init(void);
#ifdef _cplusplus
    }
#endif


#endif