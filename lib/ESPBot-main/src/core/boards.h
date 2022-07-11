#pragma once

#define BOARD_UNKNOWN -1

#define BOARD_PHCZJ_MEGA_2560            7000  // PHCZJ
#define BOARD_SG_MEGA_2560            7001  // SG
#define BOARD_QIYI_MEGA_2560            7002  // QIYI

// ARM Cortex-M4F
//

#define BOARD_TEENSY31_32             4100  // Teensy3.1 and Teensy3.2
#define BOARD_TEENSY35_36             4101  // Teensy3.5 and Teensy3.6


//
// Espressif ESP32 WiFi
//
#define BOARD_SUNNYBOT_ESP32         6000  // Generic ESP32



#define _MB_1(B)  (defined(BOARD_##B) && MOTHERBOARD==BOARD_##B)



