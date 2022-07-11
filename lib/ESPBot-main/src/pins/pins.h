#pragma once

#include "../core/boards.h"



#if _MB_1(PHCZJ_MEGA_2560)
  #include "mega2560/pins_PHCZJ_MEGA_2560.h"             // ATmega1280, ATmega2560                 env:mega1280 env:mega2560
#elif _MB_1(SG_MEGA_2560)
  #include "mega2560/pins_SG_MEGA_2560.h"              // ATmega1280, ATmega2560                 env:mega1280 env:mega2560
#elif _MB_1(QIYI_MEGA_2560)
  #include "mega2560/pins_QIYI_MEGA_2560.h"              // ATmega1280, ATmega2560                 env:mega1280 env:mega2560
  #elif _MB_1(SUNNYBOT_ESP32)
  #include "esp32/pins_SUNNYBOT_ESP32.h"              
  #include "analogWrite.h"             
#endif

