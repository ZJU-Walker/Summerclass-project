#pragma once  //防止头文件被多次引用

#include "Arduino.h"
#ifndef VOLT_PIN
#define VOLT_PIN A0  ///<电压检测引脚A0
#endif
#ifndef VOLT_THRESHOLD
#define VOLT_THRESHOLD 1110  ///<电压检测引脚A0
#endif



class Battery
{
public:
	Battery();
	Battery(byte battery_Pin, unsigned int threshold = VOLT_THRESHOLD);
	unsigned int read(void);
	bool is_Volt_Low(void);

private:
	byte _Battery_Pin;
	unsigned int _Threshold;
};


