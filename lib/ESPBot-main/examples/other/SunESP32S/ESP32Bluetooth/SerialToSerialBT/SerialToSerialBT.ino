//这个例程为串口和传统蓝牙架设了一座桥梁
//同时也演示了SerialBT和普通的Serial具有一样的功能

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32BT"); //蓝牙模块名称
  Serial.println("蓝牙已启动，你可以配对了！");
}

void loop() {
  if (Serial.available()) {
    SerialBT.write(Serial.read());
    Serial.println("由SerialBT打印");
  }
  if (SerialBT.available()) {
    Serial.write(SerialBT.read());
    Serial.println("由Serial打印");
  }
  delay(20);
}
