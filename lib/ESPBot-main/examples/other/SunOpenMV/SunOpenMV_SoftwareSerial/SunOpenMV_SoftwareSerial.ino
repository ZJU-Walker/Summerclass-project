// arduino 软串口接收 软串口和串口0同时输出
#include <SoftwareSerial.h>
SoftwareSerial OpenMV_Serial(A10, A9);  // RX, TX 和IOENMV串口交叉连接
void setup() {
  // put your setup code here, to run once:
  OpenMV_Serial.begin(9600);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (OpenMV_Serial.available()) {
    // Read the most recent byte
    byte byteRead = OpenMV_Serial.read();
    // char charRead = OpenMV_Serial.read();
    // ECHO the value that was read
    delay(15);  //取消注释会乱码
    OpenMV_Serial.write(byteRead);

    Serial.write(byteRead);
    // Serial.println(byteRead);
  }
}