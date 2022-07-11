// arduino 软串口接收 软串口和串口0同时输出
#include <SoftwareSerial.h>
SoftwareSerial OpenMV_Serial(A12, A11);  // RX, TX
void setup() {
  // put your setup code here, to run once:
  OpenMV_Serial.begin(9600);
  Serial.begin(9600);
  OpenMV_Serial.listen();
}

void loop() {
  int32_t temp = 0;
    char buff[100] = {0};
  // put your main code here, to run repeatedly:
  
  if (OpenMV_Serial.available()) {
    while (OpenMV_Serial.available()) {
      // Read the most recent byte
      buff[temp++] = OpenMV_Serial.read();
      
      //char charRead = OpenMV_Serial.read();
      }
      // ECHO the value that was read

     
      
      Serial.write(buff);
      delay(15);//取消注释会乱码 根据buff值调整延时时间
      OpenMV_Serial.write(buff);//取消注释会乱码
      //Serial.println(byteRead);
    

  }
  //  delay(500);
  //   OpenMV_Serial.write("hello world?");
  //   OpenMV_Serial.println("test@");
}