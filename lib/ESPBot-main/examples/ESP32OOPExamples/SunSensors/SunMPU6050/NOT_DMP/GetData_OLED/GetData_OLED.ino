
#include <MPU6050_tockn.h> //点击自动打开管理库页面安装: http://librarymanager/All#MPU6050_tockn
#include <Wire.h>
#include <U8g2lib.h> //点击自动打开管理库页面并安装: http://librarymanager/All#U8g2
MPU6050 mpu6050(Wire);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/SCL, /* data=*/SDA); // ESP32 Thing, HW I2C with pin remapping

long timer = 0;

void setup()
{
    Serial.begin(115200);
    u8g2.begin();
  u8g2.enableUTF8Print();
    Wire.begin();
    mpu6050.begin();
    Serial.println(millis());
    mpu6050.calcGyroOffsets(true);//需要6.6秒时间
    Serial.println(millis());
    // mpu6050.setGyroOffsets(-1.36, -0.49, 0.96);
}

void loop()
{
    mpu6050.update();

    if (millis() - timer > 1000)
    {
          u8g2.clearBuffer();                         // clear the internal memory
  u8g2.setFont(u8g2_font_unifont_t_chinese2); // choose a suitable font
  u8g2.setCursor(0, 16);
  u8g2.print(mpu6050.getGyroAngleZ()); //汉字要用print
  u8g2.sendBuffer();  

        Serial.println("=======================================================");

        Serial.print("gyroAngleX : ");
        Serial.print(mpu6050.getGyroAngleX());
        Serial.print("\tgyroAngleY : ");
        Serial.print(mpu6050.getGyroAngleY());
        Serial.print("\tgyroAngleZ : ");
        Serial.println(mpu6050.getGyroAngleZ());

        Serial.print("angleX : ");
        Serial.print(mpu6050.getAngleX());
        Serial.print("\tangleY : ");
        Serial.print(mpu6050.getAngleY());
        Serial.print("\tangleZ : ");
        Serial.print(mpu6050.getAngleZ());
        Serial.println("\t单位:度");
        Serial.println("=======================================================\n");
        timer = millis();
    }
}