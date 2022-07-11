
#include <MPU6050_tockn.h> //点击自动打开管理库页面安装: http://librarymanager/All#MPU6050_tockn
#include <Wire.h>

MPU6050 mpu6050(Wire);

long timer = 0;

void setup()
{
    Serial.begin(115200);
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