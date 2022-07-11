/**
 * @file ESP32_WiFiCar.ino 连接wifi开环控制小车
 * @author igcxl (igcxl@qq.com)
 * @brief 连接wifi开环控制小车
 * @note 连接wifi开环控制小车
 * @version 0.9
 * @date 2021-11-02
 * @copyright Copyright © Sunnybot.cn 2021
 *
 */
#include <WiFi.h>
#include <WiFiUdp.h>

#include "analogWrite.h"

int AX, AY, BX, BY;
int M1_Speed, M2_Speed, M3_Speed, M4_Speed;
int local_max;

const char* ssid = "testap";
const char* password = "12345678";

WiFiUDP Udp;
unsigned int localUdpPort = 9999;
char incomingPacket[255];

//******************PWM引脚和电机驱动引脚***************************//

#define AIN1 32  // A电机控制PWM波
#define AIN2 33  // A电机控制PWM波

#define BIN1 25  // B电机控制PWM波
#define BIN2 26  // B电机控制PWM波

#define CIN1 27  // C电机控制PWM波
#define CIN2 14  // C电机控制PWM波//该端口重启时会输出高电平 需更换

#define DIN1 13  // D电机控制PWM波
#define DIN2 12  // D电机控制PWM波
//******************电机启动初始值 **********************//
int motorDeadZone =
    30;  //高频时电机启动初始值高约为130，低频时电机启动初始值低约为30,和电池电压、电机特性、PWM频率等有关，需要单独测试
/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：PWM 还需要添加限幅
**************************************************************************/
void Set_PWM(int motora, int motorb, int motorc, int motord) {
  if (motora > 0)
    analogWrite(AIN2, motora + motorDeadZone),
        analogWrite(AIN1, 0);  //赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if (motora == 0)
    analogWrite(AIN2, 0), analogWrite(AIN1, 0);
  else if (motora < 0)
    analogWrite(AIN1, -motora + motorDeadZone),
        analogWrite(
            AIN2,
            0);  //高频时电机启动初始值高约为130，低频时电机启动初始值低约为30

  if (motorb > 0)
    analogWrite(BIN2, motorb + motorDeadZone),
        analogWrite(BIN1, 0);  //赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if (motorb == 0)
    analogWrite(BIN2, 0), analogWrite(BIN1, 0);
  else if (motorb < 0)
    analogWrite(BIN1, -motorb + motorDeadZone),
        analogWrite(
            BIN2,
            0);  //高频时电机启动初始值高约为130，低频时电机启动初始值低约为30

  if (motorc > 0)
    analogWrite(CIN1, motorc + motorDeadZone),
        analogWrite(CIN2, 0);  //赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if (motorc == 0)
    analogWrite(CIN2, 0), analogWrite(CIN1, 0);
  else if (motorc < 0)
    analogWrite(CIN2, -motorc + motorDeadZone),
        analogWrite(
            CIN1,
            0);  //高频时电机启动初始值高约为130，低频时电机启动初始值低约为30

  if (motord > 0)
    analogWrite(DIN1, motord + motorDeadZone),
        analogWrite(DIN2, 0);  //赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if (motord == 0)
    analogWrite(DIN1, 0), analogWrite(DIN2, 0);
  else if (motord < 0)
    analogWrite(DIN2, -motord + motorDeadZone),
        analogWrite(
            DIN1,
            0);  //高频时电机启动初始值高约为130，低频时电机启动初始值低约为30
}

void setup() {
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(CIN1, OUTPUT);
  pinMode(DIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(CIN2, OUTPUT);
  pinMode(DIN2, OUTPUT);
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  while (!WiFi.isConnected()) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("Connected");
  Serial.print("IP Address:");
  Serial.println(WiFi.localIP());

  Udp.begin(localUdpPort);  //启用UDP监听以接收数据
}

void loop() {
  if (Udp.parsePacket())  //解析包不为空
  {
    memset(incomingPacket, 0, sizeof(incomingPacket));
    Udp.read(incomingPacket, 255);

    //解析
    int Data = atoi(incomingPacket);
    AX = Data / 1000000 - 16;
    AY = (Data / 10000) % 100 - 16;
    BX = (Data / 100) % 100 - 16;
    BY = Data % 100 - 16;
    Serial.print("AX:");
    Serial.println(AX);
    Serial.print("AY:");
    Serial.println(AY);
    Serial.print("BX:");
    Serial.println(BX);
    Serial.print("BY:");
    Serial.println(BY);

    M1_Speed = (+BX + AX - AY) * 8;
    M2_Speed = (-BX - AX - AY) * 8;
    M3_Speed = (+BX - AX - AY) * 8;
    M4_Speed = (-BX + AX - AY) * 8;
    Set_PWM(M1_Speed, M2_Speed, M3_Speed, M4_Speed);
  }
}