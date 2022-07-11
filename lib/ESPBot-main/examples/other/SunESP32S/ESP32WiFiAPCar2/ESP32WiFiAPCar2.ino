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

IPAddress local_IP(192,168,4,1);
IPAddress gateway(192,168,4,1);
IPAddress subnet(255,255,255,0);

const char* ssid = "ESP32SAP";
const char* password = "12345678";

WiFiUDP Udp;
unsigned int localUdpPort = 9999;
char incomingPacket[255];

//******************PWM引脚和电机驱动引脚***************************//

#define AIN1 32  // M1(A)电机控制PWM波M1
#define AIN2 33  // M1(A)电机控制PWM波

#define BIN1 25  // M2(B) 电机控制PWM波
#define BIN2 26  // M2(B)电机控制PWM波

//******************电机启动初始值 **********************//
int motorDeadZone =
    30;  //高频时电机启动初始值高约为130，低频时电机启动初始值低约为30,和电池电压、电机特性、PWM频率等有关，需要单独测试
/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：PWM 还需要添加限幅
**************************************************************************/
void Set_PWM(int motora, int motorb) {
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

 }

void setup() {
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN2, OUTPUT);

  Serial.begin(115200);

//WiFi.mode(WIFI_AP); //设置工作在AP模式

   WiFi.softAPConfig(local_IP, gateway, subnet); //设置AP地址
  while(!WiFi.softAP(ssid, password)){}; //启动AP
  Serial.println("AP启动成功");

  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP()); // 打印AP IP地址

  WiFi.softAPsetHostname("myHostName"); //设置主机名
  Serial.print("HostName: ");
  Serial.println(WiFi.softAPgetHostname()); //打印主机名

delay(100);
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