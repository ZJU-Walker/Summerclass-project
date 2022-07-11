#include <WiFi.h>

const char* ssid = "ESP32_Web";     // Enter SSID 请修改为"ESP32_学号"
const char* password = "12345678";  // Enter Password

void setup() {
  Serial.begin(115200);

  // 建立WiFi热点 Create AP
  WiFi.softAP(ssid, password);
  //打印热点IP
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
}

void loop() {}