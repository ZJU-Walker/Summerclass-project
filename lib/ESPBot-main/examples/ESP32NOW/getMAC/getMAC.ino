#include <Arduino.h>
#include <WiFi.h>
uint8_t mac[6];
char macStr[30] = {0};
char szMAC[18] ;
String macAdr= WiFi.macAddress();
void setup() {
  Serial.begin(115200);
  Serial.println();
  strcpy(szMAC,macAdr.c_str());
  Serial.print("ESP Board MAC Address:  ");
  Serial.println(szMAC);
  // sscanf(&(WiFi.macAddress()),"%02X:%02X:%02X:%02X:%02X:%02X", mac[0],
  // mac[1], mac[2], mac[3], mac[4], mac[5])

  sscanf(szMAC, "%02x:%02x:%02x:%02x:%02x:%02x", &mac[0], &mac[1], &mac[2],
         &mac[3], &mac[4], &mac[5]);
  Serial.println(mac[0]);
  Serial.println(mac[1]);
  Serial.println(mac[2]);
  Serial.println(mac[3]);
  Serial.println(mac[4]);
  Serial.println(mac[5]);
  sprintf(macStr, "0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X", mac[0], mac[1],
          mac[2], mac[3], mac[4], mac[5]);

  Serial.println(macStr);
}

void loop() {}