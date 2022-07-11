#define UP HIGH   //按键松开UP 高电平“HIGH”
#define DOWN LOW  //按键松开DOWN 低电平“LOW”
void setup() {
  Serial.begin(115200);
  pinMode(30, INPUT_PULLUP);
  while (digitalRead(30) == UP) {
    Serial.print("isRobotOn:");
    Serial.println(!digitalRead(30));
  }
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Sunnybot On");
  delay(1000);
}
