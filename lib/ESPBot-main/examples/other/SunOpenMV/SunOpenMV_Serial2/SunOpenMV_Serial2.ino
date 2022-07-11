// arduino 硬串口2接收 串口2和串口0同时输出
void setup() {
  // put your setup code here, to run once:
 Serial2.begin(19200);
  Serial.begin(19200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (OpenMV_Serial.available()) {
    // Read the most recent byte
    byte byteRead =Serial2.read();
    // char charRead =Serial2.read();
    // ECHO the value that was read
   
   Serial2.write(byteRead);

    Serial.write(byteRead);
    // Serial.println(byteRead);
  }
}