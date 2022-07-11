#define SINGLEGRAY_PIN 12              //灰度传感器引脚
void setup() {
  //start serial connection
  Serial.begin(115200);
  //configure pin 12 as an input and enable the internal pull-up resistor
  pinMode(SINGLEGRAY_PIN, INPUT_PULLUP);


}

void loop() {
  //read the pushbutton value into a variable
  int sensorVal = digitalRead(SINGLEGRAY_PIN);
  //print out the value of the pushbutton
  Serial.println(sensorVal);

}
