#include "OOPConfig.h"

SunEncoder myEnc(4, 5);

void setup() {
  myEnc.init();
  Serial.begin(115200);
  Serial.println("Basic SunEncoder Test:");
}

long oldPosition  = -999;

void loop() {
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.println(newPosition);
  }
}
