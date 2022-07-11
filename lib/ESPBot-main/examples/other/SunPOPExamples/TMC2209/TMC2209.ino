#define NEN_PIN          33 // !Enable
#define STEP_PIN         34 // Step
#define DIR_PIN          40 // Direction

bool rotDir = false;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(NEN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(NEN_PIN, LOW);      
  digitalWrite(DIR_PIN, rotDir);      
}


void loop() {

  for (uint16_t i = 800; i>0; i--) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(2000);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(2000);
  }
  rotDir = !rotDir;
  digitalWrite(DIR_PIN, rotDir);
  digitalWrite(LED_BUILTIN, rotDir);
}
