#define PIN_LED 7
unsigned int count;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  count = 0;
}

void loop() {
  if(count < 5) {
    digitalWrite(PIN_LED, 0);
    delay(1000);
    for(int i=0; i<6; i++) {
      digitalWrite(PIN_LED, 0);
      delay(100);
      digitalWrite(PIN_LED, 1);
      delay(100);
      count++;
    }
  } else {
    digitalWrite(PIN_LED, 1);
  }
}
