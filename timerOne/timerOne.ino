#include <TimerOne.h>

const int TRIGGER_PIN = 11;
const long FPS = 100;
const long PERIOD_US = 1000000 / FPS;

void triggerPulse() {
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(50);
  digitalWrite(TRIGGER_PIN, LOW);
}

void setup() {
  pinMode(TRIGGER_PIN, OUTPUT);
  Timer1.initialize(PERIOD_US);
  Timer1.attachInterrupt(triggerPulse);
}

void loop() {}
