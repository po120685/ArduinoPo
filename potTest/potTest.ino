int potPin = A5;
int potVal = 0; 

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
pinMode(potPin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
potVal = analogRead(potPin);
Serial.println(potVal);
delay(20);
}
