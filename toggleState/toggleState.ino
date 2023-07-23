int button = 9; 
int led = 13;
int state;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(button, INPUT_PULLUP);
  pinMode(led, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
 state = digitalRead(button);
  while (state == 0) {
    digitalWrite(led, HIGH);
    Serial.println(state);
    delay(5000);
    digitalWrite(led,LOW);
    delay(5000);
    state = digitalRead(button);
   if (state == 1) {
     break;
   }
  } 
  if (state == 1) {
    digitalWrite(led,LOW);
    Serial.println(state);

  }
}
