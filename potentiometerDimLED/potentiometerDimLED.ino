
int LED = 6; //setting up LED to digital port 6 
int potentiometer = A0; //setting up potentiometer to analog port A0 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED, OUTPUT); //setting up LED as an OUTPUT
}

void loop() {
  // put your main code here, to run repeatedly:
  int knob = analogRead(potentiometer); //setting variable 'knob' to the value of the potentiometer
  knob = map(knob, 1, 1024, 1, 255); 
  Serial.println(knob);
  analogWrite(LED, knob); //sending 'knob', which is a value between 1 to 255, to the LED

}
