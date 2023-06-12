#include<Servo.h>

Servo myservo; //create servo object to control a servo

void setup() {
  // put your setup code here, to run once:
myservo.attach(7,600,2300); //(pin,min,max) min and max pulsewidth
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

myservo.write(0);
delay(150);

myservo.write(20);
delay(150);

}
