#include<Servo.h>

Servo myservo; //create servo object to control a servo

const int servoAnalogOut = A1; 
int servoPosition;
unsigned int servoValue0Deg, servoValue180Deg; // Variables to store min and max values of servo position
int pos = 0; // set pos as variable

void setup() {
  // put your setup code here, to run once:
//myservo.attach(9,600,2300); //(pin,min,max) min and max pulsewidth
myservo.attach(7);
Serial.begin(9600);
calibration();
}

void loop() {
  // put your main code here, to run repeatedly:

servoPosition = map(analogRead(servoAnalogOut),servoValue0Deg,servoValue180Deg, 0 , 180); // map function calibrates map(value,fromLow, fromHigh, toLow, toHigh)
Serial.println(servoPosition);
delay(1000);
//
//myservo.write(0);
//delay(1500);
//
//myservo.write(170);
//delay(1500);

}

void calibration() {
  myservo.write(0); // set servo to 0 position
  delay(2000);
  servoValue0Deg = analogRead(servoAnalogOut); // read value for position 0
  Serial.println("Pot value for 0 deg is " + String(servoValue0Deg));
  delay(500);
  myservo.write(180); //set servo to 180 degrees
  delay(2000);
  servoValue180Deg = analogRead(servoAnalogOut);
  Serial.println("Potvalue for 180 deg is " + String(servoValue180Deg)); // read value for 180 degrees
  delay(500);
  Serial.println("Now going to 90 Degrees");
  myservo.write(90);
  delay(2000);
  myservo.detach();
  delay(1000);
}
