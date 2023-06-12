//random oreintation 
//speed they can go at
//gait transition
//tubing 
//0-180 
#include <Servo.h>

Servo myservo2;
//Servo myservo3;
Servo myservo4;
Servo myservo5;
Servo myservo6;
Servo myservo7;
Servo myservo8;
Servo myservo9;
Servo myservo10;
Servo myservo11;
Servo myservo12;
Servo myservo13;
int intPW2;
int intPW4;
int intPW5;
int intPW6;
int intPW7;
int intPW8;
int intPW9;
int intPW10;
int intPW11;
int intPW12;
int intPW13;
int gaitSpeedCC = 170; 
int gaitSpeedC  = 10;
int stopPW    = 90;

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);

myservo2.attach(2);
//myservo3.attach(3);
myservo4.attach(4);
myservo5.attach(5);
myservo6.attach(6);
myservo7.attach(7);
myservo8.attach(8);
myservo9.attach(9);
myservo10.attach(10);
myservo11.attach(11);
myservo12.attach(12);
myservo13.attach(13);

}

void loop() {
  intPW2  = random(0,89); //clockwise
//  intPW3  = random(0,89); //clockwise
  intPW4  = random(0,89); //clockwise
  intPW5  = random(0,89); //clockwise
  intPW6  = random(0,89); //clockwise
  intPW7  = random(0,89); //counterclockwise
  intPW8  = random(0,89); //counterclockwise
  intPW9  = random(90,180); //counterclockwise
  intPW10 = random(90,180); //counterclockwise
  intPW11 = random(90,180); //counterclockwise
  intPW12 = random(90,180); //counterclockwise
  intPW13 = random(90,180); //counterclockwise

  myservo2.write(intPW2);
  myservo4.write(intPW4);
  myservo5.write(intPW5);
  myservo6.write(intPW6);
  myservo7.write(intPW7);
  myservo8.write(intPW8);
  myservo9.write(intPW9);
  myservo10.write(intPW10);
  myservo11.write(intPW11);
  myservo12.write(intPW12);
  myservo13.write(intPW13);
  delay(500); // random speed for 500ms
  //Serial.println(dt);
  //Serial.println(intSpeed);
  myservo2.write(stopPW);
//  myservo3.write(stopPW);
  myservo4.write(stopPW);
  myservo5.write(stopPW);
  myservo6.write(stopPW);
  myservo7.write(stopPW);
  myservo8.write(stopPW);
  myservo9.write(stopPW);
  myservo10.write(stopPW);
  myservo11.write(stopPW);
  myservo12.write(stopPW);
  myservo13.write(stopPW);
  delay(5000);// stop for 5000ms
  myservo2.write(gaitSpeedCC); //counterclockwise
  myservo4.write(gaitSpeedCC); //counterclockwise
  myservo5.write(gaitSpeedCC);
  myservo6.write(gaitSpeedCC); //counterclockwise
  myservo7.write(gaitSpeedCC);
  myservo8.write(gaitSpeedCC); //clockwise
  myservo9.write(gaitSpeedC);
  myservo10.write(gaitSpeedC); //clockwise
  myservo11.write(gaitSpeedC);
  myservo12.write(gaitSpeedC); //clockwise
  myservo13.write(gaitSpeedC);
  delay(31000); //spin for 20000ms
  }
