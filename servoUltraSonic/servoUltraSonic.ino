#include<Servo.h>
int trig=8;
int echo=9;
int dt=10;
Servo servo;

//int distance,duration;
void setup() {
  // put your setup code here, to run once:
pinMode(trig,OUTPUT);
pinMode(echo,INPUT);
Serial.begin(9600);
servo.attach(11);
}

void loop() {
  // put your main code here, to run repeatedly:

 
if (calc_dis()<10)
{
  for (int i=0;i<=100;i++)
  {
    servo.write(i);
    delay(1);
  }
  delay(100);
  for (int i=100;i>=0;i--)
  {
    servo.write(i);
    delay(1);
  }
  delay(100);
}
}

//This code is written to calculate the DISTANCE using ULTRASONIC SENSOR

int calc_dis()
{
  int duration,distance;
  digitalWrite(trig,HIGH);
  delay(dt);
  digitalWrite(trig,LOW);
  duration=pulseIn(echo,HIGH);
  distance = (duration/2) / 29.1;
  return distance;
}



//#include<Servo.h>
//int trig = 8;
//int echo = 9;
//Servo servo;
//
////int distance,duration;
//void setup() {
//  // put your setup code here, to run once:
//
//  pinMode(trig,OUTPUT);
//  pinMode(echo,INPUT);
//  Serial.begin(9600);
//  servo.attach(11);
//
//}
//
//void loop() {
//  // put your main code here, to run repeatedly:
//
//  if (calc_dis()<5)
//    {
//      for (int i=500;i<=0;i++)
//      {
//      servo.write(i);
//      delay(0.5);
//      }
//      delay(0.5);
//      for (int i =500; i>=0;i--)
//      {
//        servo.write(i);
//        delay(0.5);
//      }
//      delay(0.5);
//    }
//
//}
//
//
//int calc_dis() 
//{
//  int duration, distance;
//  digitalWrite(trig,HIGH);
//  delay(0.5);
//  digitalWrite(trig,LOW);
//  duration=pulseIn(echo,HIGH);
//  distance = duration;
//  return distance;
//}
