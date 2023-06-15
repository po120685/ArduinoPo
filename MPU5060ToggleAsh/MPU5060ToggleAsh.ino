//This code uses a push button as a toggle switch to 1) record data from a sensor (MPU6050 accerlerameter/gyroscope and Dynamixel servos) until the button is pushed again to close the file. 
//Each file has a unique number according to the number of clicks
//The name of the file can be changed in line sprintf(fileName,"JUN8_%d.CSV",buttonPressCount++);

//Button
#define button_pin 7 
 int buttonStateNow; 
 int buttonStatePrev=0; 
 int LEDState = 0;
 int buttonPressCount = 1;
 int pushbutton = 0;

//SD
#include <SD.h>
#include <Wire.h>
int chipSelect = 53; // pin4 for UNO pin53 for Mega
File file;
int count;
char fileName[20];

//MPU6050
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

//Dynamixel Servo initiation
#include <Dynamixel2Arduino.h>
#include <actuator.h>

#if defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7,8);
  #define DXL_SERIAL Serial
  #define DEBUG_SERIAL soft_serial
  const int DXL_DIR_PIN = 2;

#endif

const uint8_t DXL_ID1  = 1; 
const uint8_t DXL_ID2  = 2;
const uint8_t DXL_ID3 = 3;
const float DXL_PROTOCOL_VERSION = 2.0; 

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

using namespace ControlTableItem;

//LED Pin Number
int LEDPin = 13;

//set variables for random initial positions 
int randNum1; 
int randNum2; 
int randNum3;

 void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);

  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl.ping(DXL_ID1);
  dxl.ping(DXL_ID2);
  dxl.ping(DXL_ID3);

//Button
pinMode(button_pin, INPUT_PULLUP);

//LED
pinMode(LEDPin, OUTPUT);

//MPU6050
if (!mpu.begin()) {
		Serial.println("Failed to find MPU6050 chip");
		while (1) {
		  delay(10);
		}
	}
	Serial.println("MPU6050 Found!");

	// set accelerometer range to +-8G
	mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

	// set gyro range to +- 500 deg/s
	mpu.setGyroRange(MPU6050_RANGE_500_DEG);

	// set filter bandwidth to 21 Hz
	mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

	delay(100);

  pinMode(chipSelect, OUTPUT); // chip select pin must be set to OUTPUT mode
if (SD.begin(chipSelect)) { //Initialize SD card
  Serial.println("Initialized SD card."); // if return is false, something went wrong
  Serial.println("Current File name: ");
}
else {
  Serial.println("SD card was not initialized.");
}
  delay(1000);
}


void loop() {
  // put your main code here, to run repeatedly:

  //Button read here and print
  buttonStateNow = digitalRead(7);
  //Serial.println(buttonStateNow);


//Button conditions start here
if (buttonStatePrev==1 && buttonStateNow==0 && pushbutton ==0) {
       
    sprintf(fileName,"JUN8_%d.CSV",buttonPressCount++); //Set file name
    delay(100);
    file = SD.open(fileName, FILE_WRITE);
    file.println("time, xAcc, yAcc, zAcc, xRot, yRot, zRot, temp, servo1Pos, servo2Pos, servo3Pos"); //set file header
    
    randNum1 = random(0,360); //generate random position numbers
    randNum2 = random(0,360);
    randNum3 = random(0,360);
    delay(500);

    dxl.torqueOff(DXL_ID1); //Initiate servo motors for velocity
    dxl.torqueOff(DXL_ID2);
    dxl.torqueOff(DXL_ID3);
    dxl.setOperatingMode(DXL_ID1, OP_POSITION); //Initiate servo motors for position
    dxl.setOperatingMode(DXL_ID2, OP_POSITION);
    dxl.setOperatingMode(DXL_ID3, OP_POSITION);
    dxl.torqueOn(DXL_ID1);
    dxl.torqueOn(DXL_ID2);
    dxl.torqueOn(DXL_ID3);

    dxl.setGoalPosition(DXL_ID1,randNum1,UNIT_DEGREE); //Set Servos to random initial positions
    dxl.setGoalPosition(DXL_ID2,randNum2,UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID3,randNum3,UNIT_DEGREE);
    delay(3000);

    

    dxl.torqueOff(DXL_ID1); //Initiate servo motors for velocity
    dxl.torqueOff(DXL_ID2);
    dxl.torqueOff(DXL_ID3);
    dxl.setOperatingMode(DXL_ID1, OP_VELOCITY);
    dxl.setOperatingMode(DXL_ID2, OP_VELOCITY);
    dxl.setOperatingMode(DXL_ID3, OP_VELOCITY);
    dxl.torqueOn(DXL_ID1);
    dxl.torqueOn(DXL_ID2);
    dxl.torqueOn(DXL_ID3);
    if (file) {
      while (pushbutton==0) { //recording and writing loop start here
      
      dxl.setGoalVelocity(DXL_ID1, 50);
      dxl.setGoalVelocity(DXL_ID2, 100);
      dxl.setGoalVelocity(DXL_ID3, 200);

      sensors_event_t a, g, temp; //Get MPU5060 Events
      mpu.getEvent(&a, &g, &temp);
      //Serial.println(a.acceleration.x); //write to file     
      file.print(temp.timestamp);
      file.print(",");
      file.print(a.acceleration.x);
      file.print(",");
      file.print(a.acceleration.y);
      file.print(",");
      file.print(a.acceleration.z); 
      file.print(",");
      file.print(g.gyro.x);
      file.print(",");
      file.print(g.gyro.y);
      file.print(",");
      file.print(g.gyro.z);
      file.print(",");
      file.print(temp.temperature);
      file.print(",");
      file.print(dxl.getPresentPosition(DXL_ID1, UNIT_DEGREE)); //need to take modulous 360. rawValue/360, remainder is the modulous
      file.print(",");
      file.print(dxl.getPresentPosition(DXL_ID2, UNIT_DEGREE));
      file.print(",");
      file.println(dxl.getPresentPosition(DXL_ID3, UNIT_DEGREE));
 
      //Serial.println(fileName); 

      digitalWrite(LEDPin,HIGH);

      delay(100);

      buttonStateNow = digitalRead(button_pin);
      if (buttonStateNow == 0) {
        pushbutton = 1;
        break;
      }
      }
     } else {
      //Serial.println("Could not open File (writing)");
    }

} else { //close file and turn off Servo Motors
pushbutton = 0;
file.close();
//Serial.println("File Closed");
//Serial.println("Do nothing");
dxl.torqueOff(DXL_ID1);
dxl.torqueOff(DXL_ID2);
dxl.torqueOff(DXL_ID3);
digitalWrite(LEDPin,LOW);
}

buttonStatePrev = buttonStateNow;

delay(100);

}