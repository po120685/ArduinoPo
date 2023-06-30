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

const uint8_t DXL_ID1   = 1; 
const uint8_t DXL_ID2   = 2;
const uint8_t DXL_ID3   = 3;
const uint8_t DXL_ID4   = 4; 
const uint8_t DXL_ID5   = 5;
const uint8_t DXL_ID6   = 6;
const uint8_t DXL_ID7   = 7; 
const uint8_t DXL_ID8   = 8;
const uint8_t DXL_ID9   = 9;
const uint8_t DXL_ID10  = 10; 
const uint8_t DXL_ID11  = 11;
const uint8_t DXL_ID12  = 12;
const uint8_t DXL_ID13  = 13; 
const uint8_t DXL_ID14  = 14;
const uint8_t DXL_ID15  = 15;
const uint8_t DXL_ID16  = 16; 
const uint8_t DXL_ID17  = 17;
const uint8_t DXL_ID18  = 18;
const uint8_t DXL_ID19  = 19;
const uint8_t DXL_ID20  = 20;
const float DXL_PROTOCOL_VERSION = 2.0; 

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

using namespace ControlTableItem;

//LED Pin Number
int LEDPin = 13;

//set variables for random initial positions 
int randNum1; 
int randNum2; 
int randNum3;
int randNum4; 
int randNum5; 
int randNum6;
int randNum7; 
int randNum8; 
int randNum9;
int randNum10; 
int randNum11; 
int randNum12;
int randNum13; 
int randNum14; 
int randNum15;
int randNum16; 
int randNum17; 
int randNum18;
int randNum19; 
int randNum20;

//set velocity 
int vel = 200;

 void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);

  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl.ping(DXL_ID1);
  dxl.ping(DXL_ID2);
  dxl.ping(DXL_ID3);
  dxl.ping(DXL_ID4);
  dxl.ping(DXL_ID5);
  dxl.ping(DXL_ID6);
  dxl.ping(DXL_ID7);
  dxl.ping(DXL_ID8);
  dxl.ping(DXL_ID9);
  dxl.ping(DXL_ID10);
  dxl.ping(DXL_ID11);
  dxl.ping(DXL_ID12);
  dxl.ping(DXL_ID13);
  dxl.ping(DXL_ID14);
  dxl.ping(DXL_ID15);
  dxl.ping(DXL_ID16);
  dxl.ping(DXL_ID17);
  dxl.ping(DXL_ID18);
  dxl.ping(DXL_ID19);
  dxl.ping(DXL_ID20);

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
    file.println("time, xAcc, yAcc, zAcc, xRot, yRot, zRot, temp, servo1Pos, servo2Pos, servo3Pos, servo4Pos, servo5Pos, servo6Pos, servo7Pos, servo8Pos, servo9Pos, servo10Pos, servo11Pos, servo12Pos, servo13Pos, servo14Pos, servo15Pos, servo16Pos, servo17Pos, servo18Pos, servo19Pos, servo20Pos"); //set file header
     
    randNum1  = random(0,360); //generate random position numbers
    randNum2  = random(0,360);
    randNum3  = random(0,360);
    randNum4  = random(0,360); //generate random position numbers
    randNum5  = random(0,360);
    randNum6  = random(0,360);
    randNum7  = random(0,360); //generate random position numbers
    randNum8  = random(0,360);
    randNum9  = random(0,360);
    randNum10 = random(0,360); //generate random position numbers
    randNum11 = random(0,360);
    randNum12 = random(0,360);
    randNum13 = random(0,360); //generate random position numbers
    randNum14 = random(0,360);
    randNum15 = random(0,360);
    randNum16 = random(0,360); //generate random position numbers
    randNum17 = random(0,360);
    randNum18 = random(0,360);
    randNum19 = random(0,360); //generate random position numbers
    randNum20 = random(0,360);
    delay(500);

    dxl.torqueOff(DXL_ID1); //Initiate servo motors for velocity
    dxl.torqueOff(DXL_ID2);
    dxl.torqueOff(DXL_ID3);
    dxl.torqueOff(DXL_ID4); //Initiate servo motors for velocity
    dxl.torqueOff(DXL_ID5);
    dxl.torqueOff(DXL_ID6);
    dxl.torqueOff(DXL_ID7); //Initiate servo motors for velocity
    dxl.torqueOff(DXL_ID8);
    dxl.torqueOff(DXL_ID9);
    dxl.torqueOff(DXL_ID10); //Initiate servo motors for velocity
    dxl.torqueOff(DXL_ID11);
    dxl.torqueOff(DXL_ID12);
    dxl.torqueOff(DXL_ID13); //Initiate servo motors for velocity
    dxl.torqueOff(DXL_ID14);
    dxl.torqueOff(DXL_ID15);
    dxl.torqueOff(DXL_ID16); //Initiate servo motors for velocity
    dxl.torqueOff(DXL_ID17);
    dxl.torqueOff(DXL_ID18);
    dxl.torqueOff(DXL_ID19); //Initiate servo motors for velocity
    dxl.torqueOff(DXL_ID20);
    dxl.setOperatingMode(DXL_ID1, OP_POSITION); //Initiate servo motors for position
    dxl.setOperatingMode(DXL_ID2, OP_POSITION);
    dxl.setOperatingMode(DXL_ID3, OP_POSITION);
    dxl.setOperatingMode(DXL_ID4, OP_POSITION); //Initiate servo motors for position
    dxl.setOperatingMode(DXL_ID5, OP_POSITION);
    dxl.setOperatingMode(DXL_ID6, OP_POSITION);
    dxl.setOperatingMode(DXL_ID7, OP_POSITION); //Initiate servo motors for position
    dxl.setOperatingMode(DXL_ID8, OP_POSITION);
    dxl.setOperatingMode(DXL_ID9, OP_POSITION);
    dxl.setOperatingMode(DXL_ID10, OP_POSITION); //Initiate servo motors for position
    dxl.setOperatingMode(DXL_ID11, OP_POSITION);
    dxl.setOperatingMode(DXL_ID12, OP_POSITION);
    dxl.setOperatingMode(DXL_ID13, OP_POSITION); //Initiate servo motors for position
    dxl.setOperatingMode(DXL_ID14, OP_POSITION);
    dxl.setOperatingMode(DXL_ID15, OP_POSITION);
    dxl.setOperatingMode(DXL_ID16, OP_POSITION); //Initiate servo motors for position
    dxl.setOperatingMode(DXL_ID17, OP_POSITION);
    dxl.setOperatingMode(DXL_ID18, OP_POSITION);
    dxl.setOperatingMode(DXL_ID19, OP_POSITION); //Initiate servo motors for position
    dxl.setOperatingMode(DXL_ID20, OP_POSITION);
    dxl.torqueOn(DXL_ID1);
    dxl.torqueOn(DXL_ID2);
    dxl.torqueOn(DXL_ID3);
    dxl.torqueOn(DXL_ID4);
    dxl.torqueOn(DXL_ID5);
    dxl.torqueOn(DXL_ID6);
    dxl.torqueOn(DXL_ID7);
    dxl.torqueOn(DXL_ID8);
    dxl.torqueOn(DXL_ID9);
    dxl.torqueOn(DXL_ID10);
    dxl.torqueOn(DXL_ID11);
    dxl.torqueOn(DXL_ID12);
    dxl.torqueOn(DXL_ID13);
    dxl.torqueOn(DXL_ID14);
    dxl.torqueOn(DXL_ID15);
    dxl.torqueOn(DXL_ID16);
    dxl.torqueOn(DXL_ID17);
    dxl.torqueOn(DXL_ID18);
    dxl.torqueOn(DXL_ID19);
    dxl.torqueOn(DXL_ID20);
    

    dxl.setGoalPosition(DXL_ID1,randNum1,UNIT_DEGREE); //Set Servos to random initial positions
    dxl.setGoalPosition(DXL_ID2,randNum2,UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID3,randNum3,UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID4,randNum4,UNIT_DEGREE); //Set Servos to random initial positions
    dxl.setGoalPosition(DXL_ID5,randNum5,UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID6,randNum6,UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID7,randNum7,UNIT_DEGREE); //Set Servos to random initial positions
    dxl.setGoalPosition(DXL_ID8,randNum8,UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID9,randNum9,UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID10,randNum10,UNIT_DEGREE); //Set Servos to random initial positions
    dxl.setGoalPosition(DXL_ID11,randNum11,UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID12,randNum12,UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID13,randNum13,UNIT_DEGREE); //Set Servos to random initial positions
    dxl.setGoalPosition(DXL_ID14,randNum14,UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID15,randNum15,UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID16,randNum16,UNIT_DEGREE); //Set Servos to random initial positions
    dxl.setGoalPosition(DXL_ID17,randNum17,UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID18,randNum18,UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID19,randNum19,UNIT_DEGREE); //Set Servos to random initial positions
    dxl.setGoalPosition(DXL_ID20,randNum20,UNIT_DEGREE);
    delay(3000);

    

    dxl.torqueOff(DXL_ID1); //Initiate servo motors for velocity
    dxl.torqueOff(DXL_ID2);
    dxl.torqueOff(DXL_ID3);
    dxl.torqueOff(DXL_ID4); //Initiate servo motors for velocity
    dxl.torqueOff(DXL_ID5);
    dxl.torqueOff(DXL_ID6);
    dxl.torqueOff(DXL_ID7); //Initiate servo motors for velocity
    dxl.torqueOff(DXL_ID8);
    dxl.torqueOff(DXL_ID9);
    dxl.torqueOff(DXL_ID10); //Initiate servo motors for velocity
    dxl.torqueOff(DXL_ID11);
    dxl.torqueOff(DXL_ID12);
    dxl.torqueOff(DXL_ID13); //Initiate servo motors for velocity
    dxl.torqueOff(DXL_ID14);
    dxl.torqueOff(DXL_ID15);
    dxl.torqueOff(DXL_ID16); //Initiate servo motors for velocity
    dxl.torqueOff(DXL_ID17);
    dxl.torqueOff(DXL_ID18);
    dxl.torqueOff(DXL_ID19); //Initiate servo motors for velocity
    dxl.torqueOff(DXL_ID20);
    dxl.setOperatingMode(DXL_ID1, OP_VELOCITY);
    dxl.setOperatingMode(DXL_ID2, OP_VELOCITY);
    dxl.setOperatingMode(DXL_ID3, OP_VELOCITY);
    dxl.setOperatingMode(DXL_ID4, OP_VELOCITY);
    dxl.setOperatingMode(DXL_ID5, OP_VELOCITY);
    dxl.setOperatingMode(DXL_ID6, OP_VELOCITY);
    dxl.setOperatingMode(DXL_ID7, OP_VELOCITY);
    dxl.setOperatingMode(DXL_ID8, OP_VELOCITY);
    dxl.setOperatingMode(DXL_ID9, OP_VELOCITY);
    dxl.setOperatingMode(DXL_ID10, OP_VELOCITY);
    dxl.setOperatingMode(DXL_ID11, OP_VELOCITY);
    dxl.setOperatingMode(DXL_ID12, OP_VELOCITY);
    dxl.setOperatingMode(DXL_ID13, OP_VELOCITY);
    dxl.setOperatingMode(DXL_ID14, OP_VELOCITY);
    dxl.setOperatingMode(DXL_ID15, OP_VELOCITY);
    dxl.setOperatingMode(DXL_ID16, OP_VELOCITY);
    dxl.setOperatingMode(DXL_ID17, OP_VELOCITY);
    dxl.setOperatingMode(DXL_ID18, OP_VELOCITY);
    dxl.setOperatingMode(DXL_ID19, OP_VELOCITY);
    dxl.setOperatingMode(DXL_ID20, OP_VELOCITY);
    
    dxl.torqueOn(DXL_ID1);
    dxl.torqueOn(DXL_ID2);
    dxl.torqueOn(DXL_ID3);
    dxl.torqueOn(DXL_ID4);
    dxl.torqueOn(DXL_ID5);
    dxl.torqueOn(DXL_ID6);
    dxl.torqueOn(DXL_ID7);
    dxl.torqueOn(DXL_ID8);
    dxl.torqueOn(DXL_ID9);
    dxl.torqueOn(DXL_ID10);
    dxl.torqueOn(DXL_ID11);
    dxl.torqueOn(DXL_ID12);
    dxl.torqueOn(DXL_ID13);
    dxl.torqueOn(DXL_ID14);
    dxl.torqueOn(DXL_ID15);
    dxl.torqueOn(DXL_ID16);
    dxl.torqueOn(DXL_ID17);
    dxl.torqueOn(DXL_ID18);
    dxl.torqueOn(DXL_ID19);
    dxl.torqueOn(DXL_ID20);
    if (file) {
      while (pushbutton==0) { //recording and writing loop start here
      
      dxl.setGoalVelocity(DXL_ID1,  vel);
      dxl.setGoalVelocity(DXL_ID2,  vel);
      dxl.setGoalVelocity(DXL_ID3,  vel);
      dxl.setGoalVelocity(DXL_ID4,  vel);
      dxl.setGoalVelocity(DXL_ID5,  vel);
      dxl.setGoalVelocity(DXL_ID6,  vel);
      dxl.setGoalVelocity(DXL_ID7,  vel);
      dxl.setGoalVelocity(DXL_ID8,  vel);
      dxl.setGoalVelocity(DXL_ID9,  vel);
      dxl.setGoalVelocity(DXL_ID10, vel);
      dxl.setGoalVelocity(DXL_ID11, vel);
      dxl.setGoalVelocity(DXL_ID12, vel);
      dxl.setGoalVelocity(DXL_ID13, vel);
      dxl.setGoalVelocity(DXL_ID14, vel);
      dxl.setGoalVelocity(DXL_ID15, vel);
      dxl.setGoalVelocity(DXL_ID16, vel);
      dxl.setGoalVelocity(DXL_ID17, vel);
      dxl.setGoalVelocity(DXL_ID18, vel);
      dxl.setGoalVelocity(DXL_ID19, vel);
      dxl.setGoalVelocity(DXL_ID20, vel);
      

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
      file.print(dxl.getPresentPosition(DXL_ID3, UNIT_DEGREE));
      file.print(",");
      file.print(dxl.getPresentPosition(DXL_ID4, UNIT_DEGREE)); //need to take modulous 360. rawValue/360, remainder is the modulous
      file.print(",");
      file.print(dxl.getPresentPosition(DXL_ID5, UNIT_DEGREE));
      file.print(",");
      file.print(dxl.getPresentPosition(DXL_ID6, UNIT_DEGREE));
      file.print(",");
      file.print(dxl.getPresentPosition(DXL_ID7, UNIT_DEGREE)); //need to take modulous 360. rawValue/360, remainder is the modulous
      file.print(",");
      file.print(dxl.getPresentPosition(DXL_ID8, UNIT_DEGREE));
      file.print(",");
      file.print(dxl.getPresentPosition(DXL_ID9, UNIT_DEGREE)); //need to take modulous 360. rawValue/360, remainder is the modulous
      file.print(",");
      file.print(dxl.getPresentPosition(DXL_ID10, UNIT_DEGREE));
      file.print(",");
      file.print(dxl.getPresentPosition(DXL_ID11, UNIT_DEGREE));
      file.print(",");
      file.print(dxl.getPresentPosition(DXL_ID12, UNIT_DEGREE)); //need to take modulous 360. rawValue/360, remainder is the modulous
      file.print(",");
      file.print(dxl.getPresentPosition(DXL_ID13, UNIT_DEGREE));
      file.print(",");
      file.print(dxl.getPresentPosition(DXL_ID14, UNIT_DEGREE));
      file.print(",");
      file.print(dxl.getPresentPosition(DXL_ID15, UNIT_DEGREE)); //need to take modulous 360. rawValue/360, remainder is the modulous
      file.print(",");
      file.print(dxl.getPresentPosition(DXL_ID16, UNIT_DEGREE));
      file.print(",");
      file.print(dxl.getPresentPosition(DXL_ID17, UNIT_DEGREE));
      file.print(",");
      file.print(dxl.getPresentPosition(DXL_ID18, UNIT_DEGREE));
      file.print(",");
      file.print(dxl.getPresentPosition(DXL_ID19, UNIT_DEGREE));
      file.print(",");
      file.println(dxl.getPresentPosition(DXL_ID20, UNIT_DEGREE));
 
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
dxl.torqueOff(DXL_ID4);
dxl.torqueOff(DXL_ID5);
dxl.torqueOff(DXL_ID6);
dxl.torqueOff(DXL_ID7);
dxl.torqueOff(DXL_ID8);
dxl.torqueOff(DXL_ID9);
dxl.torqueOff(DXL_ID10);
dxl.torqueOff(DXL_ID11);
dxl.torqueOff(DXL_ID12);
dxl.torqueOff(DXL_ID13);
dxl.torqueOff(DXL_ID14);
dxl.torqueOff(DXL_ID15);
dxl.torqueOff(DXL_ID16);
dxl.torqueOff(DXL_ID17);
dxl.torqueOff(DXL_ID18);
dxl.torqueOff(DXL_ID19);
dxl.torqueOff(DXL_ID20);
digitalWrite(LEDPin,LOW);
}

buttonStatePrev = buttonStateNow;

delay(100);

}