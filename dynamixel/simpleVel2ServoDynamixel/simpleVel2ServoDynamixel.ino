#include <Dynamixel2Arduino.h>
#include <actuator.h>

#include <Dynamixel2Arduino.h>

#if defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7,8);
  #define DXL_SERIAL Serial
  #define DEBUG_SERIAL soft_serial
  const int DXL_DIR_PIN = 2;

#endif

const uint8_t DXL_ID   = 1; 
const uint8_t DXL_ID2  = 2;
const uint8_t DXL_ID3  = 3;
const uint8_t DXL_ID4  = 4; 
const uint8_t DXL_ID5  = 5;
const uint8_t DXL_ID6  = 6;
const uint8_t DXL_ID7  = 7; 
const uint8_t DXL_ID8  = 8;
const uint8_t DXL_ID9  = 9;
const uint8_t DXL_ID10 = 10; 
const uint8_t DXL_ID11 = 11;
const uint8_t DXL_ID12 = 12;
const uint8_t DXL_ID13 = 13; 
const uint8_t DXL_ID14 = 14;
const uint8_t DXL_ID15 = 15;
const uint8_t DXL_ID16 = 16; 
const uint8_t DXL_ID17 = 17;
const uint8_t DXL_ID18 = 18;
const uint8_t DXL_ID19 = 19; 
const uint8_t DXL_ID20 = 20;

const float DXL_PROTOCOL_VERSION = 2.0; 

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

using namespace ControlTableItem;

int randNum;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); 

  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl.ping(DXL_ID);
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

  dxl.torqueOff(DXL_ID);
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

  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.setOperatingMode(DXL_ID2, OP_POSITION);
  dxl.setOperatingMode(DXL_ID3, OP_POSITION);
  dxl.setOperatingMode(DXL_ID4, OP_POSITION);
  dxl.setOperatingMode(DXL_ID5, OP_POSITION);
  dxl.setOperatingMode(DXL_ID6, OP_POSITION);
  dxl.setOperatingMode(DXL_ID7, OP_POSITION);
  dxl.setOperatingMode(DXL_ID8, OP_POSITION);
  dxl.setOperatingMode(DXL_ID9, OP_POSITION);
  dxl.setOperatingMode(DXL_ID10, OP_POSITION);
  dxl.setOperatingMode(DXL_ID11, OP_POSITION);
  dxl.setOperatingMode(DXL_ID12, OP_POSITION);
  dxl.setOperatingMode(DXL_ID13, OP_POSITION);
  dxl.setOperatingMode(DXL_ID14, OP_POSITION);
  dxl.setOperatingMode(DXL_ID15, OP_POSITION);
  dxl.setOperatingMode(DXL_ID16, OP_POSITION);
  dxl.setOperatingMode(DXL_ID17, OP_POSITION);
  dxl.setOperatingMode(DXL_ID18, OP_POSITION);
  dxl.setOperatingMode(DXL_ID19, OP_POSITION);
  dxl.setOperatingMode(DXL_ID20, OP_POSITION);
  dxl.torqueOn(DXL_ID);
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

  //dxl.torqueOff(DXL_ID);
  //dxl.torqueOff(DXL_IDSec);
  //dxl.torqueOff(DXL_IDThir);
  //dxl.setOperatingMode(DXL_ID, OP_VELOCITY);
  //dxl.setOperatingMode(DXL_IDSec, OP_VELOCITY);
  //dxl.setOperatingMode(DXL_IDThir, OP_VELOCITY);
  //dxl.torqueOn(DXL_ID);
  //dxl.torqueOn(DXL_IDSec);
  //dxl.torqueOn(DXL_IDThir);


}

void loop() {

  //randNum= random(0,360);
  delay(500);
  //Serial.println(randNum);
  dxl.setGoalPosition(DXL_ID,0,UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID2,0,UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID3,0,UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID4,0,UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID5,0,UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID6,0,UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID7,0,UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID8,0,UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID9,0,UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID10,0,UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID11,0,UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID12,0,UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID13,0,UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID14,0,UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID15,0,UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID16,0,UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID17,0,UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID18,0,UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID19,0,UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID20,0,UNIT_DEGREE);
  
  delay(100);
  // put your main code here, to run repeatedly:
  ////dxl.setGoalVelocity(DXL_ID, 50);
  ////dxl.setGoalVelocity(DXL_IDSec, 100);
  ////dxl.setGoalVelocity(DXL_IDThir, 200);
  //delay(1000);
  //Serial.print(dxl.getPresentPosition(DXL_ID)); //use default encoder value
  //DEBUG_SERIAL.print(dxl.getPresentPosition(DXL_ID, UNIT_DEGREE)); //use angle in degree
  ////DEBUG_SERIAL.println(" ");
  ////delay(100);
}
