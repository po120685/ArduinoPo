#include <Wire.h>
#include <Dynamixel2Arduino.h>

#if defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7,8);
  #define DXL_SERIAL Serial
  #define DEBUG_SERIAL soft_serial
  const int DXL_DIR_PIN = 2;
#endif

const uint8_t DXL_ID1  = 1; 
const uint8_t DXL_ID2  = 2;
const uint8_t DXL_ID3  = 3;

const float DXL_PROTOCOL_VERSION = 2.0; 

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

using namespace ControlTableItem;

void setup(void) {
  DEBUG_SERIAL.begin(9600);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  dxl.torqueOff(DXL_ID1);
  dxl.torqueOff(DXL_ID2);
  dxl.torqueOff(DXL_ID3);

  dxl.setOperatingMode(DXL_ID1, OP_POSITION);
  dxl.setOperatingMode(DXL_ID2, OP_POSITION);
  dxl.setOperatingMode(DXL_ID3, OP_POSITION);

  dxl.torqueOn(DXL_ID1);
  dxl.torqueOn(DXL_ID2);
  dxl.torqueOn(DXL_ID3);

  dxl.setGoalPosition(DXL_ID1, 3500);  // replace with your calibrated value
  delay(1000);

}

void loop() {
  //0-4095 position mapping
  //millis()/100.0 time variable - bigger means slower motion 
  // 1000 * sin... scales the wave - how much it ossiclates in between
  // 2000 + ... shifts the wave back and forth
  int drive = 3000 + 500 * sin(millis()/250.0);

 //reads position of servo1 and position of servo2
  int pos1 = dxl.getPresentPosition(DXL_ID1); 
  int pos2 = dxl.getPresentPosition(DXL_ID2);

  //off set between the servos in position
  int offset = 0;

  // leader
  dxl.setGoalPosition(DXL_ID1, drive);

  // followers
  dxl.setGoalPosition(DXL_ID2, pos1 + offset);
  dxl.setGoalPosition(DXL_ID3, pos2 + offset);

  delay(20);
}
