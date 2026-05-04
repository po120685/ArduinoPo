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

  dxl.setOperatingMode(DXL_ID1, OP_VELOCITY);
  dxl.setOperatingMode(DXL_ID2, OP_VELOCITY);
  dxl.setOperatingMode(DXL_ID3, OP_VELOCITY);

  dxl.torqueOn(DXL_ID1);
  dxl.torqueOn(DXL_ID2);
  dxl.torqueOn(DXL_ID3);
}

void loop() {
  dxl.setGoalVelocity(DXL_ID1, -100);
  dxl.setGoalVelocity(DXL_ID2, 50);
  dxl.setGoalVelocity(DXL_ID3, 200);
}