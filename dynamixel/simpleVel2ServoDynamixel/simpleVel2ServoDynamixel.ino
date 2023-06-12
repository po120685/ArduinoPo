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

const uint8_t DXL_ID     = 1; 
const uint8_t DXL_IDSec  = 2;
const uint8_t DXL_IDThir = 3;
const float DXL_PROTOCOL_VERSION = 2.0; 

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

using namespace ControlTableItem;


void setup() {
  // put your setup code here, to run once:
  DEBUG_SERIAL.begin(115200); 

  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl.ping(DXL_ID);
  dxl.ping(DXL_IDSec);
  dxl.ping(DXL_IDThir);

  dxl.torqueOff(DXL_ID);
  dxl.torqueOff(DXL_IDSec);
  dxl.torqueOff(DXL_IDThir);
  dxl.setOperatingMode(DXL_ID, OP_VELOCITY);
  dxl.setOperatingMode(DXL_IDSec, OP_VELOCITY);
  dxl.setOperatingMode(DXL_IDThir, OP_VELOCITY);
  dxl.torqueOn(DXL_ID);
  dxl.torqueOn(DXL_IDSec);
  dxl.torqueOn(DXL_IDThir);


}

void loop() {
  // put your main code here, to run repeatedly:
  dxl.setGoalVelocity(DXL_ID, 50);
  dxl.setGoalVelocity(DXL_IDSec, 100);
  dxl.setGoalVelocity(DXL_IDThir, 200);
  //delay(1000);
  //Serial.print(dxl.getPresentPosition(DXL_ID)); //use default encoder value
  DEBUG_SERIAL.print(dxl.getPresentPosition(DXL_ID, UNIT_DEGREE)); //use angle in degree
  DEBUG_SERIAL.println(" ");
  delay(100);
}
