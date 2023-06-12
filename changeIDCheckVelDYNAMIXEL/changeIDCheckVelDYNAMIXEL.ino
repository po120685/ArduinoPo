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

//const uint8_t DXL_ID = 100; 
const uint8_t DXL_ID = 1; 
const float DXL_PROTOCOL_VERSION = 2.0; 

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

using namespace ControlTableItem;




void setup() {
  // put your setup code here, to run once:
  uint8_t present_id = DXL_ID;
  uint8_t new_id = 2;

  DEBUG_SERIAL.begin(115200); 

  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl.ping(DXL_ID);

  dxl.torqueOff(DXL_ID);
  new_id = 2;
  new_id = DXL_ID;

  dxl.setOperatingMode(DXL_ID, OP_VELOCITY);
  dxl.torqueOn(DXL_ID);


}

void loop() {
  // put your main code here, to run repeatedly:
}