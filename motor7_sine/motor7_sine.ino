#include <Dynamixel2Arduino.h>
#include <SoftwareSerial.h>

// DYNAMIXEL Shield on Mega 2560: hardware Serial (pins 0/1) for DXL,
// pin 2 for direction control. Debug prints go out pins 7/8 instead,
// since pins 0/1 are claimed by the shield.
SoftwareSerial soft_serial(7, 8);
#define DXL_SERIAL   Serial
#define DXL_DIR_PIN  2
#define DEBUG_SERIAL soft_serial

const float DXL_PROTOCOL_VERSION = 1.0;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

const uint16_t ADDR_TORQUE_ENABLE = 24;
const uint16_t ADDR_GOAL_POSITION = 30;

const uint8_t DXL_ID = 11;
const uint32_t BAUDRATE = 115200;

const int CENTER_POS = 190;   // motor 7's resting center position
const int AMPLITUDE = 10;     // ticks -- how big the wiggle is
const float FREQUENCY = 0.5;  // Hz -- how fast it wiggles

void writeByte(uint8_t id, uint16_t addr, uint8_t value) {
  dxl.write(id, addr, &value, 1);
}

void writeWord(uint8_t id, uint16_t addr, uint16_t value) {
  uint8_t data[2];
  data[0] = value & 0xFF;
  data[1] = (value >> 8) & 0xFF;
  dxl.write(id, addr, data, 2);
}

void setup() {
  DEBUG_SERIAL.begin(57600);

  dxl.begin(BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  writeByte(DXL_ID, ADDR_TORQUE_ENABLE, 1);
  DEBUG_SERIAL.println("Torque enabled, starting sine motion");

  writeWord(DXL_ID, ADDR_GOAL_POSITION, CENTER_POS);
  delay(1000);
}

void loop() {
  float t = millis() / 1000.0;
  int pos = CENTER_POS + (int)(AMPLITUDE * sin(2 * PI * FREQUENCY * t));

  writeWord(DXL_ID, ADDR_GOAL_POSITION, pos);

  delay(20);
}
