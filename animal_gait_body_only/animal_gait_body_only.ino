#include <Dynamixel2Arduino.h>
#include <SoftwareSerial.h>

// =========================
// Wiring (DYNAMIXEL Shield on Mega 2560)
// ---------------------------------------------------------
// DXL_SERIAL: hardware Serial (pins 0/1) -- used by the shield for
//             DYNAMIXEL half-duplex communication.
// DXL_DIR_PIN 2: flow-control / direction pin for the shield.
// Because hardware Serial is claimed by the shield, normal Serial
// Monitor debug prints can't use it -- soft_serial (pins 7/8) with
// SoftwareSerial stands in for that instead.
// ---------------------------------------------------------
SoftwareSerial soft_serial(7, 8);          // debug RX, TX only
#define DXL_SERIAL   Serial
#define DXL_DIR_PIN  2
#define DEBUG_SERIAL soft_serial

const float DXL_PROTOCOL_VERSION = 1.0;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// =========================
// Control table (same addresses as the Python version)
// =========================
const uint16_t ADDR_TORQUE_ENABLE = 24;
const uint16_t ADDR_GOAL_POSITION = 30;
const uint16_t ADDR_MOVING_SPEED  = 32;

// =========================
// Settings
// =========================
const uint8_t DXL_ID_1 = 1;   // body joint 1 (head)
const uint8_t DXL_ID_2 = 7;   // body joint 2 (tail)
const uint32_t BAUDRATE = 115200;

// =========================
// Base motion parameters
// =========================
const int CENTER_POS_1 = 692;
const int CENTER_POS_2 = 508;
const int MOVING_SPEED = 300;   // 0-1023; lower = smoother/slower hardware
                                 // interpolation, higher = snappier but more
                                 // prone to looking choppy.

// =========================
// Gait sequence
// ---------------------------------------------------------
// Every gait is a single HELD POSE (ease to a target, then stay
// there). offset1 / offset2 are ticks added to CENTER_POS for
// joint 1 (head) and joint 2 (tail).
//
//   C-shape: head and tail bend the SAME direction -> single curve.
//   S-shape: head has already flipped toward the new side while the
//            tail is still finishing the previous bend -> double
//            curve, matching how a real traveling wave leads with
//            the head. That's why in S-right, offset1 is negative
//            (heading toward left/C-left) while offset2 is still
//            positive (tail hasn't caught up from C-right yet).
//
// duration: how long (seconds) this pose is held before the next one
// (must be >= easeTime, or the next transition interrupts it early).
// ---------------------------------------------------------
struct Gait {
  const char* name;
  int offset1;
  int offset2;
  float easeTime;
  float duration;
};

Gait GAITS[4] = {
  { "C-right", 200,  180,  0.20, 0.45 },
  { "S-right", -200, 180,  0.20, 0.35 },
  { "C-left", -200, -180,  0.20, 0.45 },
  { "S-left",  200, -180,  0.20, 0.35 },
};
const int NUM_GAITS = 4;

// =========================
// Low-level write helpers (mirror write1/2ByteTxOnly from Python)
// ---------------------------------------------------------
// dxl.write() defaults to a 100ms timeout waiting for a status-packet
// reply -- effectively blocking like Python's write2ByteTxRx. Passing
// a small explicit timeout here keeps these calls fast/non-blocking,
// closer to the fire-and-forget write2ByteTxOnly behavior we used on
// the Python side.
// ---------------------------------------------------------
void writeByte(uint8_t id, uint16_t addr, uint8_t value) {
  dxl.write(id, addr, &value, 1);
}

void writeWord(uint8_t id, uint16_t addr, uint16_t value) {
  uint8_t data[2];
  data[0] = value & 0xFF;
  data[1] = (value >> 8) & 0xFF;
  dxl.write(id, addr, data, 2);
}
// =========================
// Easing helpers
// =========================
float lerp(float a, float b, float t) {
  return a + (b - a) * t;
}

float smoothstep(float t) {
  if (t < 0.0) t = 0.0;
  if (t > 1.0) t = 1.0;
  return t * t * (3 - 2 * t);
}

// =========================
// State
// =========================
int gaitIndex = 0;
unsigned long gaitStartTime = 0;

float lastCommandedPos1, lastCommandedPos2;
float switchPos1, switchPos2;

void setup() {
  DEBUG_SERIAL.begin(57600);

  dxl.begin(BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  writeByte(DXL_ID_1, ADDR_TORQUE_ENABLE, 1);
  writeByte(DXL_ID_2, ADDR_TORQUE_ENABLE, 1);
  DEBUG_SERIAL.println("Torque enabled");

  writeWord(DXL_ID_1, ADDR_MOVING_SPEED, MOVING_SPEED);
  writeWord(DXL_ID_2, ADDR_MOVING_SPEED, MOVING_SPEED);
  DEBUG_SERIAL.println("Moving speed set");

  writeWord(DXL_ID_1, ADDR_GOAL_POSITION, CENTER_POS_1);
  writeWord(DXL_ID_2, ADDR_GOAL_POSITION, CENTER_POS_2);
  delay(2000);

  DEBUG_SERIAL.println("Starting C/S gait sequence");

  gaitIndex = 0;
  gaitStartTime = millis();
  lastCommandedPos1 = CENTER_POS_1;
  lastCommandedPos2 = CENTER_POS_2;
  switchPos1 = lastCommandedPos1;
  switchPos2 = lastCommandedPos2;
}

void loop() {
  unsigned long now = millis();
  Gait gait = GAITS[gaitIndex];
  float elapsedInGait = (now - gaitStartTime) / 1000.0;

  // Advance to the next gait once this one's duration has elapsed
  if (elapsedInGait > gait.duration) {
    gaitIndex = (gaitIndex + 1) % NUM_GAITS;
    gait = GAITS[gaitIndex];
    gaitStartTime = now;
    elapsedInGait = 0.0;
    switchPos1 = lastCommandedPos1;
    switchPos2 = lastCommandedPos2;
  }

  // Single smoothstep ease directly from the actual current position
  // to the gait's final target -- no double-layered easing
  float target1 = CENTER_POS_1 + gait.offset1;
  float target2 = CENTER_POS_2 + gait.offset2;
  float e = smoothstep(elapsedInGait / gait.easeTime);
  float pos1 = lerp(switchPos1, target1, e);
  float pos2 = lerp(switchPos2, target2, e);

  lastCommandedPos1 = pos1;
  lastCommandedPos2 = pos2;

  writeWord(DXL_ID_1, ADDR_GOAL_POSITION, (int)pos1);
  writeWord(DXL_ID_2, ADDR_GOAL_POSITION, (int)pos2);

  // Uncomment to watch the state machine while tuning:
  // DEBUG_SERIAL.print(gait.name);
  // DEBUG_SERIAL.print(" pos1="); DEBUG_SERIAL.print((int)pos1);
  // DEBUG_SERIAL.print(" pos2="); DEBUG_SERIAL.println((int)pos2);

  delay(20);
}
