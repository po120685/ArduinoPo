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
const uint8_t DXL_ID_1 = 1;    // body joint 1 (head)
const uint8_t DXL_ID_2 = 7;    // body joint 2 (tail)
const uint8_t DXL_ID_8 = 10;    // right front fin
const uint8_t DXL_ID_11 = 11;  // left front fin
const uint8_t ALL_MOTOR_IDS[4] = { DXL_ID_1, DXL_ID_2, DXL_ID_8, DXL_ID_11 };
const uint32_t BAUDRATE = 115200;

// =========================
// Base motion parameters
// =========================
const int CENTER_POS_1 = 692;
const int CENTER_POS_2 = 508;
const int MOVING_SPEED = 300;   // 0-1023; lower = smoother/slower hardware 100
                                 // interpolation, higher = snappier but more
                                 // prone to looking choppy.

// =========================
// Front fin parameters (degrees -- converted to ticks automatically)
// ---------------------------------------------------------
// Motor 8 (right fin):  140deg = pointed laterally (out of the way)
//                       100deg = fins just touching
//                        60deg = directly below body (full plant)
// Motor 11 (left fin):   56deg = pointed laterally (out of the way)
//                       100deg = fins just touching
//                       150deg = directly below body (full plant)
// ---------------------------------------------------------
const float FIN8_LIFTED_DEG   = 79;  // right fin, out of the way //140
const float FIN8_PLANTED_DEG  = 140;   // right fin, planted //79
const float FIN11_LIFTED_DEG  = 132;   // left fin, out of the way //56
const float FIN11_PLANTED_DEG = 56;  // left fin, planted //132

const int FIN_MOVING_SPEED = 300;   // separate hardware speed limit for fins 100
const float FIN_EASE_TIME  = 0.45;  // seconds -- kept comfortably under the 0.15
                                     // shortest gait duration (0.35s) so the
                                     // fin plant/lift never gets cut off
const int FIN_ZERO_POS     = 0;     // raw hardware zero, startup/home pose

int degToPos(float deg) {
  // AX-series servos: 0-300 degrees maps linearly to 0-1023 ticks.
  return (int)(deg / 300.0 * 1023.0);
}

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
//
// finRightDeg / finLeftDeg: target angle for the right (8) / left (11)
// fin during this gait. Right fin plants during C-left+S-left (the
// pivot for the rightward stroke); left fin plants during
// C-right+S-right.
// ---------------------------------------------------------
struct Gait {
  const char* name;
  int offset1;
  int offset2;
  float easeTime;
  float duration;
  float finRightDeg;
  float finLeftDeg;
};

Gait GAITS[4] = {
  { "C-right", 200, 180, 0.20, 0.45, FIN8_LIFTED_DEG,  FIN11_PLANTED_DEG }, // 200,  180,  1.20, 2.70,
  { "S-right", -200, 180, 0.20, 0.35, FIN8_LIFTED_DEG,  FIN11_PLANTED_DEG }, //  -200, 180,  1.20, 2.10,
  { "C-left", -200, -180, 0.20, 0.45, FIN8_PLANTED_DEG, FIN11_LIFTED_DEG  }, //  -200, -180,  1.20, 2.70,
  { "S-left",  200, -180, 0.20, 0.35, FIN8_PLANTED_DEG, FIN11_LIFTED_DEG  }, //  200, -180,  1.20, 2.10,
};
const int NUM_GAITS = 4;

// =========================
// Low-level write helpers (mirror write1/2ByteTxOnly from Python)
// =========================
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
float lastCommandedFin8, lastCommandedFin11;
float switchPos1, switchPos2;
float switchFin8, switchFin11;

void setup() {
  DEBUG_SERIAL.begin(57600);

  dxl.begin(BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  for (int i = 0; i < 4; i++) {
    writeByte(ALL_MOTOR_IDS[i], ADDR_TORQUE_ENABLE, 1);
  }
  DEBUG_SERIAL.println("Torque enabled");

  writeWord(DXL_ID_1, ADDR_MOVING_SPEED, MOVING_SPEED);
  writeWord(DXL_ID_2, ADDR_MOVING_SPEED, MOVING_SPEED);
  writeWord(DXL_ID_8, ADDR_MOVING_SPEED, FIN_MOVING_SPEED);
  writeWord(DXL_ID_11, ADDR_MOVING_SPEED, FIN_MOVING_SPEED);
  DEBUG_SERIAL.println("Moving speed set");

  writeWord(DXL_ID_1, ADDR_GOAL_POSITION, CENTER_POS_1);
  writeWord(DXL_ID_2, ADDR_GOAL_POSITION, CENTER_POS_2);
  writeWord(DXL_ID_8, ADDR_GOAL_POSITION, FIN_ZERO_POS);
  writeWord(DXL_ID_11, ADDR_GOAL_POSITION, FIN_ZERO_POS);
  delay(2000);

  DEBUG_SERIAL.println("Starting animal-like gait sequence");

  gaitIndex = 0;
  gaitStartTime = millis();
  lastCommandedPos1 = CENTER_POS_1;
  lastCommandedPos2 = CENTER_POS_2;
  lastCommandedFin8 = FIN_ZERO_POS;
  lastCommandedFin11 = FIN_ZERO_POS;
  switchPos1 = lastCommandedPos1;
  switchPos2 = lastCommandedPos2;
  switchFin8 = lastCommandedFin8;
  switchFin11 = lastCommandedFin11;
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
    switchFin8 = lastCommandedFin8;
    switchFin11 = lastCommandedFin11;
  }

  // Body: single smoothstep ease directly from the actual current
  // position to the gait's final target -- no double-layered easing
  float target1 = CENTER_POS_1 + gait.offset1;
  float target2 = CENTER_POS_2 + gait.offset2;
  float eBody = smoothstep(elapsedInGait / gait.easeTime);
  float pos1 = lerp(switchPos1, target1, eBody);
  float pos2 = lerp(switchPos2, target2, eBody);

  // Fins: eased over FIN_EASE_TIME so they plant/lift decisively
  float targetFin8 = degToPos(gait.finRightDeg);
  float targetFin11 = degToPos(gait.finLeftDeg);
  float eFin = smoothstep(elapsedInGait / FIN_EASE_TIME);
  float fin8 = lerp(switchFin8, targetFin8, eFin);
  float fin11 = lerp(switchFin11, targetFin11, eFin);

  lastCommandedPos1 = pos1;
  lastCommandedPos2 = pos2;
  lastCommandedFin8 = fin8;
  lastCommandedFin11 = fin11;

  writeWord(DXL_ID_1, ADDR_GOAL_POSITION, (int)pos1);
  writeWord(DXL_ID_2, ADDR_GOAL_POSITION, (int)pos2);
  writeWord(DXL_ID_8, ADDR_GOAL_POSITION, (int)fin8);
  writeWord(DXL_ID_11, ADDR_GOAL_POSITION, (int)fin11);

  // Uncomment to watch the state machine while tuning:
  // DEBUG_SERIAL.print(gait.name);
  // DEBUG_SERIAL.print(" pos1="); DEBUG_SERIAL.print((int)pos1);
  // DEBUG_SERIAL.print(" pos2="); DEBUG_SERIAL.print((int)pos2);
  // DEBUG_SERIAL.print(" fin8="); DEBUG_SERIAL.print((int)fin8);
  // DEBUG_SERIAL.print(" fin11="); DEBUG_SERIAL.println((int)fin11);

  delay(20);
}
