#include <Dynamixel2Arduino.h>
//#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TimerOne.h>

// NOTE: deliberately NOT using `using namespace ControlTableItem;` here --
// the library defines its own MOVING_SPEED item, which collides with this
// sketch's own `const int MOVING_SPEED` below. The three items actually
// needed are qualified with ControlTableItem:: at their call site instead.

// =========================
// Wiring (DYNAMIXEL Shield + SD module + MPU6050 on Mega 2560)
// ---------------------------------------------------------
// DXL_SERIAL: hardware Serial (pins 0/1) -- used by the shield for
//             DYNAMIXEL half-duplex communication.
// DXL_DIR_PIN 2: flow-control / direction pin for the shield.
// Because hardware Serial is claimed by the shield, normal Serial
// Monitor debug prints can't use it -- soft_serial (pins 7/8) with
// SoftwareSerial stands in for that instead.
//
// SD card module: hardware SPI (MOSI 51 / MISO 50 / SCK 52), CS on
// pin 4. Note: on the Mega, pin 53 (the native SS pin) must stay an
// OUTPUT for the hardware SPI peripheral to run in master mode, even
// though it isn't used as the SD card's chip-select pin. (An earlier
// version of this logging code set pin 51 instead -- that's MOSI,
// not SS, and doesn't satisfy this requirement.)
//
// MPU6050 IMU: hardware I2C (SDA 20 / SCL 21). AD0 unconnected ->
// address 0x68. INT/XDA/XCL unused.
// ---------------------------------------------------------

//SoftwareSerial soft_serial(7, 8);          // debug RX, TX only
#define DXL_SERIAL   Serial
#define DXL_DIR_PIN  2
//#define DEBUG_SERIAL soft_serial

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
const uint8_t DXL_ID_1  = 1;   // body joint 1 (head)
const uint8_t DXL_ID_2  = 7;   // body joint 2 (tail)
const uint8_t DXL_ID_10  = 10;  // right front fin
const uint8_t DXL_ID_11 = 11;  // left front fin
const uint8_t ALL_MOTOR_IDS[4] = { DXL_ID_1, DXL_ID_2, DXL_ID_10, DXL_ID_11 };
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
// Front fin parameters (degrees -- converted to ticks automatically)
// ---------------------------------------------------------
// Motor 10 (right fin):  79deg = pointed laterally (out of the way)
//                       140deg = directly below body (full plant)
// Motor 11 (left fin):  132deg = pointed laterally (out of the way)
//                        56deg = directly below body (full plant)
// ---------------------------------------------------------
// NOTE: motor IDs 10 and 11 turned out to be physically swapped
// relative to what the gait logic assumes (finRightDeg was reaching
// the physically-left fin and vice versa). Swapping each motor's own
// LIFTED/PLANTED values is mathematically equivalent to swapping which
// motor receives which field, since each motor only ever gets one of
// these two targets -- confirmed correct by observation.
const float FIN10_LIFTED_DEG   = 79;   // (physically) out of the way
const float FIN10_PLANTED_DEG  = 140;  // (physically) planted
const float FIN11_LIFTED_DEG  = 132;  // (physically) out of the way
const float FIN11_PLANTED_DEG = 56;   // (physically) planted

const int FIN_MOVING_SPEED = 300;   // separate hardware speed limit for fins
const float FIN_EASE_TIME  = 0.10;  // seconds -- fins plant/lift decisively
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
//            the head.
//
// duration: how long (seconds) this pose is held before the next one
// (must be >= easeTime, or the next transition interrupts it early).
//
// finRightDeg / finLeftDeg: target angle for the right (10) / left (11)
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
  float finDelay;   // seconds to wait into this gait before the fin
                     // starts easing -- lets the fin plant/lift land
                     // right at the end of the pose instead of the start
};

Gait GAITS[4] = {
  // Fin targets now stay UNCHANGED through each C pose (holding
  // whatever the previous S pose left them at), and only switch to
  // their new target after a short finDelay into the following S pose
  // -- so the lift/plant happens just after the C-hold ends, not
  // during it. Because one fin lifts exactly as the other plants,
  // this delay applies to both at once. //ease time (ramp to position), hold time (how long switching to next position), fin delay (pause)
  { "C-right", 200,  180,  0.20, 0.45, FIN10_PLANTED_DEG, FIN11_LIFTED_DEG,  0.0   },
  { "S-right", -200, 180,  0.20, 0.35, FIN10_LIFTED_DEG,  FIN11_PLANTED_DEG, 0.025 },
  { "C-left", -200, -180,  0.20, 0.45, FIN10_LIFTED_DEG,  FIN11_PLANTED_DEG, 0.0   },
  { "S-left",  200, -180,  0.20, 0.35, FIN10_PLANTED_DEG, FIN11_LIFTED_DEG,  0.025 },
// Speed/timing scaling reference (baseline calibrated at MOVING_SPEED=50)
// MovingSpeed,EaseTime,CDuration,SDuration,FinEaseTime,FinDelay
// 50,1.20,2.70,2.10,0.60,0.150
// 100,0.60,1.35,1.05,0.30,0.075
// 150,0.40,0.90,0.70,0.20,0.050
// 200,0.30,0.675,0.525,0.15,0.0375
// 300,0.20,0.45,0.35,0.10,0.025
};
const int NUM_GAITS = 4;

// =========================
// Low-level write helpers (mirror write1/2ByteTxOnly from Python)
// ---------------------------------------------------------
// Note: an explicit short write timeout was tried here previously and
// made no real difference on this hardware, so it's intentionally
// left out -- these use the library's default.
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
// Camera trigger (BlackFly S sync, external hardware trigger)
// ---------------------------------------------------------
// TimerOne owns Timer1 to fire a jitter-free 100Hz interrupt -- this
// only affects analogWrite() on pins 11/12 (Timer1's PWM outputs),
// which this sketch never uses (servos go over the DXL bus, not
// Arduino PWM). D9 is physically blocked by the DYNAMIXEL shield, so
// D11 is used instead.
//
// Runs as a hardware interrupt, independent of loop()'s blocking DXL
// round trips / SD writes / delay(20) -- the pulse train stays steady
// even while loop() is stalled on something else.
// ---------------------------------------------------------
const int CAMERA_TRIGGER_PIN = 11;
const long CAMERA_FPS = 100;
const long CAMERA_TRIGGER_PERIOD_US = 1000000 / CAMERA_FPS;

void cameraTriggerPulse() {
  digitalWrite(CAMERA_TRIGGER_PIN, HIGH);
  delayMicroseconds(50);
  digitalWrite(CAMERA_TRIGGER_PIN, LOW);
}

// =========================
// SD logging (measured servo state + commanded targets + IMU)
// ---------------------------------------------------------
// Present position is READ BACK from each servo via
// dxl.readControlTableItem(), which is a blocking round trip over the
// same half-duplex DXL bus the gait uses for its goal-position writes.
// Doing all 4 servos in one loop pass would stack 4 round trips on top
// of that iteration's 4 writes and stall the gait timing. Instead this
// spreads it out: ONE servo read per loop pass, and only writes a row
// to SD once a full cycle (4 passes) completes. A new cycle only
// starts every LOG_INTERVAL_MS, so logging runs at roughly 4Hz rather
// than the 50Hz gait loop rate.
//
// cmd_pos1/cmd_pos2/cmd_fin10/cmd_fin11 are the COMMANDED targets --
// what the code told the servo to aim for that instant. They're cheap
// (already sitting in RAM, no bus round trip) so they're grabbed
// fresh at the moment the row is written. The id{N}_pos columns are
// the MEASURED position actually read back from each servo -- compare
// the two to see tracking error/lag.
//
// timestamp_ms is milliseconds since THIS RUN started (runStartMillis,
// set once at boot after SD/IMU init), not since the board was last
// reset and not wall-clock time -- it resets to 0 each power cycle, so
// rows from different runs are distinguished by the "run" column and
// the "=== RUN N START ===" marker rows, not by timestamp_ms alone.
//
// IMU reads are a fast I2C transaction (no half-duplex turnaround
// like DXL), so the accel/gyro snapshot is taken in one shot right
// when the row is written rather than spread out.
// ---------------------------------------------------------
const int SD_CS_PIN = 4;
// NOTE: the built-in Arduino SD library only supports classic 8.3
// short filenames -- max 8 chars before the dot, max 3 after. A
// longer name (e.g. "dxl_log_test.csv") doesn't error loudly; SD.open()
// just returns an invalid File handle, and every write silently no-ops
// since setupSD()/logRow() both check `if (file)` first. Keep this at
// 8 characters or fewer.
const char* LOG_FILENAME = "dxl1.csv";
const unsigned long LOG_INTERVAL_MS = 250;  // how often a NEW read/log cycle can start

bool sdReady = false;
bool imuReady = false;

enum LogField { F_POS, FIELD_COUNT };
int32_t logBuffer[4][FIELD_COUNT];   // [index into ALL_MOTOR_IDS][field]
int logServoIndex = 0;
int logFieldIndex = 0;
bool loggingInProgress = false;
unsigned long logCycleStartTime = 0;
unsigned long lastLogTime = 0;

// Run counter persisted in EEPROM so each power-up/reset gets its own
// marker row and its own zeroed timestamp, without needing a new file.
const int EEPROM_RUN_COUNTER_ADDR = 0;
unsigned int runNumber = 0;
unsigned long runStartMillis = 0;

Adafruit_MPU6050 mpu;

bool setupSD() {
  // Mega hardware SPI needs the native SS pin (53) held as OUTPUT to
  // stay in master mode, even though the SD card's own CS is on pin 4.
  pinMode(SS, OUTPUT);

  // These prints go to the regular hardware Serial (USB), not
  // DEBUG_SERIAL -- see the boot-diagnostic note in setup(). This
  // function only ever runs before dxl.begin() claims that port.
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("SD init failed -- logging disabled"));
    return false;
  }

  if (!SD.exists(LOG_FILENAME)) {
    File f = SD.open(LOG_FILENAME, FILE_WRITE);
    if (f) {
      f.println("timestamp_ms,run,gait,cmd_pos1,cmd_pos2,cmd_fin10,cmd_fin11,"
                 "id1_pos,id7_pos,id10_pos,id11_pos,"
                 "accelX,accelY,accelZ,gyroX,gyroY,gyroZ");
      f.close();
    } else {
      Serial.println(F("SD.begin() OK but could not create/open log file"));
      return false;
    }
  }

  File marker = SD.open(LOG_FILENAME, FILE_WRITE);
  if (marker) {
    marker.print("=== RUN ");
    marker.print(runNumber);
    marker.println(" START ===");
    marker.close();
  } else {
    Serial.println(F("SD.begin() OK, header exists, but could not open for run marker"));
    return false;
  }

  Serial.print(F("SD OK -- logging to "));
  Serial.print(LOG_FILENAME);
  Serial.print(F(" (run "));
  Serial.print(runNumber);
  Serial.println(F(")"));
  return true;
}

// Startup diagnostic blink on the built-in LED (pin 13, unused by
// anything else on this board) -- lets you read SD/IMU init status
// without the optional debug adapter wired up. 1 blink = OK, 3 fast
// blinks = FAILED. SD status blinks first, then a pause, then IMU
// status. Purely a boot-time indicator -- doesn't affect the gait
// loop either way.
void blinkGroup(int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(150);
    digitalWrite(LED_BUILTIN, LOW);
    delay(150);
  }
}

void blinkStatus(bool sdOk, bool imuOk) {
  blinkGroup(sdOk ? 1 : 3);
  delay(600);
  blinkGroup(imuOk ? 1 : 3);
  delay(1000);
}

bool setupIMU() {
  // Same deal as setupSD(): prints to regular Serial since this only
  // ever runs before dxl.begin() takes over that port.
  if (!mpu.begin()) {
    Serial.println(F("MPU6050 not found -- IMU logging disabled"));
    return false;
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println(F("MPU6050 OK (m/s^2, rad/s)"));
  return true;
}

// Writes one completed cycle's worth of measured servo data, plus a
// fresh IMU snapshot and the current commanded targets, as one CSV row.
void logRow(unsigned long now, const char* gaitName, float cmdPos1, float cmdPos2, float cmdFin10, float cmdFin11) {
  if (!sdReady) return;

  float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  if (imuReady) {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    ax = accel.acceleration.x; ay = accel.acceleration.y; az = accel.acceleration.z;
    gx = gyro.gyro.x;          gy = gyro.gyro.y;          gz = gyro.gyro.z;
  }

  unsigned long logElapsed = logCycleStartTime - runStartMillis;

  File logFile = SD.open(LOG_FILENAME, FILE_WRITE);
  if (!logFile) return;

  logFile.print(logElapsed);      logFile.print(',');
  logFile.print(runNumber);       logFile.print(',');
  logFile.print(gaitName);        logFile.print(',');
  logFile.print(cmdPos1, 1);      logFile.print(',');
  logFile.print(cmdPos2, 1);      logFile.print(',');
  logFile.print(cmdFin10, 1);     logFile.print(',');
  logFile.print(cmdFin11, 1);     logFile.print(',');

  for (int i = 0; i < 4; i++) {
    logFile.print(logBuffer[i][F_POS]);  logFile.print(',');
  }

  logFile.print(ax, 3); logFile.print(',');
  logFile.print(ay, 3); logFile.print(',');
  logFile.print(az, 3); logFile.print(',');
  logFile.print(gx, 3); logFile.print(',');
  logFile.print(gy, 3); logFile.print(',');
  logFile.println(gz, 3);

  logFile.close();
}

// Call once per loop iteration. Advances the read state machine by
// exactly one field, and writes a row once a full cycle completes.
void updateLogging(unsigned long now, const char* gaitName, float cmdPos1, float cmdPos2, float cmdFin10, float cmdFin11) {
  if (!sdReady) return;

  if (!loggingInProgress && (now - lastLogTime >= LOG_INTERVAL_MS)) {
    loggingInProgress = true;
    logServoIndex = 0;
    logFieldIndex = 0;
    logCycleStartTime = now;
    lastLogTime = now;
  }

  if (!loggingInProgress) return;

  uint8_t id = ALL_MOTOR_IDS[logServoIndex];
  switch (logFieldIndex) {
    case F_POS: logBuffer[logServoIndex][F_POS] = dxl.readControlTableItem(ControlTableItem::PRESENT_POSITION, id); break;
  }

  logFieldIndex++;
  if (logFieldIndex >= FIELD_COUNT) {
    logFieldIndex = 0;
    logServoIndex++;
  }

  if (logServoIndex >= 4) {
    logRow(now, gaitName, cmdPos1, cmdPos2, cmdFin10, cmdFin11);
    loggingInProgress = false;
  }
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
  //DEBUG_SERIAL.begin(57600);
  pinMode(LED_BUILTIN, OUTPUT);

  // --- Boot-time diagnostic window ---------------------------------
  // Hardware Serial (pins 0/1) is only claimed by the DXL bus once
  // dxl.begin() runs below. Until then it's just sitting idle, so
  // it's safe to use it as a normal Serial Monitor for a few seconds
  // at boot -- open the monitor at 115200 baud and reset the board.
  // Once dxl.begin() runs, the same pins switch to DXL protocol
  // traffic and the Serial Monitor will show garbage from then on;
  // that's expected, not a fault. This window is the only place SD
  // and IMU init status get printed.
  Serial.begin(BAUDRATE);
  delay(1500);  // gives the Monitor time to reconnect after the reset caused by opening it
  Serial.println(F("=== Boot diagnostic (Serial goes to DXL traffic after this) ==="));

  Wire.begin();
  imuReady = setupIMU();

  EEPROM.get(EEPROM_RUN_COUNTER_ADDR, runNumber);
  if (runNumber == 0xFFFF) runNumber = 0;  // handle a totally blank/fresh EEPROM
  runNumber++;
  EEPROM.put(EEPROM_RUN_COUNTER_ADDR, runNumber);

  sdReady = setupSD();
  runStartMillis = millis();

  pinMode(CAMERA_TRIGGER_PIN, OUTPUT);
  Timer1.initialize(CAMERA_TRIGGER_PERIOD_US);
  Timer1.attachInterrupt(cameraTriggerPulse);

  Serial.print(F("Summary -- SD: "));
  Serial.print(sdReady ? F("OK") : F("FAILED"));
  Serial.print(F(", IMU: "));
  Serial.println(imuReady ? F("OK") : F("FAILED"));
  Serial.println(F("Switching Serial to DXL bus now..."));
  Serial.flush();  // make sure the summary is actually sent before DXL traffic starts

  blinkStatus(sdReady, imuReady);

  dxl.begin(BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  for (int i = 0; i < 4; i++) {
    writeByte(ALL_MOTOR_IDS[i], ADDR_TORQUE_ENABLE, 1);
  }
  //DEBUG_SERIAL.println("Torque enabled");

  writeWord(DXL_ID_1, ADDR_MOVING_SPEED, MOVING_SPEED);
  writeWord(DXL_ID_2, ADDR_MOVING_SPEED, MOVING_SPEED);
  writeWord(DXL_ID_10, ADDR_MOVING_SPEED, FIN_MOVING_SPEED);
  writeWord(DXL_ID_11, ADDR_MOVING_SPEED, FIN_MOVING_SPEED);
  //DEBUG_SERIAL.println("Moving speed set");

  writeWord(DXL_ID_1, ADDR_GOAL_POSITION, CENTER_POS_1);
  writeWord(DXL_ID_2, ADDR_GOAL_POSITION, CENTER_POS_2);
  writeWord(DXL_ID_10, ADDR_GOAL_POSITION, degToPos(FIN10_PLANTED_DEG));
  writeWord(DXL_ID_11, ADDR_GOAL_POSITION, degToPos(FIN11_PLANTED_DEG));
  delay(2000);

  //DEBUG_SERIAL.println("Starting animal-like gait sequence");

  gaitIndex = 0;
  gaitStartTime = millis();
  lastCommandedPos1 = CENTER_POS_1;
  lastCommandedPos2 = CENTER_POS_2;
  lastCommandedFin8 = degToPos(FIN10_PLANTED_DEG);
  lastCommandedFin11 = degToPos(FIN11_PLANTED_DEG);
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
  // position to the gait's final target
  float target1 = CENTER_POS_1 + gait.offset1;
  float target2 = CENTER_POS_2 + gait.offset2;
  float eBody = smoothstep(elapsedInGait / gait.easeTime);
  float pos1 = lerp(switchPos1, target1, eBody);
  float pos2 = lerp(switchPos2, target2, eBody);

  // Fins: wait finDelay seconds into the gait, then ease over
  // FIN_EASE_TIME so they plant/lift decisively, landing right at
  // the end of the pose rather than the start
  float targetFin8 = degToPos(gait.finRightDeg);
  float targetFin11 = degToPos(gait.finLeftDeg);
  float finElapsed = elapsedInGait - gait.finDelay;
  float eFin = smoothstep(finElapsed / FIN_EASE_TIME);
  float fin8 = lerp(switchFin8, targetFin8, eFin);
  float fin11 = lerp(switchFin11, targetFin11, eFin);

  lastCommandedPos1 = pos1;
  lastCommandedPos2 = pos2;
  lastCommandedFin8 = fin8;
  lastCommandedFin11 = fin11;

  writeWord(DXL_ID_1, ADDR_GOAL_POSITION, (int)pos1);
  writeWord(DXL_ID_2, ADDR_GOAL_POSITION, (int)pos2);
  writeWord(DXL_ID_10, ADDR_GOAL_POSITION, (int)fin8);
  writeWord(DXL_ID_11, ADDR_GOAL_POSITION, (int)fin11);

  // One read of one field per pass -- see comment block above updateLogging().
  updateLogging(now, gait.name, pos1, pos2, fin8, fin11);

  delay(20);
}
