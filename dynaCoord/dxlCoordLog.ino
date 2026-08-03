#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>
#include <Dynamixel2Arduino.h>

#if defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8);   // shield's debug UART pins
  #define DXL_SERIAL   Serial        // shield's DXL bus — pins 0/1, fixed in hardware
  #define DEBUG_SERIAL soft_serial   // only viewable with an external adapter on 7/8
  const int DXL_DIR_PIN = 2;         // shield's DIR pin — fixed in hardware
#endif

const uint8_t DXL_ID1  = 1; 
const uint8_t DXL_ID2  = 2;
const uint8_t DXL_ID3  = 3;
const uint8_t servoIds[3] = {DXL_ID1, DXL_ID2, DXL_ID3};

const float DXL_PROTOCOL_VERSION = 2.0; 

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

using namespace ControlTableItem;

// --- SD card setup ---
const int SD_CS_PIN = 4;
const char* LOG_FILENAME = "dxl_log.csv";
bool sdReady = false;

// --- run counter (persisted in EEPROM across resets/power cycles) ---
const int EEPROM_RUN_COUNTER_ADDR = 0;
unsigned int runNumber = 0;
unsigned long runStartMillis = 0;   // marks when this run's "zero point" is

// --- logging timing ---
unsigned long lastLogTime = 0;
const unsigned long LOG_INTERVAL_MS = 250; // how often a NEW logging cycle can start

// --- non-blocking, spread-out logging state machine ---
enum LogField { F_POS, F_VEL, F_LOAD, FIELD_COUNT };
int32_t logBuffer[3][FIELD_COUNT]; // [servo index 0-2][field]
int logServoIndex = 0;
int logFieldIndex = 0;
bool loggingInProgress = false;
unsigned long logCycleStartTime = 0;

// --- MPU6050 setup ---
const uint8_t MPU_ADDR = 0x68;          // default I2C address (AD0 tied low/unconnected)
const uint8_t MPU_PWR_MGMT_1 = 0x6B;
const uint8_t MPU_ACCEL_XOUT_H = 0x3B;  // accel/gyro registers are contiguous from here

int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;

void mpuInit() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_PWR_MGMT_1);
  Wire.write(0);           // wake the MPU6050 up (it starts in sleep mode by default)
  Wire.endTransmission(true);
}

void mpuReadAll() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true); // 14 bytes: accel(6) + temp(2) + gyro(6)

  accelX = Wire.read() << 8 | Wire.read();
  accelY = Wire.read() << 8 | Wire.read();
  accelZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();  // skip the two temperature bytes, not logging that
  gyroX  = Wire.read() << 8 | Wire.read();
  gyroY  = Wire.read() << 8 | Wire.read();
  gyroZ  = Wire.read() << 8 | Wire.read();
}

void setup(void) {
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

  // --- MPU6050 init ---
  mpuInit();

  // --- bump persistent run counter ---
  EEPROM.get(EEPROM_RUN_COUNTER_ADDR, runNumber);
  if (runNumber == 0xFFFF) runNumber = 0; // handle a totally blank/fresh EEPROM
  runNumber++;
  EEPROM.put(EEPROM_RUN_COUNTER_ADDR, runNumber);

  // --- SD card init ---
  pinMode(51, OUTPUT); // keep native SS as output for SPI master mode on Mega
  if (!SD.begin(SD_CS_PIN)) {
    sdReady = false;
  } else {
    sdReady = true;

    // Write CSV header only if this is a brand new file
    if (!SD.exists(LOG_FILENAME)) {
      File logFile = SD.open(LOG_FILENAME, FILE_WRITE);
      if (logFile) {
        logFile.println("timestamp_ms,id,position,velocity,load,accelX,accelY,accelZ,gyroX,gyroY,gyroZ");
        logFile.close();
      }
    }

    // Write a marker row for this run/restart
    File logFile = SD.open(LOG_FILENAME, FILE_WRITE);
    if (logFile) {
      logFile.print("=== RUN ");
      logFile.print(runNumber);
      logFile.println(" START ===,,,,,,,,,,");
      logFile.close();
    }

    runStartMillis = millis();   // this run's clock starts now
  }
}

void loop() {
  int drive = 3000 + 500 * sin(millis()/250.0);

  int pos1 = dxl.getPresentPosition(DXL_ID1); 
  int pos2 = dxl.getPresentPosition(DXL_ID2);

  int offset = 0;

  dxl.setGoalPosition(DXL_ID1, drive);
  dxl.setGoalPosition(DXL_ID2, pos1 + offset);
  dxl.setGoalPosition(DXL_ID3, pos2 + offset);

  unsigned long now = millis();
  unsigned long elapsed = now - runStartMillis;

  // --- start a new logging cycle if it's time and one isn't already running ---
  if (sdReady && !loggingInProgress && (now - lastLogTime >= LOG_INTERVAL_MS)) {
    loggingInProgress = true;
    logServoIndex = 0;
    logFieldIndex = 0;
    logCycleStartTime = now;
    lastLogTime = now;
  }

  // --- do ONE register read per loop pass, not fifteen at once ---
  if (loggingInProgress) {
    uint8_t id = servoIds[logServoIndex];

    switch (logFieldIndex) {
      case F_POS:  logBuffer[logServoIndex][F_POS]  = dxl.readControlTableItem(PRESENT_POSITION, id); break;
      case F_VEL:  logBuffer[logServoIndex][F_VEL]  = dxl.readControlTableItem(PRESENT_VELOCITY, id); break;
      case F_LOAD: logBuffer[logServoIndex][F_LOAD] = dxl.readControlTableItem(PRESENT_LOAD, id);     break;
    }

    logFieldIndex++;
    if (logFieldIndex >= FIELD_COUNT) {
      logFieldIndex = 0;
      logServoIndex++;
    }

    // once all 3 servos x 3 fields are collected, write everything in one batch
    if (logServoIndex >= 3) {
      // I2C reads are fast (no half-duplex turnaround like DXL), so it's fine
      // to grab the IMU snapshot right here rather than spreading it out
      mpuReadAll();

      unsigned long logElapsed = logCycleStartTime - runStartMillis;
      File logFile = SD.open(LOG_FILENAME, FILE_WRITE);
      if (logFile) {
        for (int i = 0; i < 3; i++) {
          logFile.print(logElapsed);
          logFile.print(",");
          logFile.print(servoIds[i]);
          logFile.print(",");
          logFile.print(logBuffer[i][F_POS]);
          logFile.print(",");
          logFile.print(logBuffer[i][F_VEL]);
          logFile.print(",");
          logFile.print(logBuffer[i][F_LOAD]);
          logFile.print(",");
          logFile.print(accelX);
          logFile.print(",");
          logFile.print(accelY);
          logFile.print(",");
          logFile.print(accelZ);
          logFile.print(",");
          logFile.print(gyroX);
          logFile.print(",");
          logFile.print(gyroY);
          logFile.print(",");
          logFile.println(gyroZ);
        }
        logFile.close();
      }
      loggingInProgress = false;
    }
  }

  delay(20);
}
