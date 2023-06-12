#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

#include<SD.h>
int chipSelect = 4;
File file;

void setup(void) {
	Serial.begin(115200);

	// Try to initialize!
	if (!mpu.begin()) {
		Serial.println("Failed to find MPU6050 chip");
		while (1) {
		  delay(10);
		}
	}
	Serial.println("MPU6050 Found!");

	// set accelerometer range to +-8G
	mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

	// set gyro range to +- 500 deg/s
	mpu.setGyroRange(MPU6050_RANGE_500_DEG);

	// set filter bandwidth to 21 Hz
	mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

	delay(100);

  pinMode(chipSelect, OUTPUT); // chip select pin must be set to OUTPUT mode
if (!SD.begin(chipSelect)) { //Initialize SD card
  Serial.println("Could not initialize SD card."); // if return is false, something went wrong
}

if (SD.exists("FILE.CSV")) { // if "file.txt" exists, file will be deleted
   Serial.println("File exists."); 
   if (SD.remove("FILE.CSV") == true) {
     Serial.println("Sucessfully removed file.");
     file = SD.open("FILE.CSV", FILE_WRITE); //open "file.txt" to write data
     //file.println("xAcc(m/s), yAcc(m/s), zAcc(m/s), xRot(rad/s), yRot(rad/s), zRot(rad/s), temp(C)");
     file.println("time, xAcc, yAcc, zAcc, xRot, yRot, zRot, temp, servo1Pos");
     file.close();
     delay(1000);
   } else{
     Serial.println("Could not remove file.");
   }
 }
}

void loop() {
	/* Get new sensor events with the readings */
	sensors_event_t a, g, temp;
	mpu.getEvent(&a, &g, &temp);
Serial.println(a.acceleration.x);
//initialize SD card
file = SD.open("FILE.CSV", FILE_WRITE); //open "file.txt" to write data
if (file) {
  file.print(temp.timestamp);
  file.print(",");
  file.print(a.acceleration.x);
  file.print(",");
  file.print(a.acceleration.y);
  file.print(",");
  file.print(a.acceleration.z); // write number to file
  file.print(",");
  file.print(g.gyro.x);
  file.print(",");
  file.print(g.gyro.y);
  file.print(",");
  file.print(g.gyro.z);
  file.print(",");
  file.println(temp.temperature);
  file.close(); 

  Serial.print("Wrote number acceleration X (m/s2): "); // debug output: show written number in serial monitor
  Serial.println(a.acceleration.x); 
  Serial.print("Wrote number acceleration Y (m/s2): "); // debug output: show written number in serial monitor
  Serial.println(a.acceleration.y); 
  Serial.print("Wrote number acceleration Z (m/s2): "); // debug output: show written number in serial monitor
  Serial.println(a.acceleration.z); 

  Serial.print("Wrote number Rotation  (rad/s): "); // debug output: show written number in serial monitor
  Serial.println(g.gyro.x); 
  Serial.print("Wrote number Rotation  (rad/s): "); // debug output: show written number in serial monitor
  Serial.println(g.gyro.y);
  Serial.print("Wrote number Rotation  (rad/s): "); // debug output: show written number in serial monitor
  Serial.println(g.gyro.z);


} else {
  Serial.println("Could not open file (writing). ");
}

	Serial.println("");
	delay(500);
}

