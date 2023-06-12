//This code uses a push button as a toggle switch to 1) record data from a sensor (MPU6050 accerlerameter/gyroscope) until the button is pushed again to close the file. 
//Each file has a unique number according to the number of clicks
//The name of the file can be changed in line sprintf(fileName,"JUN8_%d.CSV",buttonPressCount++);
//Button
#define button_pin 7 
 int buttonStateNow; 
 int buttonStatePrev=0; 
 int LEDState = 0;
 int buttonPressCount = 1;
 int pushbutton = 0;

//SD
#include <SD.h>
#include <Wire.h>
int chipSelect = 4; 
File file;
int count;
char fileName[20];

//MPU6050
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

 void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
pinMode(button_pin, INPUT_PULLUP);

//MPU6050
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
if (SD.begin(chipSelect)) { //Initialize SD card
  Serial.println("Initialized SD card."); // if return is false, something went wrong
  Serial.println("Current File name: ");
}
else {
  Serial.println("SD card was not initialized.");
}
  delay(1000);
}


void loop() {
  // put your main code here, to run repeatedly:
  buttonStateNow = digitalRead(7);
  Serial.println(buttonStateNow);


if (buttonStatePrev==1 && buttonStateNow==0 && pushbutton ==0) {
       
    sprintf(fileName,"JUN8_%d.CSV",buttonPressCount++);
    file = SD.open(fileName, FILE_WRITE);
    if (file) {
      while (pushbutton==0) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      Serial.println(a.acceleration.x);         
      file.println(a.acceleration.x); 
      Serial.println(fileName); 

      delay(100);

      buttonStateNow = digitalRead(7);
      if (buttonStateNow == 0) {
        pushbutton = 1;
        break;
      }
      }
     } else {
      Serial.println("Could not open File (writing)");
    }

} else {
pushbutton = 0;
file.close();
Serial.println("File Closed");
Serial.println("Do nothing");
}

buttonStatePrev = buttonStateNow;

delay(100);

}



//}
