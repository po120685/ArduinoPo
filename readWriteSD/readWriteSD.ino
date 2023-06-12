#include <SD.h>

int chipSelect = 4; //chip select pin for the microSD Card Adapter
File file; // file object that is used to read and write data

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200); // start serial connection to print out debug messages and data

pinMode(chipSelect, OUTPUT); // chip select pin must be set to OUTPUT mode
if (!SD.begin(chipSelect)) { //Initialize SD card
  Serial.println("Could not initialize SD card."); // if return is false, something went wrong
}

if (SD.exists("file.csv")) { // if "file.txt" exists, file will be deleted
   Serial.println("File exists."); 
   if (SD.remove("file.txt") == true) {
     Serial.println("Sucessfully removed file.");
   } else{
     Serial.println("Could not remove file.");
   }
 }
}

void loop() {
  // put your main code here, to run repeatedly:
file = SD.open("file.csv", FILE_WRITE); //open "file.txt" to write data
if (file) {
  int number = random(10); //generate random number between 0 and 9
  file.println(number); // write number to file
  file.close(); // close file
  Serial.print(" Wrote number: "); // debug output: show written number in serial monitor
  Serial.println(number); 
} else {
  Serial.println("Could not open file (writing). ");
}

file = SD.open("file.txt", FILE_READ); //open "file.txt to read data
if (file) {
  Serial.println("---- Reading start ----");
  char character; 
  while ((character = file.read()) != -1) { // this while loop reads data and store in "file.txt and prints it to serial monitor
    Serial.print(character); 
  }
  file.close(); 
  Serial.println("---Reading end ---"); 
 }else {
   Serial.println("Could not open file (reading).");
 }
 delay(5000); //wait for 5000ms
}

