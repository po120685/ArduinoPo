int led_pin6 = 6; 
int led_pin3 = 3;
//int led_pin5 = 5; 
//int led_pin9 = 9;
//int led_pin10 = 10; 
int pot_pin0 = A0; 
int pot_pin1 = A1;
//int pot_pin2 = A2; 
//int pot_pin3 = A3;
//int pot_pin4 = A4;
int output1; 
int output2;
//int output3; 
//int output4;
//int output5;
int led_value1;
int led_value2;
//int led_value3;
//int led_value4;
//int led_value5;
void setup() {
  // put your setup code here, to run once:
  pinMode(led_pin6,OUTPUT);
  pinMode(led_pin3,OUTPUT);
  //pinMode(led_pin5,OUTPUT);
  //pinMode(led_pin9,OUTPUT);
  //pinMode(led_pin10,OUTPUT);

  //read pins 

  Serial.begin(9600); // initialize serial communication at 9600 bits per second
}

void loop() {
  // put your main code here, to run repeatedly:
   //first set of pins: ~6 to A0, output1, led_value1
   output1 = analogRead(pot_pin0); 
   led_value1 = map(output1, 0, 1023, 0, 255);
   analogWrite(led_pin6, led_value1); 
   delay(1);
   //second set of pins: ~3 to A1, output2, led_value2
   output2 = analogRead(pot_pin1);
   led_value2 = map(output2, 0, 1023, 0, 255); 
   analogWrite(led_pin3, led_value2);
   delay(1);
  //first set of pins: ~5 to A2, output1, led_value1
   //output3 = analogRead(pot_pin2); 
   //led_value3 = map(output3, 0, 1023, 0, 255);
   //analogWrite(led_pin5, led_value3); 
   //delay(1);
   //second set of pins: ~9 to A3, output2, led_value2
   //output4 = analogRead(pot_pin3);
   //led_value4 = map(output4, 0, 1023, 0, 255); 
   //analogWrite(led_pin9, led_value4);
   //delay(1);
   //second set of pins: ~10 to A4, output2, led_value2
   //output5 = analogRead(pot_pin4);
   //led_value5 = map(output5, 0, 1023, 0, 255); 
   //analogWrite(led_pin10, led_value5);
   //delay(1);
   
   //read pins 
   int sensorValue1 = analogRead(A0); 
   int sensorValue2 = analogRead(A1); 
   //int sensorValue3 = analogRead(A2); 
   //int sensorValue4 = analogRead(A3);
   //int sensorValue5 = analogRead(A4);
   Serial.println(sensorValue1);//print out the value
   Serial.print(",");
   Serial.println(sensorValue2); //print out the value
   //Serial.print(",");
   //Serial.println(sensorValue3); //print out the value
   //Serial.print(",");
   //Serial.print(sensorValue4); //print out the value
   //Serial.print(",");
   //Serial.println(sensorValue5); //print out the value
   delay(10); //delay in between for stability

}
