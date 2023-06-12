int ledPin = 9; //PWM Pin
int value = -1;

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);

}

void loop() 
{
  while (Serial.available())       //Read while there is data in the buffet
  
    value = Serial.read();
    
      if (value >=0)              // If we have meaningful data
     
      Serial.println(value);      
      analogWrite(ledPin,value);  // set PWM for LED brightness
      value = -1;
  

}
