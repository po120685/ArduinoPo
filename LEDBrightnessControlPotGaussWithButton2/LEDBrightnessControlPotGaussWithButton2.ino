#include <ezButton.h>

//potentiometer can read 0-1023 , led can read 0-255, mu is from 0 to max number of arms , x has 5 values
# include <ezButton.h> 

# define LOOP_STATE_STOPPED 0
# define LOOP_STATE_STARTED 1

ezButton button(12);
int loopState = LOOP_STATE_STOPPED;

double    sigma     = 30, pi = 3.1416; // sigma also needs to be in the same units as x and mu
const int potPin1   = A1; // pin to read potentiometer
const int potPin4   = A4; // pin to read potentiometer
const int ledPin3   = 3;  // pin to out put to LED
const int ledPin5   = 5;  // pin to out put to LED
const int ledPin6   = 6;  // pin to out put to LED                     
const int ledPin9   = 9;  // pin to out put to LED
const int ledPin10  = 10; // pin to out put to LED  
const int anaThresh = 500;                    
float     readValue1;  
float     readValue4;                        
float     mu;   
float     x[5]      = {0,72,144,216,288}; 
float     gauss[5];
float     gaussWriteLed[5];    

void setup()  {
  Serial.begin(9600);  
  pinMode(potPin1,INPUT);
  pinMode(potPin4,INPUT);
  pinMode(ledPin3,OUTPUT); 
  pinMode(ledPin5,OUTPUT); 
  pinMode(ledPin6,OUTPUT); 
  pinMode(ledPin9,OUTPUT);                     
  pinMode(ledPin10,OUTPUT);  
  button.setDebounceTime(50);                    
}

void loop()  {
  button.loop();

  if (button.isPressed()) {
    if (loopState == LOOP_STATE_STOPPED)
      loopState = LOOP_STATE_STARTED;
    else 
      loopState = LOOP_STATE_STOPPED;
  }

  if (loopState == LOOP_STATE_STARTED) {
  
       readValue4 = analogRead(potPin4);
    
   if (readValue4 == 0. && readValue4 < 10.)
   {
    readValue1 = analogRead(potPin1);
   
    mu = readValue1*(288./1023.); // Sea star has 0-360 degrees and pot values are from 0-1023
   
    for (int i = 0; i<5; i++)
    {
      gauss[i] = 255.*exp(-sq(x[i] - mu)/(2*sq(sigma))); //calculate the value according to gauss using mu
      analogWrite(ledPin3,gauss[0]);
      analogWrite(ledPin5,gauss[1]);
      analogWrite(ledPin6,gauss[2]);
      analogWrite(ledPin9,gauss[3]);
      analogWrite(ledPin10,gauss[4]);
      delay(1);
    }
   //Serial.print("LED1: ");
   //Serial.print(gauss[0]);
   //Serial.print(",");
   //delay(1);
   //Serial.print("LED2: ");
   //Serial.print(gauss[1]);
   //Serial.print(",");
   //delay(1);
   //Serial.print("LED3: ");
   //Serial.print(gauss[2]);
   //Serial.print(",");
   //delay(1);
   //Serial.print("LED4: ");
   //Serial.print(gauss[3]);
   //Serial.print(",");
   //delay(1);
   //Serial.print("LED5: ");
   //Serial.println(gauss[4]);
   //Serial.print(",");
   //Serial.print("READVALUE:");
   //Serial.print(readValue); 
   //Serial.print(",");
  // Serial.print("MU:");
   //Serial.println(mu);
   Serial.println(readValue4);
   }
   

   if (readValue4 > 10. && readValue4 < 1010.)
   {
    readValue1 = analogRead(potPin1);
   
    mu = readValue1*(288./1023.); // Sea star has 0-360 degrees and pot values are from 0-1023
   
    for (int i = 0; i<5; i++)
    {
      //x[i] = x[i]*(10./5.); // convert x to mu units
      gauss[i] = 255.*exp(-sq(x[i] - mu)/(2*sq(sigma))); //calculate the value according to gauss
      //gaussWriteLed[i] = gauss[i]*(255./5.); // gauss value (0-5) led value (0-255) scale the original calculation for gauss to led values
      analogWrite(ledPin3,gauss[0]);
      analogWrite(ledPin5,gauss[4]);
      analogWrite(ledPin6,gauss[0]);
      analogWrite(ledPin9,gauss[4]);
      analogWrite(ledPin10,gauss[0]);
      delay(1);
    }
   //Serial.print("LED1: ");
   //Serial.print(gauss[0]);
   //Serial.print(",");
   //delay(15);
   //Serial.print("LED2: ");
   //Serial.print(gauss[4]);
   //Serial.print(",");
   //delay(15);
   //Serial.print("LED3: ");
   //Serial.print(gauss[0]);
   //Serial.print(",");
   //delay(15);
   //Serial.print("LED4: ");
   //Serial.print(gauss[4]);
   //Serial.print(",");
   //delay(15);
   //Serial.print("LED5: ");
   //Serial.println(gauss[0]);
   //Serial.print(",");
   //Serial.print("READVALUE:");
   //Serial.print(readValue); 
   //Serial.print(",");
  // Serial.print("MU:");
   //Serial.println(mu);
    //Serial.println(readValue4);
   }

     if (readValue4 > 1010. && readValue4 == 1023) {
    readValue1 = analogRead(potPin1);

    if(readValue1 > anaThresh) {
    
      digitalWrite(ledPin3, HIGH); // turn on LED in pin 3
      digitalWrite(ledPin10,LOW); // turn off LED in pin 10
      digitalWrite(ledPin5, LOW);
      digitalWrite(ledPin6, LOW);
      digitalWrite(ledPin9, LOW);
    }
    else {
    
    digitalWrite (ledPin3, LOW);  // turn off LED in pin 10
    digitalWrite (ledPin10,HIGH); // turn on LED in pin 3
    digitalWrite(ledPin5, LOW);
    digitalWrite(ledPin6, LOW);
    digitalWrite(ledPin9, LOW);
    }

     
  //Serial.print("LED1: ");
  //Serial.print(gauss[0]);
  //Serial.print(",");
  //delay(1);
  //Serial.print("LED2: ");
  //Serial.print(gauss[0]);
  //Serial.print(",");
  //delay(1);
  //Serial.print("LED3: ");
  //Serial.print(gauss[0]);
  //Serial.print(",");
  //delay(1);
  //Serial.print("gauss0 ");
  //Serial.print(gauss[0]);
  //Serial.print(",");
  //delay(1);
  //Serial.print("gauss4 ");
  //Serial.print(gauss[4]);
  //Serial.print(",");
  //Serial.print("READVALUE:");
  //Serial.print(readValue); 
  //Serial.print(",");
  //Serial.print("MU:");
  //Serial.println(mu);
  //Serial.print("readValue4: ");
  //Serial.println(readValue4);
  }
 }
if (loopState == LOOP_STATE_STOPPED) {
  digitalWrite(ledPin3, LOW);
  digitalWrite(ledPin5, LOW);
  digitalWrite(ledPin6, LOW);
  digitalWrite(ledPin9, LOW);
  digitalWrite(ledPin10, LOW);
}
}
//testEdit   
//testEdit2
