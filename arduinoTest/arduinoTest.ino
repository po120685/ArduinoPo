//potentiometer can read 0-1023 , led can read 0-255, mu is from 0 to max number of arms , x has 5 values
double    sigma    = 30, pi = 3.1416; // sigma also needs to be in the same units as x and mu
const int potPin   = A1; 
const int ledPin3  = 3;
const int ledPin5  = 5;
const int ledPin6  = 6;                       
const int ledPin9  = 9;
const int ledPin10 = 10;                       
float     readValue;                          
float     mu;   
float     x[5] = {0,72,144,216,288}; 
float     gauss[5];
float     gaussWriteLed[5];           

void setup() {
  Serial.begin(9600);  
  pinMode(potPin,INPUT);
  pinMode(ledPin3,OUTPUT); 
  pinMode(ledPin5,OUTPUT); 
  pinMode(ledPin6,OUTPUT); 
  pinMode(ledPin9,OUTPUT);                     
  pinMode(ledPin10,OUTPUT);                      
}

void loop() 
{
   readValue = analogRead(potPin);
   
   //mu = readValue*(360./270.); //convert potentiometer value read to mu
   
   mu = readValue*(288./1023.); // Sea star has 0-360 degrees and pot values are from 0-1023
   
   for (int i = 0; i<5; i++)
   {
    //x[i] = x[i]*(10./5.); // convert x to mu units
    gauss[i] = 255.*exp(-sq(x[i] - mu)/(2*sq(sigma))); //calculate the value according to gauss
    //gaussWriteLed[i] = gauss[i]*(255./5.); // gauss value (0-5) led value (0-255) scale the original calculation for gauss to led values
    analogWrite(ledPin3,gauss[0]);
    analogWrite(ledPin5,gauss[1]);
    analogWrite(ledPin6,gauss[2]);
    analogWrite(ledPin9,gauss[3]);
    analogWrite(ledPin10,gauss[4]);
    delay(1);
   }
   Serial.print("gauss0:");
   Serial.print(gauss[0]);
   Serial.print(",");
   Serial.print("gauss1:");
   Serial.print(gauss[1]);
   Serial.print(",");
   Serial.print("gauss2:");
   Serial.print(gauss[2]);
   Serial.print(",");
   Serial.print("gauss3:");
   Serial.print(gauss[3]);
   Serial.print(",");
   Serial.print("gauss4:");
   Serial.print(gauss[4]);
   Serial.print(",");
   Serial.print("READVALUE:");
   Serial.print(readValue); 
   Serial.print(",");
   Serial.print("MU:");
   Serial.println(mu);
  
  
}
