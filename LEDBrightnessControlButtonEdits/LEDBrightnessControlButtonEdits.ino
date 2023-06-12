const int button = 2;                        //defining the switch to pin 2
const int led    = 5;                        //defining the led to pin 5
int buttonState  = 0;                        //defining initial switch state
int brightness   = 0;                        //defining initial brightness

void setup() {
  Serial.begin(9600);                        //opens serial port and sets data rate to 9600 bps
  pinMode(button,INPUT);                     //configures the pin of the switch as input
  pinMode(led,OUTPUT);                       //configures the pin of the LED as output
}

void loop() 
{
   buttonState = digitalRead(button);    //sets the front_led_sw_state as the result of the button with digital read of the switch
  if (buttonState == HIGH) 
  {                                     // if statement to set if the button is pressed as 25% to the brightness
    brightness = brightness + 51;
    analogWrite(led,brightness);         //sets the output to the front_led pin with value of front brightness
    Serial.println(brightness);
    delay(100);
  }

if (brightness > 255)                  // if the brightness becomes greater than 255 then turn the led off
  {                                       
  brightness = 0;
  analogWrite(led, brightness);
  Serial.println(brightness);
  delay(100);
  }
}
