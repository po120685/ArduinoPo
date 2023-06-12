#define front_led_sw 3                             //defining the switch to pin 2
#define front_led 5                                //defining the led to pin 3
int front_led_sw_state = 0;                        //defining initial switch state
int front_brightness = 0;                          //defining initial brightness

void setup() {
  Serial.begin(9600);                              //opens serial port and sets data rate to 9600 bps
  pinMode(front_led_sw,INPUT);                     //configures the pin of the switch as input
  pinMode(front_led,OUTPUT);                       //configures the pin of the LED as output
}

void loop() {
  analogWrite(front_led, front_brightness);        //sets the output to the front_led pin with value of front brightness
  front_led_sw_state = digitalRead(front_led_sw); //sets the front_led_sw_state as the result of the button with digital read of the switch
  if (front_led_sw_state == HIGH) {               // if statement to set if the button is pressed as 25% to the brightness
    front_brightness = front_brightness + 63;
}

if (front_brightness > 255) {                     // if the brightness becomes greater than 255 then turn the led off
  front_brightness = 0;
  analogWrite(front_led,front_brightness);
  }
  delay(1000);
 }
