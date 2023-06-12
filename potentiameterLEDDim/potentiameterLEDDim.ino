const int pwm3 = 3; //naming pin 3 as 'pwm3' variable 
const int pwm5 = 5; //naming pin 5 as 'pwm5' variable
const int pwm9 = 9; //naming pin 9 as 'pwm9' variable
const int pwm10 = 10; //naming pin 10 as'pwm10'variable
const int pwm11 = 11; //naming pin 10 as 'pwm11' variable
const int adc0 = A0; //naming pin 0 of analog input side as 'adc'
const int adc1 = A1; //naming pin 1 of analog input side as 'adc'
const int adc2 = A2; //naming pin 2 of analog input side as 'adc'
const int adc3 = A3; //naming pin 3 of analog input side as 'adc'
const int adc4 = A4; //naming pin4 of analog input side as 'adc'
void setup() 
{
  pinMode(pwm3,OUTPUT); //setting pin 3 as output
  pinMode(pwm5,OUTPUT); //setting pin 5 as output
  pinMode(pwm9,OUTPUT); //setting pin 9 as output
  pinMode(pwm10,OUTPUT); //setting pin 10 as output
  pinMode(pwm11,OUTPUT); //setting pin 11 as output
}

void loop() 
{
  int q = analogRead(adc0); //reading analog voltage and storing it in an integer
  int w = analogRead(adc1); //reading analog voltage and storing it in an integer
  int e = analogRead(A2); //reading analog voltage and storing it in an integer
  int r = analogRead(A3); //reading analog voltage and storing it in an integer
  int t = analogRead(A4); //reading analog voltage and storing it in an integer
  q = map(q,0,204.6,0,51);
  w = map(w,204.6,409.2,51,102);
  e = map(e,409.2,613.8,102,153);
  r = map(r,613.8,818.4,153,204);
  t = map(t,818.4,1023,204,255);
/*
the above function scales the output of adc, which is 10 bit 
and gives values btw 0 to 1023, invalues btw 0 to 255 
form analogWrite function which only recieves values btw this range 
*/
analogWrite(pwm3,q);
analogWrite(pwm5,w);
analogWrite(pwm9,e);
analogWrite(pwm10,r);
analogWrite(pwm11,t);
}
