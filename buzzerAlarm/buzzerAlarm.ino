int buzzer = 13;
void setup() 
 {
  // put your setup code here, to run once:
  pinMode(buzzer, OUTPUT); 
  digitalWrite(buzzer,LOW);
  attachInterrupt(0,alarmON, RISING);
}

void loop() 
{
  // put your main code here, to run repeatedly:

}

void alarmON()
{
  digitalWrite(buzzer,HIGH);
}
