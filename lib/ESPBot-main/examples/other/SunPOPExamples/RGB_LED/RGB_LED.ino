int r=128;
int g=0;
int b=128; //紫色

void setup()
{
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);
  pinMode(12,OUTPUT);
}
void loop()
{
  digitalWrite(12,HIGH);    //5V 供电
  analogWrite(11,(255-r));
  analogWrite(10,(255-g));
  analogWrite(9,(255-b));
}
