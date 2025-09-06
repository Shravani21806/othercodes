float setpoint;
float pre_error;
float kp=0;
float ki=0;
float kd=0;
float p;
float i;
float d;
int q;
float error;
float pid;
#include <SabertoothSimplified.h>
SabertoothSimplified ST;
int count=0;
volatile int last_state=LOW;
void setup() {
  Serial.begin(9600);
  pinMode(12,INPUT_PULLUP);
  pinMode(13,INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(12), motor_count, CHANGE);
     Serial.begin(9600);
   SabertoothTXPinSerial.begin(9600);
}
void loop() {
error=setpoint-count;
p=error;
i=i+error;
d=error-pre_error;
pre_error=error;
 pid=kp*p+ki*i+kd*d;
 pid=constrain(pid,-127,127);
if(error>-1.5 && error<1.5)
{
 kp=1.9;
 ki=0;
 kd=3.2;
}
else
{
 kp=0.9;
 ki=0;
 kd=3;
}

  ST.motor(1, pid);
  // Serial.print(count);
  // Serial.print("  ");
  //  Serial.print(error);
   Serial.print(" p ");
   Serial.print(p);
   Serial.print(" q");
   Serial.println(q);
}

void motor_count(){
  int p = digitalRead(12);
  int q = digitalRead(13);
 if (p != last_state){
  if(p!=q)
  {
    count++;
  }
  else
  {
    count--;
  }
 }
 last_state=p;
}
