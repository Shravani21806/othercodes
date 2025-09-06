/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */
#include <SabertoothSimplified.h>

SabertoothSimplified ST; 
#include <Encoder.h>

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(2, 3);
//   avoid using pins with LEDs attached
float setpoint=282*3;
float pre_error;
float kp=.72;
float ki=0;
float kd=2.5;
float a;
float p;
float i;
float d;
float error;
float pid;


void setup() {
  Serial.begin(9600);
 
   SabertoothTXPinSerial.begin(9600);
}

long oldPosition  = -999;

void loop() {
  int a = myEnc.read();
   


 error=setpoint-a;
 p=error;
 i=i+error;
 d=error-pre_error;
pre_error=error;
 pid=kp*p+ki*i+kd*d;
 pid=constrain(pid,-127,127);


  ST.motor(2, pid);

  Serial.print(a);
  Serial.print("  ");
   Serial.println(error);
}