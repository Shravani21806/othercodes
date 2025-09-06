 #include "CytronMotorDriver.h"


// Configure the motor driver.
CytronMD motor(PWM_DIR, 3, 4);
// #include <Servo.h>


// Servo esc1; 
// Servo esc2; 
// int escPin1 = 7;
// int escPin2= 8;// Connect the ESC signal wire to pin 9
// int throttle = 1000; 
void setup() {
  Serial.begin(9600);
  // Serial.println("Basic Encoder Test:");
//   esc1.attach(escPin1); 
//   esc2.attach(escPin2);
//   esc1.writeMicroseconds(1000); 
//  esc2.writeMicroseconds(1000);
}


void loop() {
motor.setSpeed(50);
// esc1.writeMicroseconds(1000);
// esc2.writeMicroseconds(1000);


}

