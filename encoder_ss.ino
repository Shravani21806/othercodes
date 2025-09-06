#include "CytronMotorDriver.h"
#include <PinChangeInterrupt.h>

const int base_encoderPinA = 2;
const int base_encoderPinB = 3;

const int axis_encoderPinA = 12;
const int axis_encoderPinB =13;

int base_encoderPosition = 0;  
int base_lastEncoded = 0;   
int base_encoderValue = 0; 

int axis_encoderPosition = 0;  
int axis_lastEncoded = 0;   
int axis_encoderValue = 0; 

float error_base = 0;
float setpoint_base = 0;
float integral_base = 0;
float derivative_base = 0;
float preError_base = 0;

float Kp_base = 0;
float Ki_base = 0;
float Kd_base = 0;
float PID_base = 0;

float error_axis = 0;
float setpoint_axis = 0;
float integral_axis = 0;
float derivative_axis = 0;
float preError_axis = 0;

float Kp_axis = 0;
float Ki_axis = 0;
float Kd_axis = 0;
float PID_axis = 0;


CytronMD motor1(PWM_DIR, 9, 8); 
CytronMD motor2(PWM_DIR, 6, 7); 


void setup() {
  pinMode(base_encoderPinA, INPUT);
  pinMode(base_encoderPinB, INPUT);
  pinMode(axis_encoderPinA, INPUT);
  pinMode(axis_encoderPinB, INPUT);

  digitalWrite(base_encoderPinA, HIGH);
  digitalWrite(base_encoderPinB, HIGH);
  digitalWrite(axis_encoderPinA, HIGH);
  digitalWrite(axis_encoderPinB, HIGH);

  attachInterrupt(digitalPinToInterrupt(base_encoderPinA), updateEncoder_base, CHANGE);
  attachInterrupt(digitalPinToInterrupt(base_encoderPinB), updateEncoder_base, CHANGE);

  attachPCINT(digitalPinToPCINT(axis_encoderPinA), updateEncoder_axis, CHANGE);
  attachPCINT(digitalPinToPCINT(axis_encoderPinB), updateEncoder_axis, CHANGE);

  Serial.begin(9600);
}

void loop() {
  setpoint_base = -1940;
  error_base = setpoint_base - base_encoderPosition;
  integral_base = error_base + integral_base;
  derivative_base = error_base - preError_base;
  preError_base = error_base;
  Kp_base = 1.5;
  Ki_base = 0;
  Kd_base = 0;
  PID_base = (Kp_base * error_base) + (Ki_base * integral_base) + (Kd_base * derivative_base);
  // PID_base=0;

  setpoint_axis = 0;
  error_axis = setpoint_axis - axis_encoderPosition;
  integral_axis = error_axis + integral_axis;
  derivative_axis = error_axis - preError_axis;
  preError_axis = error_axis;
  Kp_axis = 0.3;
  Ki_axis = 0;
  Kd_axis = 0;
  PID_axis = (Kp_axis * error_axis) + (Ki_axis * integral_axis) + (Kd_axis * derivative_axis);
 PID_axis=0;

  // PID_axis = constrain (PID_axis, -100, 100);
  PID_base = constrain (PID_base, -100, 100);

  motor1.setSpeed(0);
  motor2.setSpeed(PID_base);


  Serial.print("Position Base : ");
  Serial.print(base_encoderPosition);
  Serial.print("   Position axis : ");
  Serial.print(axis_encoderPosition);
  Serial.print("  error1 : ");
  Serial.print(error_base);
  // Serial.print("   Position axis : ");
  // Serial.print(axis_encoderPosition);
  Serial.println(" ");
  
}

void updateEncoder_base() {
  int MSB = digitalRead(base_encoderPinA);
  int LSB = digitalRead(base_encoderPinB); 

  int base_encoded = (MSB << 1) | LSB;
  int sum = (base_lastEncoded << 2) | base_encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) base_encoderPosition++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) base_encoderPosition--;

  base_lastEncoded = base_encoded;
}


void updateEncoder_axis() {
  int MSB = digitalRead(axis_encoderPinA);
  int LSB = digitalRead(axis_encoderPinB); 

  int axis_encoded = (MSB << 1) | LSB;
  int sum = ( axis_lastEncoded << 2) | axis_encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) axis_encoderPosition++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) axis_encoderPosition--;

  axis_lastEncoded=axis_encoded;
}
