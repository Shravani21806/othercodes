#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Timers
unsigned long timer = 0;
float timeStep = 0.01;
float yaw = 0;

#include "CytronMotorDriver.h"
CytronMD motor1(PWM_DIR, 3, 30);  // PWM 1 = Pin 3, DIR 1 = Pin 4.
CytronMD motor2(PWM_DIR, 5, 31);
CytronMD motor3(PWM_DIR, 6, 32);

float t = 0;

float bx = 0;
float by = 0;
float px0 = 0;
float px1 = 0;
float py0 = 0;
float py1 = 10;
float kp = 4;
float ki = 0;
float kd = 48;
float setpoint = 0;
float p;
float i;
float d;
float error = 0;
float preverror = 0;
int output;
int output1;
int flag_f, flag_b;
float r = 0;
float m1;
float m2;
float m3;
float M1;
float M2;
float M3;
float pi = 3.14;
float angle1 = 0;
//int Setpoint=90;

void setup() {

  Serial.begin(115200);
  // Initialize MPU6050
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  mpu.calibrateGyro();
  mpu.setThreshold(3);

#if !defined(MIPSEL)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  Serial.println("Start");


  pinMode(30, OUTPUT);
  pinMode(31, OUTPUT);
  pinMode(32, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);

}


void loop() {
  timer = millis();
  Vector norm = mpu.readNormalizeGyro();
  yaw = yaw + norm.ZAxis * timeStep;

  bx = (1 - t) * px0 + t * px1 ;
  by = (1 - t) * py0 + t * py1 ;


  t = t + 0.001;


if  (t>0.8)
{
  setpoint=90;
}


  int c = square(bx);
  int d = square(by);
  float r = sqrt(c + d);

  float angle = atan2 (by, bx);
  float theta = angle * (180 / pi);
  float theta2 = angle1 * (180 / pi);

  error = setpoint - yaw;

  p = error;
  i = i + error;
  d = error - preverror;

  output = kp * p + ki * i + kd * d;
  preverror = error;

  output = constrain(output, -150, 150);

  M1 =  (80 * (sin((pi / 2 - angle) + angle1)) * 1.15);
  M2 =  -(80 * (sin((7 * pi / 6 - angle) + angle1)) * 1.15);
  M3 =   (80 * (sin((33 * pi / 18 - angle) + angle1)) * 1.15);


  Serial.print(" m1 ");
  Serial.print(M1);
  Serial.print(" m2 ");
  Serial.print(M2);
  Serial.print(" m3 ");
  Serial.print(M3);

  M1 = constrain(M1, -150, 150);
  M2 = constrain(M2, -150, 150);
  M3 = constrain(M3, -150, 150);
  if (t >= 1)
  { 
    m1 = 0;
    m2 = 0;
    m3 = 0;
    t = 1;
  }
  if (t < 1) {
    m1 = M1 - output;
    m2 = M2 - output;
    m3 = -M3 - output;
  }


  motor1.setSpeed(- output);
  motor2.setSpeed(- output);
  motor3.setSpeed(- output);
  Serial.print("angle");
  Serial.print(angle);
  Serial.print(kp);
  Serial.print(kd);
  Serial.print(ki);


  Serial.print(" X ");
  Serial.print(setpoint);
  Serial.print(" Y ");
  Serial.print(by);
  Serial.print(" t ");
  Serial.print(t);
  Serial.println();

}
