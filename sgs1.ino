#include <SPI.h>
#include <hiduniversal.h>
#include <usbhub.h>
#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>
#include "hidjoystickrptparser.h"
#include <Wire.h>
#include <MPU6050.h>

#include <MPU6050.h>
#include <Servo.h>

Servo esc1;
Servo esc2;  // Create a Servo object to control the ESC
int escPin1 = 10;
int escPin2 = 11;


MPU6050 mpu;

USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);
JoystickEvents JoyEvents;
JoystickReportParser Joy(&JoyEvents);

SoftwareSerial SWSerial1(NOT_A_PIN, 3);  // RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified motorSAB1(SWSerial1);
SoftwareSerial SWSerial(NOT_A_PIN, 9);  // Use SWSerial as the serial port.
SabertoothSimplified motorSAB2(SWSerial);


int motor1 = 1;
int motor2 = 2;
int motor3 = 2;
int motor4 = 1;

float pi = 3.14;

float V1 = 0;  // left_front
float V2 = 0;  //
float V3 = 0;  //
float V4 = 0;  //

float M1 = 0;
float M2 = 0;
float M3 = 0;
float M4 = 0;

float resultant = 0;

int dirPin = 8;
int stepPin = 9;
int en = 4;
int stepsPerRevolution = 200;


float angle = 0;  // in radian
float Angle = 0;  //in degree


float error = 0;
float preError = 0;
float Kp = 0;
float Ki = 0;
float Kd = 0;
float PID = 0;
float setpoint = 0;
float integral = 0;
float derivative = 0;

// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;

int L1_ref = 0;
int R1_ref = 0;
int L2_ref = 0;
int R2_ref = 0;

int yellow_ref = 0;
int red_ref = 0;
int blue_ref = 0;
int green_ref = 0;

int dir = 0;

int start_ref = 0;
int start_count = 0;
int red_count = 0;
int red_act = 0;
int red_pre = 0;
int counter = 0;
int pre_counter = 0;

int red_ref1 = 0;
int blue_count = 0;
int blue_ref1 = 0;

int throttle = 1000;
#define PULSE_PIN 8  // Digital pin connected to PUL+ (Pulse).
#define DIR_PIN 7    // Digital pin connected to DIR+ (Direction).
#define ENA_PIN 6    // Digital pin connected to ENA+ (Enable) [Optional].

const int stepsPerRevolution = 12000;


void setup() {
  Serial.begin(9600);
#if !defined(MIPSEL)
  while (!Serial)
    ;  // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  Serial.println("Start");


  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(1);

  SWSerial.begin(9600);
  SWSerial1.begin(9600);

  esc1.attach(escPin1);
  esc2.attach(escPin2);  // Attach the ESC to the signal pin
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  pinMode(8, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);

  digitalWrite(6, LOW);   // LOW to enable, HIGH to disable.
  digitalWrite(7, HIGH);  // HIGH for one direction, LOW for the opposite.
}

void loop() {

  timer = millis();

  // Read normalized values
  Vector norm = mpu.readNormalizeGyro();

  // Calculate Pitch, Roll and Yaw
  pitch = pitch + norm.YAxis * timeStep;
  roll = roll + norm.XAxis * timeStep;
  yaw = yaw + norm.ZAxis * timeStep;


  Usb.Task();

  float Buttons = JoyEvents.bu;
  float lxa = JoyEvents.lx;   //  map( JoyEvents.lx, 0, 127,0, 1023);      //map(JoyEvents.Y, 0, 0xFF, 0.f, 255.f);
  float lya = JoyEvents.ly;   //map(JoyEvents.ly, 0, 127, 0, 1023);                   // map(JoyEvents.Z1, 0, 0xFF, 0.f, 255.f);
  double rxa = JoyEvents.rx;  //         map(JoyEvents.rx, 0, 127, 0, 1023); // map(JoyEvents.Z2, 0, 0xFF, 0.f, 255.f);
  double rya = JoyEvents.ry;  // map(JoyEvents.ry, 0, 127, 0, 1023); // map(JoyEvents.Rz, 0, 0xFF, 0.f, 255.f);
  //Group initialize
  float blue = Joy.blue;
  float green = Joy.green;
  float red = Joy.red;
  float yellow = Joy.yellow;
  float L1 = Joy.lb;
  float R1 = Joy.rb;
  float gpad = JoyEvents.ht;
  float L2 = Joy.lt;
  float R2 = Joy.rt;
  float back = Joy.bk;
  float start = Joy.st;
  float leftjoy = Joy.jl;
  float rightjoy = Joy.jr;
  lxa = (lxa - 128);
  lya = (lya - 127);
  rxa = (rxa - 128);
  rya = (rya - 127);


  if (start == 1 && start_ref == 0) {
    start_ref = 1;
  }
  if (start_ref == 1 && start == 0) {
    start_count++;
    start_ref = 0;
  }

  // head change to 45 degree

  if (start_count % 2 == 0) {

    if (red == 1 && red_ref == 0) {
      esc1.writeMicroseconds(1000);
      esc2.writeMicroseconds(1000);
      red_ref = 1;
      red_count++;
    }
    if (red == 0 && red_ref == 1) {
      red_ref = 0;
    }
    if(red_count == 1){
      esc1.writeMicroseconds(1000);
      esc2.writeMicroseconds(1000);
    }
    if(red_count == 2){
      esc1.writeMicroseconds(1300);
      esc2.writeMicroseconds(1300);
    }
    if(red_count == 3){
      esc1.writeMicroseconds(1500);
      esc2.writeMicroseconds(1500);
    }
     if(red_count == 4){
      esc1.writeMicroseconds(1600);
      esc2.writeMicroseconds(1600);
    }
    
  }



  if (error > -11 && error < 11) {
    Kp = 6.2;
    Ki = 0;
    Kd = 55;
  } else if (error <= -8 && error >= 8) {
    Kp = 2.2;
    Ki = 0;
    Kd = 45;
  }
  setpoint = 0;
  error = setpoint - yaw;
  integral = integral + error;
  derivative = error - preError;
  preError = error;



  PID = Kp * error + Ki * (integral) + Kd * (derivative);
  PID = -PID;

  resultant = sqrt(sq(lxa) + sq(lya));

  angle = atan2(-lya, lxa);
  Angle = angle * (180 / pi);

  V1 = resultant;
  V2 = resultant;
  V3 = resultant;
  V4 = resultant;

  M1 = V1 + PID;
  M2 = V2 + PID;
  M3 = V3 + PID;
  M4 = V4 + PID;

  M1 = constrain(M1, -120, 120);
  M2 = constrain(M2, -120, 120);
  M3 = constrain(M3, -120, 120);
  M4 = constrain(M4, -120, 120);


  motorSAB1.motor(motor1, -M1);
  motorSAB1.motor(motor2, M2);
  motorSAB2.motor(motor3, M3);
  motorSAB2.motor(motor4, -M4);


  // Serial.print(red_count);

  // Serial.print(Kp);
  // Serial.print(" ");
  //  Serial.print(blue_count);
  // Serial.print(" ");
  //   Serial.print(counter);
  //   Serial.print(" ");
  //   Serial.print(pre_counter);
  //   // Serial.print(Ki);
  //     Serial.print(" ");
  //   Serial.print(red_count);
  //   Serial.print(" ");
  //    Serial.print(blue_count);
  // Serial.print(red_count_pre);
  // Serial.print(" ");
  // Serial.print(Kd);
  // Serial.print(" ");
  // Serial.print(yellow_count);
  // Serial.print(red_count_pre);
  //  Serial.print(" ");
  // Serial.print("Error :");
  // Serial.print(error);
  Serial.print(" Yaw = ");
  Serial.print(" ");
  Serial.print(yaw);
  // Serial.print("  Angle ");
  // Serial.print(Angle);
  // // Serial.print("  Right X: ");
  // Serial.print(lxa);
  // Serial.print("   Right Y ");
  // Serial.print(lya);
  // Serial.print("  Resulant  ");
  // Serial.println(resultant);
  // Serial.print("Left X: ");
  // Serial.print(lxa);
  // Serial.print("   Left Y :  ");
  // Serial.print(lya);

  // Serial.print("    ");
  // Serial.print("  Blue : ");
  // Serial.print(blue);
  // Serial.print("  buttons :  ");
  // Serial.print(Buttons);
  // Serial.print("  yellow :  ");
  // Serial.print(yellow);
  // Serial.print("  red :  ");
  // Serial.print(red);
  // Serial.print("  green :  ");
  // // Serial.print(green);
  // Serial.print("    ");
  // Serial.print("  LB :  ");
  // Serial.print(L1);
  // Serial.print("  LT :  ");
  // Serial.print(L2);
  // Serial.print("  RB :  ");
  // Serial.print(R1);
  // Serial.print("  RT :  ");
  // Serial.print(R2);
  // Serial.print("  gpad :  ");
  // Serial.print(gpad);
  // Serial.print("    ");
  // Serial.print("  back :  ");
  // Serial.print(back);
  // Serial.print("  start :  ");
  // Serial.print(start);
  // Serial.print("    ");
  // Serial.print("  left joy :  ");
  // Serial.print(leftjoy);
  // Serial.print("  rightjoy :  ");
  // // Serial.println(rightjoy);
  //  Serial.print("Setpoint :");
  // Serial.print(setpoint);
  Serial.println(" ");
}

