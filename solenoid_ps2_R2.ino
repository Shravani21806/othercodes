#include <Wire.h>
#include <MPU6050.h>
MPU6050 mpu;
unsigned long timer = 0;
float timeStep = 0.01;
float yaw = 0;
#include <PS2X_lib.h>
PS2X ps2x;
#include "CytronMotorDriver.h"
CytronMD motor1(PWM_DIR, 3, 46);  // PWM 1 = Pin 3, DIR 1 = Pin 4.
CytronMD motor2(PWM_DIR, 2, 47);
CytronMD motor3(PWM_DIR, 7, 50);  // PWM 1 = Pin 3, DIR 1 = Pin 4.
CytronMD motor4(PWM_DIR, 6, 51);
CytronMD motor5(PWM_DIR, 5, 53); //5 50 // PWM 1 = Pin 3, DIR 1 = Pin 4.
// CytronMD motor6(PWM_DIR, 6, 52);
float LY_mapped = 0;
float LX_mapped = 0;
float RY_mapped = 0;
float RX_mapped = 0;
int BLUE, FORWARD, BACKWARD, RIGHT, LEFT;
int GREEN = 0, RED = 0;
float L1, L2, R1, R2, LY, LX, RY, RX;
float M1 = 0;
float M2 = 0;
float M3 = 0;
float M4 = 0;
float R, theta, M, count3 = 0, count4 = 0, h = 0, j = 0, k = 0, l = 0, setpoint = 0, pre_error = 0, proportional, integral = 0, derivative, error, pid, a, b, c, d, A, B, C, D;
float kp = 0, ki = 0, kd = 0;
float motor_A = 0;
float motor_B = 0;
float motor_C = 0;
float motor_D = 0;
int R1_ref = 0;
int R2_ref = 0;
int L1_ref = 0;
int L2_ref = 0;
int c_ref1 = 0;
int t_ref1 = 0;
int s_ref1 = 0;
int R1_ref1 = 0;
bool solenoid_state = LOW;
bool solenoid_state_dribble = LOW;
bool solenoid_state_angle = LOW;
bool solenoid_state_net=LOW;

int solenoid_dribbling = 40 ;  // Pin 2 of Arduino connected to Pin 1 (IN1) of ULN2003
int solenoid_angle = 38 ;
int solenoid_base = 39 ;
int solenoid_net=41;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);

  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  mpu.calibrateGyro();
  mpu.setThreshold(1);

  // CHANGES for v1.6 HERE!!! ************PAY ATTENTION***********
int ERROR = ps2x.config_gamepad(30, 31, 32, 33, true, true);
  // if (error == 0) {
  //   Serial.print("✅ PS2 Controller Connected!");
  // } else {
  //   Serial.print("❌ Failed to connect PS2 Controller.");
  //   while (1)
  //     ;
  // }
  pinMode(solenoid_dribbling, OUTPUT);
  pinMode(solenoid_base, OUTPUT);
  pinMode(solenoid_angle, OUTPUT);
  pinMode(solenoid_net,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  // timer = millis();
  Vector norm = mpu.readNormalizeGyro();
  yaw = yaw + norm.ZAxis * timeStep;

  ps2x.read_gamepad();
  L1 = ps2x.Button(PSB_L1);
  R1 = ps2x.Button(PSB_R1);
  L2 = ps2x.Button(PSB_L2);
  R2 = ps2x.Button(PSB_R2);

  LY = ps2x.Analog(PSS_LY);
  LX = ps2x.Analog(PSS_LX);
  RY = ps2x.Analog(PSS_RY);
  RX = ps2x.Analog(PSS_RX);

  RED = ps2x.Button(PSB_RED);
  GREEN = ps2x.Button(PSB_GREEN);
  BLUE = ps2x.Button(PSB_BLUE);
  PINK = ps2x.Button(PSB_PINK);
  // Read D-pad directions
  int FORWARD = ps2x.Button(PSB_PAD_UP) ? 1 : 0;
  int BACKWARD = ps2x.Button(PSB_PAD_DOWN) ? 1 : 0;
  int RIGHT = ps2x.Button(PSB_PAD_RIGHT) ? 1 : 0;
  int LEFT = ps2x.Button(PSB_PAD_LEFT) ? 1 : 0;
  // LY=LY-128;
  // LX=LX-128;
  LY_mapped = constrain(LY, 0, 255);
  LX_mapped = constrain(LX, 0, 255);
  RY_mapped = constrain(RY, 0, 255);
  RX_mapped = constrain(RX, 0, 255);
  LY_mapped = map(LY_mapped, 0, 255, 127, -127);
  LX_mapped = map(LX_mapped, 0, 255, -127, 127);
  RY_mapped = map(RY_mapped, 0, 255, 127, -127);
  RX_mapped = map(RX_mapped, 0, 255, -127, 127);
  if (LY_mapped >= -5 && LY_mapped <= 5) {
    LY_mapped = 0;
  }
  if (LY_mapped > 5) {
    LY_mapped -= 5;
  }
  if (LY_mapped < -5) {
    LY_mapped += 5;
  }
    if (RX_mapped >= 5) {
    motor5.setSpeed(-70);
  }
  if (RX_mapped < -5) {
    motor5.setSpeed(70);
  }
  if (RX_mapped == 0) {
    motor5.setSpeed(0);
  }

  // Continuous Rotation Control
  if (R2 > 0) {
    setpoint -= 0.2;
  }
  if (L2 > 0) {
    setpoint += 0.2;
  }

  // 90° Turns
  // if (R1 == 1 && R1_ref == 0) {
  //   R1_ref = 1;
  // }

  // if (R1 == 0 && R1_ref == 1) {
  //   setpoint = setpoint + 90;
  //   R1_ref = 0;
  // }

  // if (L1 == 1 && L1_ref == 0) {
  //   L1_ref = 1;
  // }

  // if (L1 == 0 && L1_ref == 1) {
  //   setpoint = setpoint + 90;
  //   L1_ref = 0;
  // }
  
  if (RED == 1 && c_ref1 == 0) {
    solenoid_state = !solenoid_state;
    digitalWrite(solenoid_base, solenoid_state);
    c_ref1 = 1;
  }
  if (  RED == 0 && c_ref1 == 1) {
    Serial.print(" %%% ");
    c_ref1 = 0;
  }
 
  //  // Base Solenoid Control
  if (BLUE == 1 && t_ref1 == 0) {
    solenoid_state_dribble = !solenoid_state_dribble;
    digitalWrite(solenoid_dribbling, solenoid_state_dribble);
    t_ref1 = 1;
  }
  if (BLUE == 0 && t_ref1 == 1) {
    Serial.print(" ##### ");
    t_ref1 = 0;

  }

  if (GREEN == 1 && s_ref1 == 0) {
    solenoid_state_angle = !solenoid_state_angle;
    digitalWrite(solenoid_angle, solenoid_state_angle);
    s_ref1 = 1;
  }
  if (GREEN == 0 && s_ref1 == 1) {
    Serial.print(" $$$$$ ");
    s_ref1 = 0;
  }
if (R1 == 1 && R1_ref1 == 0) {
    solenoid_state_net = !solenoid_state_net;
    digitalWrite(solenoid_net, solenoid_state_net);
    R1_ref1 = 1;
  }
  if (  R1 == 0 && R1_ref1 == 1) {
    Serial.print(" %%% ");
    R1_ref1 = 0;
  }

 kp = 48;  //45
  kd = 65;  //65.5
  ki = 0.001;
   R = sqrt((LX_mapped * LX_mapped) + (LY_mapped * LY_mapped));
  theta = atan2(LY_mapped, -LX_mapped) * 180 / 3.142;

  a = R * sin((theta - 45) * 3.142 / 180);
  b = R * sin((135 - theta) * 3.142 / 180);
  c = R * sin((135 - theta) * 3.142 / 180);
  d = R * sin((theta - 45) * 3.142 / 180);

  // PID control adjustments
  error = setpoint - yaw;
  proportional = error;
  integral += error;
  derivative = error - pre_error;
  pre_error = error;
  pid = kp * proportional + ki * integral + kd * derivative;

  A = a + pid;
  B = b - pid;
  C = -c - pid;
  D = -d + pid;

  motor_A = constrain(A, -127, 127);
  motor_B = constrain(B, -127, 127);
  motor_C = constrain(C, -127, 127);
  motor_D = constrain(D, -127, 127);
    motor1.setSpeed(motor_A);
  motor2.setSpeed(motor_B);
  motor3.setSpeed(motor_C);
  motor4.setSpeed(motor_D);
  // motor5.setSpeed(10);
  Serial.print("     LY = ");
  Serial.print(LY_mapped);
  Serial.print("     LX = ");
  Serial.print(LX_mapped);
  Serial.print("     motor_A= ");
  Serial.print(motor_A);
  Serial.print("     solenoid_state_angle= ");
  Serial.print(solenoid_state_angle);
    Serial.print("     solenoid_dribble= ");
  Serial.print(solenoid_state_dribble);
    Serial.print("     solenoid_state_net= ");
  Serial.print(solenoid_state_net);
     Serial.print("     pid = ");
  Serial.print(pid);
  Serial.print("     Yaw = ");
  Serial.print(yaw);
  Serial.println();
}
