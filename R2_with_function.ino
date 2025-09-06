#include <PS2X_lib.h>
PS2X ps2x;
#include <Wire.h>
#include <MPU6050.h>
#include "CytronMotorDriver.h"
MPU6050 mpu;
#include <Servo.h>
//esc
Servo esc1;
Servo esc2;
int escpin1 = 10;
int escpin2 = 11;
int count1;  // BLDC motor speed 1
int count2;  // BLDC motor speed 2
//cytron
CytronMD motor1(PWM_DIR, 3, 48);  // PWM 1 = Pin 3, DIR 1 = Pin 4.
CytronMD motor2(PWM_DIR, 2, 49);
CytronMD motor3(PWM_DIR, 7, 47);  // PWM 1 = Pin 3, DIR 1 = Pin 4.
CytronMD motor4(PWM_DIR, 6, 46);
CytronMD motor5(PWM_DIR, 4, 44);
//imu
unsigned long timer = 0;
float timeStep = 0.00999;
float yaw = 0;
//variables 
float LY_mapped = 0;
float LX_mapped = 0;
float RY_mapped = 0;
float RX_mapped = 0;
int L1, L2, R1, R2;
float LY, LX, RY, RX;
int BLUE, FORWARD, BACKWARD, RIGHT, LEFT;
int GREEN = 0, RED = 0;
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
int motor_E = 0;

///solenoid
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
bool solenoid_state_net = LOW;

int solenoid_dribbling = 40;  //41
int solenoid_angle = 37;   //40
int solenoid_base = 41;       
int solenoid_net = 39;        

bool field = false;

void setup() {
 //imu
  Serial.begin(57600);
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  mpu.calibrateGyro();
  mpu.setThreshold(1);
  //ps2
  // int error = ps2x.config_gamepad(30, 31, 32, 33, false, false);
  int error = ps2x.config_gamepad(22, 23, 24, 25, false, false);
  if (error == 0) {
    Serial.println("PS2 Controller ready!");
  } else {
    Serial.print("Controller Error: ");
    Serial.println(error);
  }
  //esc
  esc1.attach(escpin1);
  esc2.attach(escpin2);
  count1 = 1000;  
  count2 = 1000;

  pinMode(solenoid_dribbling, OUTPUT);
  pinMode(solenoid_base, OUTPUT);
  pinMode(solenoid_angle, OUTPUT);
  pinMode(solenoid_net, OUTPUT);


}

void loop() {
  Vector norm = mpu.readNormalizeGyro();
  yaw = yaw + norm.ZAxis * timeStep;
  ps2x.read_gamepad(false, 0);

  L1 = ps2x.Button(PSB_L1);
  R1 = ps2x.Button(PSB_R1);
  L2 = ps2x.Button(PSB_L2);
  R2 = ps2x.Button(PSB_R2);

  if (L1 == 1) {
    motor_E = -100;
    //  motor5.setSpeed(100);
  } else if (R1 == 1) {
    motor_E = 100;
    // motor5.setSpeed(-100);
  } else {
    motor_E = 0;
  }
  motor5.setSpeed(motor_E);
  LY = ps2x.Analog(PSS_LY);
  LX = ps2x.Analog(PSS_LX);
  RY = ps2x.Analog(PSS_RY);
  RX = ps2x.Analog(PSS_RX);

  RED = ps2x.Button(PSB_RED);
  GREEN = ps2x.Button(PSB_GREEN);
  BLUE = ps2x.Button(PSB_BLUE);

  FORWARD = ps2x.ButtonPressed(PSB_PAD_UP);
  BACKWARD = ps2x.ButtonPressed(PSB_PAD_DOWN);
  RIGHT = ps2x.ButtonPressed(PSB_PAD_RIGHT);
  LEFT = ps2x.ButtonPressed(PSB_PAD_LEFT);

  LY_mapped = map(constrain(LY, 0, 255), 0, 255, 127, -127);
  LX_mapped = map(constrain(LX, 0, 255), 0, 255, -127, 127);
  RY_mapped = map(constrain(RY, 0, 255), 0, 255, 127, -127);
  RX_mapped = map(constrain(RX, 0, 255), 0, 255, -127, 127);

  if (abs(LY_mapped) < 5) LY_mapped = 0;
  if (abs(LX_mapped) < 5) LX_mapped = 0;
  if (LY_mapped > 5) LY_mapped -= 5;
  if (LY_mapped < -5) LY_mapped += 5;

    if (R2 > 0) {
    setpoint += 0.2;
  }
  if (L2 > 0) {
    setpoint -= 0.2;
  }
  // kp = 7;  //45
  // kd = 83;   //65.5
  // ki = 0;
  kp = 20;    //45//16
  kd = 42.5;  //65.5//30
  ki = 0;
  locomotion();
  solenoid();
  bldc();
  Serial.print("yaw = ");
  Serial.print(yaw);
  Serial.println("");
  delay(10);
}


void locomotion(){
  
    R = sqrt((LX_mapped * LX_mapped) + (LY_mapped * LY_mapped));
  theta = atan2(LY_mapped, -LX_mapped) * 180 / 3.142;

  a = R * sin((theta - 45) * 3.142 / 180);
  b = R * sin((135 - theta) * 3.142 / 180);
  d = R * sin((135 - theta) * 3.142 / 180);
  c = R * sin((theta - 45) * 3.142 / 180);
  error = setpoint - yaw;
  proportional = error;
  integral += error;
  derivative = error - pre_error;
  pre_error = error;
  pid = kp * proportional + ki * integral + kd * derivative;

  A = a + pid;
  B = b - pid;
  C = -c + pid;
  D = -d - pid;

  motor_A = constrain(A, -150, 150);
  motor_B = constrain(B, -150, 150);
  motor_C = constrain(C, -150, 150);
  motor_D = constrain(D, -150, 150);
  motor1.setSpeed(motor_A * 1.2);
  motor2.setSpeed(motor_B * 1.2);
  motor3.setSpeed(motor_C * 1.2);
  motor4.setSpeed(motor_D * 1.2);
}


void solenoid(){
  // base
  if (RED == 1 && c_ref1 == 0) {
    solenoid_state = !solenoid_state;
    digitalWrite(solenoid_base, solenoid_state);
    c_ref1 = 1;
  }
  if (RED == 0 && c_ref1 == 1) {
    Serial.print(" %%% ");
    c_ref1 = 0;
  }

  // dribble
  if (BLUE == 1 && t_ref1 == 0) {
    solenoid_state_dribble = !solenoid_state_dribble;
    digitalWrite(solenoid_dribbling, solenoid_state_dribble);
    t_ref1 = 1;
  }
  if (BLUE == 0 && t_ref1 == 1) {
    Serial.print(" ##### ");
    t_ref1 = 0;
  }

  // angle
  if (GREEN == 1 && s_ref1 == 0) {
    solenoid_state_angle = !solenoid_state_angle;
    digitalWrite(solenoid_angle, solenoid_state_angle);
    s_ref1 = 1;
  }
  if (GREEN == 0 && s_ref1 == 1) {
    Serial.print(" $$$$$ ");
    s_ref1 = 0;
  }
  //   //net
  //   if (R1 == 1 && R1_ref1 == 0) {
  //     solenoid_state_net = !solenoid_state_net;
  //     digitalWrite(solenoid_net, solenoid_state_net);
  //     R1_ref1 = 1;
  //   }
  //   if (R1 == 0 && R1_ref1 == 1) {
  //     Serial.print(" %%% ");
  //     R1_ref1 = 0;
  //  }
}

void bldc(){
  
  // BLDC control logic

  if (RIGHT == 1) {
    count1 = 1000;  // calibration speed
    count2 = 1000;
  }

  if (FORWARD == 1) {
    count1 += 50;  // Increase speed
    count2 += 50;
  }

  if (BACKWARD == 1) {
    count1 -= 50;  // Decrease speed
    count2 -= 50;
  }
  // Dribbling mode
  if (LEFT == 1) {
    count1 =1088;  //upper 1107//1105//*1104* [1106&&1220]  [1102&&1222] [1098&&1220]
    count2 = 1200;  // LOWER //1250//1270//*1260**1250**1/05[1087&&1200]**
  }

  //  Ensure the speed values remain within valid range
  count1 = constrain(count1, 1000, 2000);
  count2 = constrain(count2, 1000, 2000);

  // Apply updated speeds to ESCs
  esc1.writeMicroseconds(count1);
  esc2.writeMicroseconds(count2); 
}
