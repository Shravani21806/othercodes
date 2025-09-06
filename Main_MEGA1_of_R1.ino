#include <Wire.h>
#include <MPU6050.h>
MPU6050 mpu;
// #include <ServoTimer2.h>
unsigned long timer = 0;
float timeStep = 0.01;
float yaw = 0;

#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>
SoftwareSerial SWSerial1(NOT_A_PIN, 8);
SabertoothSimplified ST1(SWSerial1);
SoftwareSerial SWSerial2(NOT_A_PIN, 13);
SabertoothSimplified ST2(SWSerial2);
SoftwareSerial SWSerial3(NOT_A_PIN, 7);
SabertoothSimplified ST3(SWSerial3);


SoftwareSerial mySerial(11, 10);
// ServoTimer2 esc1;
// ServoTimer2 esc2;

int Ax, Bx, Y, X, LX, LY, RX, RY, Dpad, LT, RT, LB, RB;
int START;
int BACK;

float R, theta, M, count1 = 0, count2 = 0, count3 = 0, count4 = 0, h = 0, j = 0, k = 0, l = 0, setpoint = 0, pre_error = 0, proportional, integral = 0, derivative, error, pid, L1, L2, a, b, c, d, A, B, C, D;
float kp = 0, ki = 0, kd = 0;
int LT_ref = 0;
int RT_ref = 0;
int LB_ref = 0;
int RB_ref = 0;
float motor_A = 0;
float motor_B = 0;
float motor_C = 0;
float motor_D = 0;
int speed_arm = 0;
int Dpad_ref1 = 0;
int Dpad_ref2 = 0;
int Dpad_ref3 = 0;
int Dpad_ref4 = 0;
int escPin1 = 11;
int escPin2 = 12;

int Bx_ref1 = 0;
int Bx_ref = 0;
int X_ref = 0;
int X_ref1 = 0;
int Y_ref1 = 0;
int Ax_ref1 = 0;
int START_ref = 0;
int BACK_ref = 0;
bool solenoid_state = LOW;
bool solenoid_state_dribble = LOW;
bool solenoid_state_angle = LOW;
int solenoid_dribbling = 24;  // Pin 2 of Arduino connected to Pin 1 (IN1) of ULN2003
int solenoid_angle = 27;
int solenoid_base = 26;
// #define ESC_PIN1 5
// #define ESC_PIN2 8
// Function to extract values from received data
int getValue(String data, String key) {
  int startIndex = data.indexOf(key);
  if (startIndex == -1) return 0;  // Return 0 if key not found
  startIndex += key.length();
  int endIndex = data.indexOf(',', startIndex);  // Assume values are comma-separated
  if (endIndex == -1) endIndex = data.length();
  return data.substring(startIndex, endIndex).toInt();
}

void setup() {

  mySerial.begin(115200);
  SWSerial1.begin(9600);
  SWSerial2.begin(9600);
  SWSerial3.begin(9600);

  Serial.begin(115200);  // Serial Monitor for debugging
  Serial1.begin(9600);   // UART1 (RX1: Pin 19, TX1: Pin 18) from ESP32

  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  mpu.calibrateGyro();
  mpu.setThreshold(1);

  Serial.println("Arduino Mega Ready. Waiting for data...");



  // esc1.attach(ESC_PIN1);
  // esc2.attach(ESC_PIN2);

  // Arm ESC by sending a low signal for 3 seconds
  // esc1.write(1000);
  // esc2.write(1000);
  // delay(3000);

  pinMode(solenoid_dribbling, OUTPUT);
  pinMode(solenoid_base, OUTPUT);
  pinMode(solenoid_angle, OUTPUT);
}

void loop() {
  timer = millis();
  Vector norm = mpu.readNormalizeGyro();
  yaw = yaw + norm.ZAxis * timeStep;
  delay((timeStep * 1000) - (millis() - timer));

  if (Serial1.available()) {
    String receivedData = Serial1.readStringUntil('\n');  // Read data until newline
    Serial.print("Received: " + receivedData);            // Print received data

    // Parsing received data
    Ax = getValue(receivedData, "A:");
    Bx = getValue(receivedData, "B:");
    X = getValue(receivedData, "X:");
    Y = getValue(receivedData, "Y:");
    LX = getValue(receivedData, "LX:");
    LY = getValue(receivedData, "LY:");
    RX = getValue(receivedData, "RX:");
    RY = getValue(receivedData, "RY:");
    RB = getValue(receivedData, "RB:");
    LB = getValue(receivedData, "LB:");
    RT = getValue(receivedData, "RT:");
    LT = getValue(receivedData, "LT:");
    Dpad = getValue(receivedData, "Dpad:");
    START = getValue(receivedData, "START:");
    BACK = getValue(receivedData, "BACK:");


    String dataString = String(Dpad);
    mySerial.println(dataString);
    delay(5);
  }

  // arm
  if (RY > 2) {
    speed_arm = 30;
  }
  if (RY < 2) {
    speed_arm = -30;
  }
  if (RY == 0) {
    speed_arm = 0;
  }

  // CONTINUOUS ROTATION
  if (RT > 0) {
    setpoint = setpoint - 0.2;
  }
  if (LT > 0) {
    setpoint = setpoint + 0.2;
  }

  // 90 DEGREES TURN
  if (RB == 1 && RB_ref == 0) {
    RB_ref = 1;
  }
  if (RB == 0 && RB_ref == 1) {
    setpoint = setpoint + 90;
    RB_ref = 0;
  }
  if (LB == 1 && LB_ref == 0) {
    LB_ref = 1;
  }
  if (LB == 0 && LB_ref == 1) {
    setpoint = setpoint - 90;
    LB_ref = 0;
  }
 
  //for Base
  if (Ax == 1 && Ax_ref1 == 0) {
    solenoid_state = !solenoid_state;
    digitalWrite(solenoid_base, solenoid_state);
    Ax_ref1 = 1;
  }
  if (Ax == 0 && Ax_ref1 == 1) {
    // Serial.print("...............");
    Ax_ref1 = 0;
  }

  //angle
  if (Y == 1 && Y_ref1 == 0) {
       solenoid_state_angle = !solenoid_state_angle;
    digitalWrite(solenoid_angle, solenoid_state_angle);
    Y_ref1 = 1;
  }
  if (Y == 0 && Y_ref1 == 1) {
    Y_ref1 = 0;
  }

  //for Dribbling
  if (Bx == 1 && Bx_ref1 == 0) {
    solenoid_state_dribble = !solenoid_state_dribble;
    digitalWrite(solenoid_dribbling, solenoid_state_dribble);
    Bx_ref1 = 1;
  }
  if (Bx == 0 && Bx_ref1 == 1) {
    // Serial.print("############");
    Bx_ref1 = 0;
  }
  //  angle
  if (START == 1 && START_ref == 0) {
    START_ref = 1;
  }
  if (START == 0 && START_ref == 1) {
    digitalWrite(solenoid_angle, HIGH);
    Serial.print("angle high");
    START_ref = 0;
  }
  if (BACK == 1 && BACK_ref == 0) {
    BACK_ref = 1;
  }
  if (BACK == 0 && BACK_ref == 1) {
    digitalWrite(solenoid_angle, LOW);
    Serial.print("angle low");
    BACK_ref = 0;
  }

  if (error >= 5 && error <= -5) {
    kp = 30;
    kd = 50;
    ki = 0;
  } else {
    kp = 25;
    kd = 48;
    ki = 0;

    // Ensure joystick values are mapped after extraction
    // LX = map((-LX), -255, 255, +127, -127);
    // LY = map((-LY), -255, 255, +127, -127);

    R = sqrt((LX * LX) + (LY * LY));
    theta = atan2(LY, LX) * 180 / 3.142;

    a = R * sin((theta - 45) * 3.142 / 180);
    c = R * sin((135 - theta) * 3.142 / 180);
    b = R * sin((135 - theta) * 3.142 / 180);
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
    C = c + pid;
    D = d - pid;

    motor_A = constrain(A, -120, 120);
    motor_B = constrain(B, -120, 120);
    motor_C = constrain(C, -120, 120);
    motor_D = constrain(D, -120, 120);

    ST1.motor(1, motor_A);
    ST1.motor(2, motor_B);
    ST2.motor(2, motor_C);
    ST2.motor(1, motor_D);

    ST3.motor(1, speed_arm);

    // esc1.write(count1);
    // esc2.write(count2);
    // esc1.writeMicroseconds(count1);
    // esc2.writeMicroseconds(count2);
    Serial.print("   error: ");
    Serial.println(error);
    //   Serial.print("   count: ");
    // Serial.print(count1);
    // Serial.print("   count2: ");
    // Serial.print(count2);
    // Serial.print("   setpoint: ");
    // Serial.println(setpoint);
    // Serial.print("     ly: ");
    // Serial.println(LY);
  }
}   