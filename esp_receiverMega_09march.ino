#include <Wire.h>
#include <MPU6050.h>
MPU6050 mpu;
unsigned long timer = 0;
float timeStep = 0.01;
float yaw = 0;

#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>
SoftwareSerial SWSerial1(NOT_A_PIN, 2);
SabertoothSimplified ST1(SWSerial1);
SoftwareSerial SWSerial2(NOT_A_PIN, 3);
SabertoothSimplified ST2(SWSerial2);
SoftwareSerial SWSerial3(NOT_A_PIN, 4);
SabertoothSimplified ST3(SWSerial3);

int Ax, Bx, Y, X, LX, LY, RX, RY, Dpad, LT, RT, LB, RB;

float R, theta, M, count1 = 0, count2 = 0, count3 = 0, count4 = 0, h = 0, j = 0, k = 0, l = 0, setpoint = 0, pre_error = 0, proportional, integral = 0, derivative, error, pid, L1, L2, a, b, c, d, A, B, C, D;
float kp = 0, ki = 0, kd = 0;
int red_ref = 0;
int L1_ref = 0;
int R1_ref = 0;
float motor_A = 0;
float motor_B = 0;
float motor_C = 0;
float motor_D = 0;

void setup() {
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
}

void loop() {
  timer = millis();
  Vector norm = mpu.readNormalizeGyro();
  yaw = yaw + norm.ZAxis * timeStep;
  delay((timeStep * 1000) - (millis() - timer));

  if (Serial1.available()) {
    String receivedData = Serial1.readStringUntil('\n');  // Read data until newline
    Serial.println("Received: " + receivedData);          // Print received data

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
  }

  // PID control adjustments
  error = setpoint - yaw;
  if (abs(error) >= 5) {
    kp = 5;
    kd = 20;
    ki = 0.0001;
  } else {
    kp = 30;
    kd = 90;
    ki = 0.001;
  }

  // Ensure joystick values are mapped after extraction
  LX = map(LX, -255, 255, -127, 127);
  LY = map(LY, -255, 255, -127, 127);

  R = sqrt((LX * LX) + (LY * LY));
  theta = atan2(LY, LX) * 180 / 3.142;

  a = R * sin((theta - 45) * 3.142 / 180);
  b = R * sin((135 - theta) * 3.142 / 180);
  d = R * sin((135 - theta) * 3.142 / 180);
  c = R * sin((theta - 45) * 3.142 / 180);

  proportional = error;
  integral += error;
  derivative = error - pre_error;
  pre_error = error;
  pid = kp * proportional + ki * integral + kd * derivative;

  A = a + pid;
  B = b - pid;
  C = c - pid;
  D = -d - pid;

  motor_A = constrain(A, -120, 120);
  motor_B = constrain(B, -120, 120);
  motor_C = constrain(C, -120, 120);
  motor_D = constrain(D, -120, 120);

  ST1.motor(1, motor_A);
  ST1.motor(2, motor_B);
  ST2.motor(2, motor_C);
  ST2.motor(1, motor_D);

  Serial.print("   Error: ");
  Serial.print(error);
  Serial.print("     Yaw: ");
  Serial.println(yaw);
}

// Function to extract values from received data
int getValue(String data, String key) {
  int startIndex = data.indexOf(key);
  if (startIndex == -1) return 0;  // Return 0 if key not found
  startIndex += key.length();
  int endIndex = data.indexOf(',', startIndex); // Assume values are comma-separated
  if (endIndex == -1) endIndex = data.length();
  return data.substring(startIndex, endIndex).toInt();
}
