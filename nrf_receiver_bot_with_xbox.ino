#include <SPI.h>
// #include <nRF24L01.h>
#include <RF24.h>
#define CE_PIN 23
#define CSN_PIN 22
RF24 radio(CE_PIN, CSN_PIN);
const uint64_t address = 0xF0F0F0F0E1LL;  // Match transmitter
struct XboxData {
  int LX, LY, RX, RY;
  bool A, B, X, Y, LB, RB, LT, RT, L3, R3, START, BACK, XBOX, UP, DOWN, LEFT, RIGHT;
};
#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>
SoftwareSerial SWSerial1(NOT_A_PIN, 12);
SabertoothSimplified ST1(SWSerial1);
SoftwareSerial SWSerial2(NOT_A_PIN, 13);
SabertoothSimplified ST2(SWSerial2);
SoftwareSerial SWSerial3(NOT_A_PIN, 7);
SabertoothSimplified ST3(SWSerial3);


#include <SoftwareSerial.h>

SoftwareSerial mySerial(11, 10);  // (RX, TX) - TX is unused

#include <Wire.h>
#include <MPU6050.h>


MPU6050 mpu;
unsigned long timer = 0;
float timeStep = 0.01;
float yaw = 0;



float R, theta, M, count1 = 0, setpoint = 0, pre_error, proportional, integral, derivative, error, pid, L1, L2, R1, R2, LY, LX, a, b, c, d, A, B, C, D;
float kp = 0, ki = 0, kd = 0;
int RB_ref = 0;
int LB_ref = 0;
int Xx_ref = 0;
int R3_ref = 0;
int up_ref = 0;
int down_ref = 0;
int right_ref = 0;

float motor_A = 0;
float motor_B = 0;
float motor_C = 0;
float motor_D = 0;

int start_count = 0;
int start_ref = 0;
int RY, RX, Ax, Bx, Xx, Yx, start, back, up, down, left, right, RB, LB, L3, R3, LT, RT;

int solenoid_base = 27;  // Pin 2 of Arduino connected to Pin 1 (IN1) of ULN2003
int solenoid_dribble = 25;
int solenoid_angle = 26;
int Bx_ref = 0;
int Bx_ref1 = 0;
int Xx_ref1 = 0;
int left_ref1 = 0;

int speed_arm = 0;

unsigned long previousMillis = 0;
const long interval = 10;

void setup() {

  Serial.begin(115200);

  radio.begin();
  radio.setChannel(100);          // ✅ Match the same channel as transmitter
  radio.setDataRate(RF24_2MBPS);  // ✅ Match the same data rate
  radio.setPALevel(RF24_PA_MAX);  // ✅ Maximum power
  radio.openReadingPipe(1, address);
  radio.startListening();

  SWSerial1.begin(9600);
  SWSerial2.begin(9600);
  SWSerial3.begin(9600);


  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    // delay(500);
  }
  mpu.calibrateGyro();
  mpu.setThreshold(1);

  pinMode(solenoid_angle, OUTPUT);
  pinMode(solenoid_base, OUTPUT);
  pinMode(solenoid_dribble, OUTPUT);
}

void loop() {
  XboxData receivedData;
  if (radio.available()) {

    radio.read(&receivedData, sizeof(receivedData));

    LX = receivedData.LX;
    LY = receivedData.LY;
    RX = receivedData.RX;
    RY = receivedData.RY;
    Ax = receivedData.A;
    Bx = receivedData.B;
    Xx = receivedData.X;
    Yx = receivedData.Y;
    start = receivedData.START;
    back = receivedData.BACK;
    up = receivedData.UP;
    down = receivedData.DOWN;
    left = receivedData.LEFT;
    right = receivedData.RIGHT;
    LB = receivedData.LB;
    RB = receivedData.RB;
    R3 = receivedData.R3;
    L3 = receivedData.L3;
    RT = receivedData.RT;
    LT = receivedData.LT;

    String dataString = String(up) + "," + String(down) + "," + String(right);
    mySerial.println(dataString);
    delay(1);
  }

  //  Serial.print("UP: ");
  //     Serial.print(up);
  //     Serial.print(", DOWN: ");
  //     Serial.print(down);
  //     Serial.print(", RIGHT: ");
  //     Serial.print(right);
  //
  timer = millis();
  Vector norm = mpu.readNormalizeGyro();
  yaw = yaw + norm.ZAxis * timeStep;
  if (millis() - previousMillis >= interval) {
    previousMillis = millis();
  }
  // // CLAW ROTATION
  if (Yx == 1 && Ax == 0) {
    ST3.motor(1, 75);
  }
  if (Ax == 1 && Yx == 0) {
    ST3.motor(1, -75);
  }
  if (Yx == 0 && Ax == 0) {
    ST3.motor(1, 0);
  }

  // claw rotation
  if (RX > 0) {
    ST3.motor(1, 20);
  }
  if (RX < 0) {
    ST3.motor(1, -20);
  }
  if (RX == 0 ) {
    ST3.motor(1, 0);
  }


  if (start == 1 && start_ref == 0) {
    start_ref = 1;
  }

  if (start_ref == 1 && start == 0) {
    start_count++;
    start_ref = 0;
  }
  if (start_count % 2 != 0) {
    // CONTINUOUS ROTATION
    if (LT == 1) {
      setpoint = setpoint + 0.5;
    }
    if (RT == 1) {
      setpoint = setpoint - 0.5;
    }
    // 90 DEGREES TURN
    if (LB == 1 && LB_ref == 0) {
      LB_ref = 1;
    }
    if (LB == 0 && LB_ref == 1) {
      setpoint = setpoint + 90;
      LB_ref = 1;
    }
    if (RB == 1 && RB_ref == 0) {
      RB_ref = 1;
    }
    if (RB == 0 && RB_ref == 1) {
      setpoint = setpoint - 90;
      RB_ref = 1;
    }
  }

  // solenoid
  if (start_count % 2 == 0) {
    if (Bx == 1 && Bx_ref1 == 0) {
      Bx_ref1 = 1;
    }
    if (Bx == 0 && Bx_ref1 == 1) {
      digitalWrite(solenoid_base, HIGH);
      Bx_ref1 = 2;
    }
    if (Bx == 1 && Bx_ref1 == 2) {
      digitalWrite(solenoid_base, LOW);
      Bx_ref1 = 0;
    }

    if (Xx == 1 && Xx_ref1 == 0) {
      Xx_ref1 = 1;
    }
    if (Xx == 0 && Xx_ref1 == 1) {
      digitalWrite(solenoid_dribble, HIGH);
      Xx_ref1 = 2;
    }
    if (Xx == 1 && Xx_ref == 2) {
      digitalWrite(solenoid_dribble, LOW);
      Xx_ref1 = 0;
    }

    if (left == 1 && left_ref1 == 0) {
      left_ref1 = 1;
    }
    if (left == 0 && left_ref1 == 1) {
      digitalWrite(solenoid_angle, HIGH);
      left_ref1 = 2;
    }
    if (left == 1 && left_ref1 == 2) {
      digitalWrite(solenoid_angle, LOW);
      left_ref1 = 0;
    }
  }


  if (error >= 5 && error <= -5) {
    kp = 9, kd = 20, ki = 0.0001;
  } else {
    kp = 31, kd = 90;
    ki = 0.001;
  }


  LX = map(LX, -255, 255, -127, 127);
  LY = map(LY, -255, 255, -127, 127);

  R = sqrt((LX * LX) + (LY * LY));
  theta = atan2(LY, LX);
  theta = theta * 180 / 3.142;

  a = R * sin((theta - 45) * 3.142 / 180);
  b = R * sin((135 - theta) * 3.142 / 180);
  c = R * sin((135 - theta) * 3.142 / 180);
  d = R * sin((theta - 45) * 3.142 / 180);


  error = setpoint - yaw;
  proportional = error;
  integral = integral + error;
  derivative = error - pre_error;
  pre_error = error;
  pid = kp * proportional + ki * integral + kd * derivative;

  A = a - pid;
  B = b + pid;
  C = c - pid;
  D = d + pid;
  motor_A = constrain(A, -120, 120);
  motor_B = constrain(B, -120, 120);
  motor_C = constrain(C, -120, 120);
  motor_D = constrain(D, -120, 120);
  ST1.motor(1, motor_A);
  ST1.motor(2, motor_B);
  ST2.motor(2, motor_C);
  ST2.motor(1, motor_D);


  // esc1.writeMicroseconds(count1);
  // esc2.writeMicroseconds(count1);

  Serial.print("   LX  ");
  Serial.print(LX);
  Serial.print("   LY ");
  Serial.print(LY);

  // // // Serial.print("   up ");
  // // Serial.print(Ax);
  Serial.print("   Y ");
  Serial.print(Yx);
  Serial.print("   A ");
  Serial.println(Ax);
  // Serial.print("   left ");
  // Serial.print(left);
  // Serial.print("   right ");
  // Serial.print(right);
  // Serial.print("   error  ");
  // Serial.print(error);
  // Serial.print("  start_count  ");
  // Serial.print(start_count);
  // Serial.print("     Yaw = ");
  // Serial.println(yaw);
}