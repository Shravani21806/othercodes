#include <SPI.h>
// #include <nRF24L01.h>
#include <RF24.h>
#define CE_PIN 23
#define CSN_PIN 22
RF24 radio(CE_PIN, CSN_PIN);
const uint64_t address = 0xF0F0F0F0E1LL;  // Match transmitter
struct XboxData {
  int LX, LY, RX, RY;
  bool A, B, X, Y, LB, RB, L3, R3, START, BACK, XBOX, UP, DOWN, LEFT, RIGHT;
};
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

float R, theta, M, count1 = 0, count2 = 0, count3 = 0, count4 = 0, h = 0, j = 0, k = 0, l = 0, setpoint = 0, pre_error, proportional, integral, derivative, error, pid, L1, L2, R1, R2, LY, LX, a, b, c, d, A, B, C, D;
float kp = 0, ki = 0, kd = 0;
int red_ref = 0;
int L1_ref = 0;
int R1_ref = 0;
float motor_A = 0;
float motor_B = 0;
float motor_C = 0;
float motor_D = 0;

int RY, RX, Ax, Bx, Xx, Yx, up, down, left, right,RB,LB;




void setup() {

  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_LOW);
  // radio.setDataRate(RF24_250KBPS);
  radio.startListening();

  SWSerial1.begin(9600);
  SWSerial2.begin(9600);
  SWSerial3.begin(9600);

  Serial.begin(115200);
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  mpu.calibrateGyro();
  mpu.setThreshold(1);
}

void loop() {
  if (radio.available()) {
    XboxData receivedData;
    radio.read(&receivedData, sizeof(receivedData));

    timer = millis();
    Vector norm = mpu.readNormalizeGyro();
    yaw = yaw + norm.ZAxis * timeStep;
    delay((timeStep * 1000) - (millis() - timer));

    LX = receivedData.LX;
    LY = receivedData.LY;
    RX = receivedData.RX;
    RY = receivedData.RY;
    Ax=receivedData.A;
    Bx=receivedData.B;
    Xx=receivedData.X;
    Yx=receivedData.Y;
    up=receivedData.UP;
    down=receivedData.DOWN;
    left=receivedData.LEFT;
    right=receivedData.RIGHT;
    LB=receivedData.LB;
    RB=receivedData.RB;
     if(Ax == 1){
    ST3.motor(-127);
  }
   if(Ax == 0){
    ST3.motor(0);
  }
  if(Bx==1){
    ST3.motor(127);
  }
   if(Bx==0){
    ST3.motor(0);
  }

    if (error >= 5 && error <= -5) {
      kp = 5, kd = 20, ki = 0.0001;
    } else {
      kp = 30, kd = 90;
      ki = 0.001;
    }
    LX = map(LX, -255, 255, -127, 127);
    LY = map(LY, -255, 255, -127, 127);

    R = sqrt((LX * LX) + (LY * LY));
    theta = atan2(LY, LX);
    theta = theta * 180 / 3.142;

    a = R * sin((theta - 45) * 3.142 / 180);
    b = R * sin((135 - theta) * 3.142 / 180);
   d = R * sin((135 - theta) * 3.142 / 180);
    c = R * sin((theta - 45) * 3.142 / 180);


    error = setpoint - yaw;
    proportional = error;
    integral = integral + error;
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

    Serial.print("   LX  ");
    Serial.print(LX);
    Serial.print("   LY ");
    Serial.print(LY);
    Serial.print("   A ");
    Serial.print(Ax);
    Serial.print("   B ");
    Serial.print(Bx);
    Serial.print("   LB ");
    Serial.print(LB);
    Serial.print("   RB ");
    Serial.print(RB);
    Serial.print("   error  ");
    Serial.print(error);
    Serial.print("     Yaw = ");
    Serial.println(yaw);
}
}