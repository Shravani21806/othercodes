 


#include <Wire.h>
#include <MPU6050.h>
MPU6050 mpu;
unsigned long timer = 0;
float timeStep = 0.01;
float yaw = 0;
int R;
int theta;

// #include <SoftwareSerial.h>
// #include <SabertoothSimplified.h>

// SoftwareSerial SWSerial1(NOT_A_PIN, 9); 
// SabertoothSimplified ST1(SWSerial1); 
// SoftwareSerial SWSerial2(NOT_A_PIN, 3);
// SabertoothSimplified ST2(SWSerial2);

float setpoint=0;
float pre_error;
float kp=5  ;
float ki=0;
float kd=1;
float p;
float i;
float d;
float error;
float pid;
void setup()
{
  // SWSerial1.begin(9600);
  
  // SWSerial2.begin(9600);
 
  Serial.begin(115200);
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
   mpu.calibrateGyro();
    mpu.setThreshold(1);
 
} 

void loop()
{   
 timer = millis();
  Vector norm = mpu.readNormalizeGyro();
   yaw = yaw + norm.ZAxis * timeStep;
  Serial.print(" Yaw = ");
  Serial.println(yaw);
    delay((timeStep*1000) - (millis() - timer));

//  error=setpoint-yaw;
//  p=error;
//  i=i+error;
//  d=error-pre_error;
// pre_error=error;
// pid=kp*p+ki*i+kd*d;
// //  pid=constrain(pid,-127,127);

//  Serial.print("pid ");
//  Serial.print(pid);


//   int a=R*sin((theta-45)*3.142/180);
//   int b=R*sin((135-theta)*3.142/180);
//   int c=R*sin((theta-45)*3.142/180);
//   int d=R*sin((135-theta)*3.142/180);

//   int A=a+pid;
//   int B=b-pid;
//   int C=c+pid;
//   int D=d+pid;

  // ST1.motor(1,A);
  // ST1.motor(2,B);
  // ST2.motor(1,C);
  // ST2.motor(2,D);


}


