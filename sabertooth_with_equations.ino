int theta=90;


#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>

SoftwareSerial SWSerial1(NOT_A_PIN, 9); 
SabertoothSimplified ST1(SWSerial1); 
SoftwareSerial SWSerial2(NOT_A_PIN, 6);
SabertoothSimplified ST2(SWSerial2);
void setup()
{
  SWSerial1.begin(9600);
  
  SWSerial2.begin(9600);
}

void loop()
{
  int a=100*sin((theta-45)*3.142/180);
  int b=100*sin((135-theta)*3.142/180);
  int c=100*sin((theta-45)*3.142/180);
  int d=100*sin((135-theta)*3.142/180);

  ST1.motor(1,a);
  ST1.motor(2,b);
  ST2.motor(1,-c);
  ST2.motor(2,d);


}


