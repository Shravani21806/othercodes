
int setpoint = 0, pre_error, p, i, d, error, pid;
float kp,kd,ki;
float s1,s2,s3,s4,s5,s6,s7,s8;
int IN1=A0;
int IN2=A1;
int IN3=A2;
int IN4=A3;
int ENA=11;
int ENB=9;
 void setup(){
    Serial.begin(9600);
  pinMode(2,INPUT), pinMode(A4,INPUT), pinMode(3,INPUT), pinMode(A5,INPUT), pinMode(4,INPUT), pinMode(A6,INPUT), pinMode(5,INPUT), pinMode(A7,INPUT);
  pinMode(A0,OUTPUT), pinMode(A1,OUTPUT),pinMode(A2,OUTPUT), pinMode(A3,OUTPUT), pinMode(11,OUTPUT), pinMode(9,OUTPUT);
 }

 void loop(){
 s1=digitalRead(2),s2=digitalRead(A4),s3=digitalRead(3),s4=digitalRead(A5),s5=digitalRead(4),s6=digitalRead(A6),s7=digitalRead(5),s8=digitalRead(A7);
  analogWrite(11,50);
  analogWrite(9,50);
  if(s4==0||s5==0){
    forward();
  }
  if(s1==1||s2==1||s3==1){
    left();

  }
  if(s6==1||s7==1||s8==1){
    right();
  }

  
  Serial.print(s1);
  Serial.print(s2);
  Serial.print(s3);
  Serial.print(s4);
  Serial.print(s5);
  Serial.print(s6);
  Serial.print(s7);
  Serial.print(s8);
  Serial.println("");
 }

 void forward(){
  digitalWrite(IN1,0);
  digitalWrite(IN2,1);
  digitalWrite(IN3,0);
  digitalWrite(IN4,1);
 }
 void left(){
  digitalWrite(IN1,1);
  digitalWrite(IN2,0);
  digitalWrite(IN3,0);
  digitalWrite(IN4,1);
 }
 void right(){
  digitalWrite(IN1,0);
  digitalWrite(IN2,1);
  digitalWrite(IN3,0);
  digitalWrite(IN4,0);
 }