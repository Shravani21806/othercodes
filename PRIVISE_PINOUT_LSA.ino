// int setpoint = 0, pre_error, p, i, d, error, pid;
// float kp,kd,ki;
int j_pin;
int a;
int b;
int led=2;
int s1, s2, s3, s4, s5, s6, s7, s8;
int IN1 = A6;
int IN2 = A5;
int IN3 = A4;
int IN4 = A3;
int ENA = 11;
int ENB = 9;
int counter;

void setup() {
  Serial.begin(9600);
  pinMode(13,INPUT),pinMode(2, OUTPUT),pinMode(12, INPUT), pinMode(4, INPUT), pinMode(3, INPUT), pinMode(5, INPUT), pinMode(6, INPUT), pinMode(7, INPUT), pinMode(8, INPUT), pinMode(9, INPUT);
  pinMode(A3, OUTPUT), pinMode(A4, OUTPUT), pinMode(A5, OUTPUT), pinMode(A6, OUTPUT), pinMode(11, OUTPUT), pinMode(9, OUTPUT);
}

void loop() {
 j_pin=digitalRead(13), s1 = digitalRead(12), s2 = digitalRead(3), s3 = digitalRead(4), s4 = digitalRead(5), s5 = digitalRead(6), s6 = digitalRead(7), s7 = digitalRead(8), s8 = digitalRead(9);
 
  if(s4==1&&s5==1){
    forward();
  }
  if(s4==1||s5==1||s3==1){
    forward();
  }
  if(s1==1||s2==1||s3==1){
    left();
  }
  if(s6==1||s7==1){
    right();
  }
  if(s5==1&&s6==1&&s7==1){
    right();
  }
  // if(s1==0&&s2==0&&s3==0&&s4==0&&s5==0&&s6==0&&s7==0&&s8==0){
  //   right();
  //   Serial.print("right");
  // }

 if(j_pin==1&&b==0){
  counter++;
  b=1;
 }
  if(j_pin==0&&b==1){
  b=0;
 }
    if(counter==1&&a==1){
    rightturn();
      Serial.print("right");
      a=2;
    }
    if(counter==2&&a==2){
      forward();
      a=3;
    }
     if(counter==3&&a==3){
      leftturn();
      a=1;
    }
 if(s2==1&&s3==1&&s4==1&&s5==1&&s6==1&&s7==1){
  digitalWrite(2,HIGH);
 }
 else{
  digitalWrite(2,LOW);
 }
    // if(counter==3){
    //   left();
    // }
 
 

  Serial.print(    s1);
  Serial.print(  s2);
  Serial.print(  s3);
  Serial.print(  s4);  //off when all 6 on
  Serial.print(  s5);
  Serial.print(  s6);
  Serial.print(  s7);
  Serial.print(  s8);
   
  Serial.print("  j_pin");
  Serial.print(j_pin);
   
  Serial.print("  counter");
  Serial.println(counter);

  
}
void forward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  analogWrite(11, 60);
  analogWrite(9, 60);
}
void left() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
    analogWrite(11, 70);
  analogWrite(9, 70);
}

void right() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);
  analogWrite(11, 60);
  analogWrite(9, 70);
}
void rightturn() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);
  analogWrite(11, 80);
  analogWrite(9, 80);
}
void leftturn() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(11, 80);
  analogWrite(9, 80);
}
void stop() {
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 0);
}
