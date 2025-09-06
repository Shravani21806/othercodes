// int setpoint = 0, pre_error, p, i, d, error, pid;
// float kp,kd,ki;
int j_pin;
int a, b, c;
int s1, s2, s3, s4, s5, s6, s7, s8;
int IN1 = 7;
int IN2 = 6;
int IN3 = 8;
int IN4 = 12;
int ENA = 11;
int ENB = 9;
int counter;
int counter1 = b;
int counter2 = c;
void setup() {
  Serial.begin(9600);
  pinMode(13,INPUT),pinMode(A2,INPUT),pinMode(2, INPUT), pinMode(4, INPUT), pinMode(3, INPUT), pinMode(5, INPUT), pinMode(A4, INPUT), pinMode(A5, INPUT), pinMode(A6, INPUT), pinMode(A7, INPUT);
  pinMode(7, OUTPUT), pinMode(6, OUTPUT), pinMode(8, OUTPUT), pinMode(12, OUTPUT), pinMode(11, OUTPUT), pinMode(9, OUTPUT);
}

void loop() {
 j_pin=digitalRead(13),a=digitalRead(A2), s1 = digitalRead(2), s2 = digitalRead(3), s3 = digitalRead(4), s4 = digitalRead(5), s5 = digitalRead(A4), s6 = digitalRead(A5), s7 = digitalRead(A6), s8 = digitalRead(A7);
 
  
   
if (s4==1) {
    forward();
  }
  if (s3==1||s4==1) {
    forward();
  }
 if (s1==1&& s2 == 1&&s3 == 1) {
    left();
    Serial.print("left");
  } 
  if (s1==1 || s2 == 1 || s3 == 1) {
    left();
    Serial.print("left");
  } 
  else if (s1==1&& s2 == 1&&s3 == 1&&s4==1) {
    left();
  } 
  else if(s2==1){
    left();
  }
else if (s6 == 1||s5==1) {
    right();
    Serial.print("right");
  }
  if(s1==0&&s2==0&&s3==0&&s4==0&&s5==0&&s6==0&&s7==0&&s8==0){
    left();
  }
  if(s5==1&&s6==0&&s7==0){
    right();
  }

  
  if(j_pin==1){
    // counter++;
    right();}
    // if(s1==0&&s2==0&&s3==0&&s4==0&&s5==0&&s6==0){
    //   left();
    // }
  // }
  // if(counter==1){
  //   right();
    
  // }
  // if(counter==2){
  //   right();
  // //  delay(1000);
  // }
  // if(counter==3){
  //   right();
  // // delay(1000);
  // }
  // if(counter==4){
  //   forward();
  // }
  //  if(counter==5){
  // //   forward();
  // // }
  // else if(counter==5){
  //   counter=0;
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
   Serial.print(  a);
  Serial.print("  counter");
  Serial.println(counter);

  
}
void forward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  analogWrite(11, 50);
  analogWrite(9, 50);
}
void left() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
    analogWrite(11, 60);
  analogWrite(9, 60);
}

void right() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(11, 50);
  analogWrite(9, 50);
}
void stop() {
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 0);
}
