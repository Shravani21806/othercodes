// int setpoint = 0, pre_error, p, i, d, error, pid;
// float kp,kd,ki;
int a,b,c;
float s1, s2, s3, s4, s5, s6, s7, s8;
int IN1 = A0;
int IN2 = A1;
int IN3 = A2;
int IN4 = A3;
int ENA = 11;
int ENB = 9;
int counter;
int counter1 = b;
int counter2 = c;
void setup() {
  Serial.begin(9600);
  pinMode(2, INPUT), pinMode(A4, INPUT), pinMode(3, INPUT), pinMode(A5, INPUT), pinMode(4, INPUT), pinMode(A6, INPUT), pinMode(5, INPUT), pinMode(A7, INPUT);
  pinMode(A0, OUTPUT), pinMode(A1, OUTPUT), pinMode(A2, OUTPUT), pinMode(A3, OUTPUT), pinMode(11, OUTPUT), pinMode(9, OUTPUT);
}

void loop() {
  s1 = digitalRead(2), s2 = digitalRead(A4), s3 = digitalRead(3), s4 = digitalRead(A5), s5 = digitalRead(4), s6 = digitalRead(A6), s7 = digitalRead(5), s8 = digitalRead(A7);
  analogWrite(11,60);
  analogWrite(9, 60);
   if (s1 == 1 && s2 == 1 && s3 == 1 && s4 == 1&& s7 == 1  ) {
    counter=counter+1;
  }
  if (counter == 1) {
    forward();
  }
  if (counter == 2) {
    forward();
  }
  if (counter == 3) {
    right();
  }
  if(counter==4){
    right();
  }
  if (s4 == 1 || s5 == 1 || s3 == 1) {
    forward();
  }
  if (s1 == 1 || s2 == 1 || s3 == 1) {
    left();
  }
  if (s6 == 1 || s7 == 1 || s8 == 1) {
    right();
  }

  //    if (s1 == 1 && s2 == 1 && s3 == 1 && s4 == 1 && s5 == 1) {
  //   counter1++;
  // }
  // if (b == 1) {
  //   forward();
  // }
  // if (b == 2) {
  //   forward();
  // }
  // if (b == 3) {
  //   left();
  // }
  // if (b == 4) {
  //   forward();
  // }
  // if (b == 5) {
  //   forward();
  // }
  // if(b==6){
  //   forward();
  // }
  // if(b==7){
  //   left();
  // }
  // if(b==8){
  //   left();
  // }
  // if(b==9){
  //   left();
  // }
  // if(s4==1 && s5==1 && s6==1&& s7==1&& s8==1){
  //   counter2++;
  // }
  // if (c==1){
  //   forward();
  // }
  // if(c==2){
  //   right();
  // }
  
 


  //   if(s1==0&& s2==0 && s3==0 && s4==0 && s5==0 && s6==0 && s7==0 && s8==0){
  //   digitalWrite(IN1,0);
  //   digitalWrite(IN2,0);
  //   digitalWrite(IN3,0);
  //   digitalWrite(IN4,0);
  // }
  
  Serial.print(s1);
  Serial.print(s2);
  Serial.print(s3);
  Serial.print(s4);
  Serial.print(s5);
  Serial.print(s6);
  Serial.print(s7);
  Serial.print(s8);
  Serial.print(counter);
   Serial.print("counter");
    Serial.println(counter);
  }
//
// 
void forward() {
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);
}
void left() {
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);
}
void right() {
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 0);
}
void stop(){
   digitalWrite(IN1,0);
   digitalWrite(IN2,0);
   digitalWrite(IN3,0);
   digitalWrite(IN4,0);

}
