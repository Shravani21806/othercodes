int d0 = 2;
int d1 = 3;
int d2 = 4;
int d3 = 5;
int d4 = 6;
int IN1 = A0;
int IN2 = A1;
int IN3 = A2;
int IN4 = A3;
int ENA = 9;
int ENB = 11;

void setup() {
  Serial.begin(9600);
  pinMode(d0, INPUT);
  pinMode(d1, INPUT);
  pinMode(d2, INPUT);
  pinMode(d3, INPUT);
  pinMode(d4, INPUT);
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(9, OUTPUT);
}
void loop() {
  d0 = digitalRead(2);
  d1 = digitalRead(3);
  d2 = digitalRead(4);
  d3 = digitalRead(5);
  d4 = digitalRead(6);
  analogWrite(11, 0);
  analogWrite(9, 0);

  if (d2 == 1 || d3 == 1) {
    forward();
  }
  if (d0==1 || d1==1) {
    right();
  }
  if (d3==1 || d4==1) {
    left();
  }


  Serial.print("d0");
  Serial.print(d0);
  Serial.print("d1");
  Serial.print(d1);
  Serial.print("d2");
  Serial.print(d2);
  Serial.print("d3");
  Serial.print(d3);
  Serial.print("d4");
  Serial.println(d4);
}

void forward() {
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);
}
void left() {
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);
}
void right() {
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);
}
