
float p;
int q;
int count=0;
volatile int last_state=LOW;
void setup() {
  Serial.begin(9600);
  pinMode(12,INPUT_PULLUP);
  pinMode(13,INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(12), motor_count, CHANGE);
}
void loop() {


   Serial.print(" p ");
   Serial.print(p);
   Serial.print(" q");
   Serial.println(q);
}

void motor_count(){
  int p = digitalRead(12);
  int q = digitalRead(13);
 if (p != last_state){
  if(p!=q)
  {
    count++;
  }
  else
  {
    count--;
  }
 }
 last_state=p;
}
