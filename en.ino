
int count=0;
volatile int last_state=LOW;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(2,INPUT_PULLUP);
  pinMode(3,INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(2), motor_count, CHANGE);
  

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(count);

}
void motor_count(){
  int p = digitalRead(2);
  int q = digitalRead(3);
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


 