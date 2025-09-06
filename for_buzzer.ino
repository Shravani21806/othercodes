int a=5;
void setup(){
  pinMode(a,OUTPUT);

}
void loop(){
  digitalWrite(a,HIGH);
  delay(2000);
  digitalWrite(a,LOW);
  delay(2000);
}