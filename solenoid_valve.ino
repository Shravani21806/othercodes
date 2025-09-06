# define s_pin 7

void setup() {
  // put your setup code here, to run once:
    pinMode(s_pin,OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(s_pin,HIGH);
  delay(5000);
  digitalWrite(s_pin,LOW);
  delay(1000);

}
