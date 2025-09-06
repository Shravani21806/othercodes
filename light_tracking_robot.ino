const int m1A=2;
const int m1B=3;
const int m2A=4;
const int m2B=5; 
const int m3A=6;
const int m3B=7;
const int m4A=8;
const int m4B=9;
const int ldr=A0;
const int darkness_threshold=700;
const int light_threshold=300;
void setup(){
  pinMode(M1A,OUTPUT);
  pinMode(M1B,OUTPUT);
  pinMode(M2A,OUTPUT);
  pinMode(M2B,OUTPUT);
  pinMode(M3A,OUTPUT);
  pinMode(M3B,OUTPUT);
  pinMode(M4A,OUTPUT);
  pinMode(M4B,OUTPUT);
  Serial.begin(9600);
}
void loop{
  int ldrvalue=analogRead(ldr);
  if(ldrvalue>=darkness_threshold){
    allmotors(true,true,true,true)
    // forward
  }
  else if(ldrvalue<=light_threshold){
    allmotors(false,false,false,false)
    // stop
  }
  else{
    allmotors(true,false,false,true)
    // robots adjust based on its light
  }
}
void allmotors(bool m1state,bool m2state,bool m3state,bool m4state){
  digitalWrite(m1A,m1state);
  digitalWrite(m1B,m1state);

  digitalWrite(m2A,m2state);
  digitalWrite(m2B,m2state);

  digitalWrite(m3A,m3state);
  digitalWrite(m3B,m3state);

  digitalWrite(m4A,m4state);
  digitalWrite(m4B,m4state);
}

