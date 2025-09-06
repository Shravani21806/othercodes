const int analogPin = A0;  // A0 pin for analog output
const int digitalPin = A1; // A1 pin used as digital input (D0 from sensor)
// const int ledPin = 1;     // Onboard LED
int enA = 5;
int in1 = 7;
int in2 = A5;

int enB = 6;
int in3 = 4;
int in4 = 3;

int enC = 9;
int in5 = 11;
int in6 = 12;

int enD = 10;
int in7 = 13;
int in8 = 8;
void setup() {
  Serial.begin(9600);
  pinMode(digitalPin, INPUT); // A1 as digital input
  pinMode(2, OUTPUT);
    pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(enC, OUTPUT);
  pinMode(enD, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(in5, OUTPUT);
  pinMode(in6, OUTPUT);
  pinMode(in7, OUTPUT);
  pinMode(in8, OUTPUT);
}

void loop() {
  int analogValue = analogRead(analogPin);       // Read analog value from A0
  int digitalState = digitalRead(digitalPin);    // Read digital state from A1

  // Print analog value
  Serial.print("Analog Moisture Value: ");
  Serial.println(analogValue);

  // Interpret analog value (adjust thresholds if needed)
  // if (analogValue > 800) {
  //   Serial.println("Analog Status: Soil is Dry");
  // } else if (analogValue > 400) {
  //   Serial.println("Analog Status: Soil is Moist");
  // } else {
  //   Serial.println("Analog Status: Soil is Wet");
  // }

  // Interpret digital pin state
  if(analogValue<270){
    Serial.println("Digital Status: Soil is Wet (Below Threshold)");
    digitalWrite(2, HIGH);
      digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  analogWrite(enA, 50);
  analogWrite(enB, 55);


  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);

  analogWrite(enC, 40);
  analogWrite(enD, 40);
  }
 

  else {
    Serial.println("Digital Status: Soil is Dry (Above Threshold)");
    digitalWrite(2, LOW); 
     analogWrite(enA, 0);
  analogWrite(enB, 0);
    analogWrite(enC, 0);
  analogWrite(enD, 0);// Turn ON LED
  }

  Serial.println("-----------------------------");
  delay(1000); // Delay before next reading
}
