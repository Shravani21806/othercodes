

void setup() {
  Serial.begin(115200);
  pinMode(12, INPUT);
  pinMode(13,INPUT);
  pinMode(16,INPUT);
}

void loop() {
  int x = analogRead(12);  // Read X-Axis value (0-4095)
  int y = analogRead(13);  // Read Y-Axis value (0-4095)
  int xA=map(x,0,4095,-255,255);
  int yA = map(y,0,4095,-255,255);
  
  // Read the button state (LOW when pressed)
  int buttonState = digitalRead(16);
  
  // Print the joystick values to the Serial Monitor
  Serial.print("X-Axis: ");
  Serial.print(xA);
  Serial.print("   Y-Axis: ");
  Serial.print(yA);
  Serial.print("  Button: ");
  Serial.println(buttonState == LOW ? "Pressed" : "Released");
  
  
}

