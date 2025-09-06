void setup() {
    Serial.begin(9600);  // Use default Serial (Pins 0 and 1)
}

void loop() {
    if (Serial.available()) {
        Serial.write(Serial.read());  // Print received data on Serial Monitor
    }
}
