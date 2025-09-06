void setup() {
    // Initialize Serial for debugging
    Serial.begin(115200);

    // Initialize Serial1 for UART communication
    Serial1.begin(9600); // Match the baud rate of ESP32
    Serial.println("Arduino Mega is ready to receive data.");
}

void loop() {
    // Check if data is available on Serial1
    if (Serial1.available() > 0) {
        String receivedData = Serial1.readStringUntil('\n'); // Read data until newline
        Serial.print("Data received: ");
        Serial.println(receivedData);
         // Print received data to Serial Monitor
        }
}
