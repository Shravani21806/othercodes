

void setup() {

    Serial.begin(115200);  // Serial Monitor for debugging
    Serial1.begin(9600);   // UART1 (RX1: Pin 19, TX1: Pin 18) from ESP32
    Serial.println("Arduino Mega Ready. Waiting for data...");
}

void loop() {
    if (Serial1.available()) {
        String receivedData = Serial1.readStringUntil('\n');  // Read data until newline
        Serial.println("Received: " + receivedData);  // Print received data

        // Parsing received data
        int A = receivedData.indexOf("A:");
        int B = receivedData.indexOf("B:");
        int X = receivedData.indexOf("X:");
        int Y = receivedData.indexOf("Y:");
        int LX = receivedData.indexOf("LX:");
        int LY = receivedData.indexOf("LY:");
        int RX = receivedData.indexOf("RX:");
        int RY = receivedData.indexOf("RY:");


    }
}