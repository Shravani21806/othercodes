void setup() {
    Serial.begin(9600);  // Monitor output
    Serial1.begin(9600); // UART1 for communication with ESP32
}

void loop() {
    if (Serial1.available()) {
        String data = Serial1.readStringUntil('\n'); // Read data from ESP32
        Serial.println("Received: " + data); // Print raw data

        // Parse data into an array
        int values[15]; // Array for joystick, trigger, and button values
        int index = 0;
        char *ptr = strtok((char *)data.c_str(), ",");
        while (ptr != nullptr && index < 15) {
            values[index++] = atoi(ptr);
            ptr = strtok(nullptr, ",");
        }

        // Print parsed values
        if (index == 15) {
            Serial.println("Joystick:");
            Serial.print("  LX: "); Serial.println(values[0]);
            Serial.print("  LY: "); Serial.println(values[1]);
            Serial.print("  RX: "); Serial.println(values[2]);
            Serial.print("  RY: "); Serial.println(values[3]);

            Serial.println("Triggers:");
            Serial.print("  L2: "); Serial.println(values[4]);
            Serial.print("  R2: "); Serial.println(values[5]);

            Serial.println("Buttons:");
            Serial.print("  A: "); Serial.println(values[6]);
            Serial.print("  B: "); Serial.println(values[7]);
            Serial.print("  X: "); Serial.println(values[8]);
            Serial.print("  Y: "); Serial.println(values[9]);
            Serial.print("  L1: "); Serial.println(values[10]);
            Serial.print("  R1: "); Serial.println(values[11]);
            Serial.print("  Select: "); Serial.println(values[12]);
            Serial.print("  Start: "); Serial.println(values[13]);
            Serial.print("  System: "); Serial.println(values[14]);
        } else {
            Serial.println("Error: Incomplete data received.");
}
}
}
