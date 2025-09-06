#include <SoftwareSerial.h>
#include <Servo.h>

Servo esc1;
Servo esc2;

int escpin1 = 8;
int escpin2 = 6;

int count1;         // BLDC motor speed 1
int count2;         // BLDC motor speed 2

int Dpad_ref1 = 0;  // Initialize all Dpad refs to 0
int Dpad_ref2 = 0;
int Dpad_ref3 = 0;
int Dpad_ref4 = 0;

SoftwareSerial mySerial(11, 10);  // (RX, TX) - TX is unused

int Dpad = 0;

void setup() {
  Serial.begin(115200);    // Match this with transmitter
  mySerial.begin(115200);  // Match baud rate with transmitter
  Serial.println("Receiver Ready");

  esc1.attach(escpin1);
  esc2.attach(escpin2);

  // Initialize the PWM values for both ESCs
  count1 = 1000;  // Set initial speed to 1000 microseconds
  count2 = 1000;

  esc1.writeMicroseconds(count1);
  esc2.writeMicroseconds(count2);
}

void loop() {
  if (mySerial.available()) {                      // Check if data is coming
    String data = mySerial.readStringUntil('\r');  // Read full line
    data.trim();                                   // Remove unwanted spaces or newlines

    Dpad = data.toInt();  // Convert received string to integer

    // BLDC control logic
    if (Dpad == 4 && Dpad_ref1 == 0) {
      Dpad_ref1 = 1;
    }
    if (Dpad == 0 && Dpad_ref1 == 1) {
      count1 = 1000;   // Reset speeds
      count2 = 1000;
      Dpad_ref1 = 0;
    }

    if (Dpad == 1 && Dpad_ref2 == 0) {
      Dpad_ref2 = 1;
    }
    if (Dpad == 0 && Dpad_ref2 == 1) {
      count1 += 50;    // Increase speed
      count2 += 50;
      Dpad_ref2 = 0;
    }

    if (Dpad == 2 && Dpad_ref3 == 0) {
      Dpad_ref3 = 1;
    }
    if (Dpad == 0 && Dpad_ref3 == 1) {
      count1 -= 50;    // Decrease speed
      count2 -= 50;
      Dpad_ref3 = 0;
    }

    // Dribbling mode
    if (Dpad == 8 && Dpad_ref4 == 0) {
      Dpad_ref4 = 1;
    }
    if (Dpad == 0 && Dpad_ref4 == 1) {
      count1 = 1290;    // Custom speed values
      count2 = 1350;
      Dpad_ref4 = 0;
    }

    // Ensure the speed values remain within valid range
    count1 = constrain(count1, 1000, 2000);
    count2 = constrain(count2, 1000, 2000);

    // Apply updated speeds to ESCs
    esc1.writeMicroseconds(count1);
    esc2.writeMicroseconds(count2);

    // Print values for debugging
    Serial.print("Dpad Value: ");
    Serial.print(Dpad);
    Serial.print(" | count1: ");
    Serial.print(count1);
    Serial.print(" | count2: ");
    Serial.println(count2);
  }
}
