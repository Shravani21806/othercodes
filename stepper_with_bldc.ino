
#include <Servo.h>
Servo esc; // Create a Servo object to control the ESC
int escPin = 9; // Connect the ESC signal wire to pin 9
int throttle = 1000; // Throttle value in microseconds (1000-2000)

#define PULSE_PIN 8  // Digital pin connected to PUL+ (Pulse).
#define DIR_PIN 7    // Digital pin connected to DIR+ (Direction).
#define ENA_PIN 6    // Digital pin connected to ENA+ (Enable) [Optional].
const int stepsPerRevolution = 5000;

void setup() {

  esc.attach(escPin); // Attach the ESC to the signal pin
  esc.writeMicroseconds(1000);

  pinMode(8, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);

  digitalWrite(6, LOW); // LOW to enable, HIGH to disable.
  digitalWrite(7, HIGH); // HIGH for one direction, LOW for the opposite.
}

void loop() {

  esc.writeMicroseconds(1000);
  
for (int i = 0; i < stepsPerRevolution; i++) {
    digitalWrite(8, HIGH); // Create rising edge.
    delayMicroseconds(1000); // Wait for pulse duration.
    digitalWrite(8, LOW);  // Create falling edge.
    delayMicroseconds(1000); // Wait before next pulse.
  }

  while (true) {
    // Do nothing; the motor remains stopped.
  }
  
}
