
#include <Servo.h>

Servo esc1; 
Servo esc2;// Create a Servo object to control the ESC
int escPin1 = 10;
int escPin2= 11;
 // Connect the ESC signal wire to pin 9
int throttle = 1000;
#define PULSE_PIN 8  // Digital pin connected to PUL+ (Pulse).
#define DIR_PIN 7   // Digital pin connected to DIR+ (Direction).
#define ENA_PIN 6    // Digital pin connected to ENA+ (Enable) [Optional].

const int stepsPerRevolution = 12000;

void setup() {
  esc1.attach(escPin1); 
  esc2.attach(escPin2); // Attach the ESC to the signal pin
  esc1.writeMicroseconds(1000); 
 esc2.writeMicroseconds(1000);
  pinMode(8, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);

  digitalWrite(6, LOW); // LOW to enable, HIGH to disable.
  digitalWrite(7, HIGH); // HIGH for one direction, LOW for the opposite.
}

void loop() {
  esc1.writeMicroseconds(1000);
esc2.writeMicroseconds(1000);
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
