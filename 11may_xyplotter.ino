// === Pin Assignments ===
const byte encoderXPin = 2;  // X-axis encoder - must be interrupt pin
const byte encoderYPin = 3;  // Y-axis encoder - must be interrupt pin

// === Encoder Configuration ===
volatile long pulseCountX = 0;
volatile long pulseCountY = 0;

const float wheelDiameter = 60.0; // in mm (adjust to your wheel)
const int pulsesPerRevolution = 600; // adjust based on your encoder spec
const float wheelCircumference = 3.1416 * wheelDiameter;

// === Position Tracking ===
float positionX = 0.0; // in mm
float positionY = 0.0; // in mm

unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 100; // update every 100ms

void setup() {
  Serial.begin(9600);

  pinMode(encoderXPin, INPUT_PULLUP);
  pinMode(encoderYPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderXPin), countX, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderYPin), countY, RISING);

  lastUpdateTime = millis();
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastUpdateTime >= updateInterval) {
    updatePosition();
    lastUpdateTime = currentTime;

    // Print X, Y coordinates
    Serial.print("X Position (mm): ");
    Serial.print(positionX);
    Serial.print(" | Y Position (mm): ");
    Serial.println(positionY);
  }
}

void countX() {
  pulseCountX++;
}

void countY() {
  pulseCountY++;
}

void updatePosition() {
  noInterrupts();
  long currentX = pulseCountX;
  long currentY = pulseCountY;
  pulseCountX = 0;
  pulseCountY = 0;
  interrupts();

  float dx = (currentX / (float)pulsesPerRevolution) * wheelCircumference;
  float dy = (currentY / (float)pulsesPerRevolution) * wheelCircumference;

  positionX += dx;
  positionY += dy;
}
