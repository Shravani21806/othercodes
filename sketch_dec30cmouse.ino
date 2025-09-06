// Define sensor pins
const int DATA_PIN = 9;  // Data line (DP)
const int CLK_PIN = 10;   // Clock line (DN)

// Position variables
float positionX = 0.0;
float positionY = 0.0;

// Sensor-specific settings
const float sensorDPI = 1200.0;  // Sensor resolution (DPI)
const float inchesToMeters = 0.0254;  // Conversion factor from inches to meters

// Threshold to filter out random noise (e.g., small movements)
const int noiseThreshold = 5;

void setup() {
  pinMode(DATA_PIN, INPUT);
  pinMode(CLK_PIN, OUTPUT);
  digitalWrite(CLK_PIN, LOW);  // Start with the clock line low

  Serial.begin(115200);  // Initialize serial communication
}

void loop() {
  int deltaX = readMovement(); // Retrieve ΔX movement data
  int deltaY = readMovement(); // Retrieve ΔY movement data

  // Filter out noise if the movement is below the noise threshold
  if (abs(deltaX) < noiseThreshold) {
    deltaX = 0;
  }
  if (abs(deltaY) < noiseThreshold) {
    deltaY = 0;
  }

  // Convert to meters
  float deltaXMeters = deltaX / (sensorDPI * inchesToMeters);
  float deltaYMeters = deltaY / (sensorDPI * inchesToMeters);

  // Update position
  positionX += deltaXMeters;
  positionY += deltaYMeters;

  // Print position
  Serial.print("Position X: ");
  Serial.print(positionX, 4);  // Print with 4 decimal places
  Serial.print(" m, Y: ");
  Serial.print(positionY, 4);  // Print with 4 decimal places
  Serial.println(" m");

  delay(100);  // Adjust delay as needed
}

// Function to read movement data from the sensor
int readMovement() {
  int value = 0;

  for (int i = 0; i < 8; i++) {  // Assuming 8-bit data
    digitalWrite(CLK_PIN, HIGH);  // Clock high
    delayMicroseconds(10);        // Short delay
    value <<= 1;                  // Shift bits
    if (digitalRead(DATA_PIN)) {
      value |= 1;                 // Read data bit
    }
    digitalWrite(CLK_PIN, LOW);   // Clock low
    delayMicroseconds(10);        // Short delay
  }

  // Convert signed 8-bit value
  if (value & 0x80) {
    value |= 0xFFFFFF00;  // Sign-extend if MSB is set
  }

  return value;
}
