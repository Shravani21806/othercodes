#include <Wire.h>
#include <MPU6050.h>
#include <SoftwareSerial.h>

// Variables for position and velocity
float positionX = 0, positionY = 0, positionZ = 0;
float velocityX = 0, velocityY = 0, velocityZ = 0;

// Low-pass filter coefficient
const float alpha = 0.1;

// Previous acceleration values for filtering
float filteredAccelX = 0, filteredAccelY = 0, filteredAccelZ = 0;

// Time tracking
unsigned long prevTime;

MPU6050 mpu;

// LiDAR Variables
SoftwareSerial lidarSerial(10, 11); // RX, TX pins for LiDAR
const int LIDAR_BAUDRATE = 115200;
float lidarDistance = 0; // Distance measured by LiDAR (in meters)

void setup() 
{
  Serial.begin(115200);
  lidarSerial.begin(LIDAR_BAUDRATE);

  Serial.println("Initialize MPU6050");

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  checkSettings();

  // Initialize time tracking
  prevTime = millis();
}

void checkSettings()
{
  Serial.println();
  
  Serial.print(" * Sleep Mode:            ");
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");
  
  Serial.print(" * Clock Source:          ");
  switch(mpu.getClockSource())
  {
    case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
  }
  
  Serial.print(" * Accelerometer:         ");
  switch(mpu.getRange())
  {
    case MPU6050_RANGE_16G:            Serial.println("+/- 16 g"); break;
    case MPU6050_RANGE_8G:             Serial.println("+/- 8 g"); break;
    case MPU6050_RANGE_4G:             Serial.println("+/- 4 g"); break;
    case MPU6050_RANGE_2G:             Serial.println("+/- 2 g"); break;
  }  

  Serial.print(" * Accelerometer offsets: ");
  Serial.print(mpu.getAccelOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getAccelOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getAccelOffsetZ());
  
  Serial.println();
}

void loop()
{
  // Read normalized acceleration values
  Vector normAccel = mpu.readNormalizeAccel();

  // Apply a low-pass filter to smooth the data
  filteredAccelX = alpha * normAccel.XAxis + (1 - alpha) * filteredAccelX;
  filteredAccelY = alpha * normAccel.YAxis + (1 - alpha) * filteredAccelY;
  filteredAccelZ = alpha * normAccel.ZAxis + (1 - alpha) * filteredAccelZ;

  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;  // Convert to seconds
  prevTime = currentTime;

  // If acceleration is near zero, reset velocity and position
  if (abs(filteredAccelX) < 0.05 && abs(filteredAccelY) < 0.05 && abs(filteredAccelZ) < 0.05)
  {
    velocityX = 0;
    velocityY = 0;
    velocityZ = 0;
    return;
  }

  // Integrate acceleration to calculate velocity
  velocityX += filteredAccelX * dt;
  velocityY += filteredAccelY * dt;
  velocityZ += filteredAccelZ * dt;

  // Integrate velocity to calculate position
  positionX += velocityX * dt;
  positionY += velocityY * dt;
  positionZ += velocityZ * dt;

  // Read LiDAR Distance
  lidarDistance = readLidarDistance();

  // Print results
  Serial.print("Position: X=");
  Serial.print(positionX, 2);
  Serial.print(" Y=");
  Serial.print(positionY, 2);
  Serial.print(" Z=");
  Serial.print(positionZ, 2);
  Serial.print(" LiDAR Distance: ");
  Serial.print(lidarDistance, 2);
  Serial.println(" m");

  delay(10);
}

float readLidarDistance()
{
  if (lidarSerial.available() >= 9) // LiDAR TF02 sends 9-byte frames
  {
    uint8_t buffer[9];
    for (int i = 0; i < 9; i++)
    {
      buffer[i] = lidarSerial.read();
    }

    // Check the frame header
    if (buffer[0] == 0x59 && buffer[1] == 0x59) // Header bytes for TF02
    {
      int distance = (buffer[2] | (buffer[3] << 8)); // Combine low and high bytes
      return distance / 100.0; // Convert to meters
    }
  }

  return -1; // Return -1 if no valid data is available
}
