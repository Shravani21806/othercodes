#include <Encoder.h>

// Define pins for the encoders
#define LEFT_ENCODER_A 2
#define LEFT_ENCODER_B 3
#define RIGHT_ENCODER_A 4
#define RIGHT_ENCODER_B 5

// Encoder objects for left and right wheels
Encoder leftEncoder(LEFT_ENCODER_A, LEFT_ENCODER_B);
Encoder rightEncoder(RIGHT_ENCODER_A, RIGHT_ENCODER_B);

// Robot parameters
const int TPR = 360;        // Ticks per revolution
const double R = 0.05;      // Wheel radius in meters
const double L = 0.2;       // Distance between the two wheels in meters

// Initial position and orientation
double x = 0.0;             // X-coordinate in meters
double y = 0.0;             // Y-coordinate in meters
double theta = 0.0;         // Orientation in radians

void setup() {
    Serial.begin(9600);
    // Reset encoders
    leftEncoder.write(0);
    rightEncoder.write(0);
}

void loop() {
    // Read encoder ticks
    long ticks_left = leftEncoder.read();
    long ticks_right = rightEncoder.read();

    // Reset encoders for next loop iteration
    leftEncoder.write(0);
    rightEncoder.write(0);

    // Convert ticks to distances
    double d_left = (ticks_left / (double)TPR) * 2 * PI * R;
    double d_right = (ticks_right / (double)TPR) * 2 * PI * R;

    // Compute linear and angular displacement
    double d = (d_left + d_right) / 2;
    double delta_theta = (d_right - d_left) / L;

    // Update position and orientation
    double delta_x = d * cos(theta + delta_theta / 2);
    double delta_y = d * sin(theta + delta_theta / 2);
    x += delta_x;
    y += delta_y;
    theta += delta_theta;

    // Normalize theta to keep it within 0 to 2*PI
    if (theta > 2 * PI) theta -= 2 * PI;
    if (theta < 0) theta += 2 * PI;

    // Print the updated position and orientation
    Serial.print("x: ");
    Serial.print(x, 3); // Print with 3 decimal places
    Serial.print(" m, y: ");
    Serial.print(y, 3);
    Serial.print(" m, theta: ");
    Serial.print(theta, 3);
    Serial.println(" rad");

    delay(100); // Delay for 100 ms before next update
}
