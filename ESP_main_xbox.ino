#include <Bluepad32.h>
#include <HardwareSerial.h>

HardwareSerial mySerial(1);  // UART1 (TX: GPIO17, RX: GPIO16)

GamepadPtr myGamepad;

// Dead zone threshold to ignore small joystick drift
const int DEAD_ZONE = 15;

void onGamepadConnected(GamepadPtr gp) {
    Serial.println("🎮 Xbox Controller Connected!");
    myGamepad = gp;
}

void onGamepadDisconnected(GamepadPtr gp) {
    Serial.println("❌ Xbox Controller Disconnected!");
    myGamepad = nullptr;
}

void setup() {
    Serial.begin(115200);
    mySerial.begin(9600, SERIAL_8N1, 16, 17);

    BP32.setup(onGamepadConnected, onGamepadDisconnected);
    Serial.println("🔍 Waiting for Xbox controller...");

    BP32.forgetBluetoothKeys();
    BP32.enableNewBluetoothConnections(true);
}

// Function to apply dead zone
int applyDeadZone(int value, int deadZone) {
    if (abs(value) < deadZone) return 0;
    return value;
}

void loop() {
    BP32.update();

    if (myGamepad && myGamepad->isConnected()) {
        // Read button states
        int A = myGamepad->a();
        int B = myGamepad->b();
        int X = myGamepad->x();
        int Y = myGamepad->y();
        int Dpad = myGamepad->dpad();
        int Brake = myGamepad->brake();
        int throttle = myGamepad->throttle();

        // LB, RB, LT, RT
        int LB = myGamepad->l1();
        int RB = myGamepad->r1();
        int LT = myGamepad->l2();  // Analog trigger value (0-255)
        int RT = myGamepad->r2();  // Analog trigger value (0-255)

        // Read joystick values and map them (-512 to 512) → (-127 to 127)
        int leftStickX = map(myGamepad->axisX(), -512, 512, -127, 127);
        int leftStickY = map(myGamepad->axisY(), -512, 512, 127, -127);
        int rightStickX = map(myGamepad->axisRX(), -512, 512, -127, 127);
        int rightStickY = map(myGamepad->axisRY(), -512, 512, 127, -127);

        // Apply dead zone filter
        leftStickX = applyDeadZone(leftStickX, DEAD_ZONE);
        leftStickY = applyDeadZone(leftStickY, DEAD_ZONE);
        rightStickX = applyDeadZone(rightStickX, DEAD_ZONE);
        rightStickY = applyDeadZone(rightStickY, DEAD_ZONE);

        // Create a formatted string to send
        String data = "A:" + String(A) + " B:" + String(B) +
                      " X:" + String(X) + " Y:" + String(Y) +
                      " LX:" + String(leftStickX) +
                      " LY:" + String(leftStickY) +
                      " RX:" + String(rightStickX) +
                      " RY:" + String(rightStickY) +
                      " Dpad:" + String(Dpad) +
                      " LT:" + String(Brake) +
                      " RT:" + String(throttle) +
                      " LB:" + String(LB) + 
                      " RB:" + String(RB);

        mySerial.println(data);
        Serial.println(data);

        delay(15);
    }
}