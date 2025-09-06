#include <Bluepad32.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// Define separate variables for each axis value
int axisLX = 0;
int axisLY = 0;
int axisRX = 0;
int axisRY = 0;
int throttle =0;
int brake =0;
int dpad=0;

// This callback gets called any time a new gamepad is connected.
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not find empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}
void processGamepad(ControllerPtr ctl) {
    const int DEAD_ZONE = 42;

    // Get the joystick axis values from the controller
    axisLX = ctl->axisX();
    axisLY = ctl->axisY();
    axisRX = ctl->axisRX();
    axisRY = ctl->axisRY();
    throttle = ctl->throttle();
    brake = ctl->brake();
    dpad =  ctl->dpad();

    // Apply dead zone for left joystick
    if (abs(axisLX) < DEAD_ZONE) axisLX = 0;
    if (abs(axisLY) < DEAD_ZONE) axisLY = 0;

    // Apply dead zone for right joystick
    if (abs(axisRX) < DEAD_ZONE) axisRX = 0;
    if (abs(axisRY) < DEAD_ZONE) axisRY = 0;

    // Format the message to include joystick values and button states
    String message = String("LX:") + axisLX +
                     " LY:" + axisLY +
                     " RX:" + axisRX +
                     " RY:" + axisRY +
                     "Throttle: " + throttle +
                     "Brake: " + brake +
                     " Dpad:" + dpad + 
                     " ButtonA:" + (ctl->a() ? 1 : 0) +
                     " ButtonB:" + (ctl->b() ? 1 : 0) +
                     " ButtonX:" + (ctl->x() ? 1 : 0) +
                     " ButtonY:" + (ctl->y() ? 1 : 0) +
                     " ButtonLB:" + (ctl->l1() ? 1 : 0) +
                     " ButtonRB:" + (ctl->r1() ? 1 : 0) +
                     " ButtonSelect:" + (ctl->miscSelect() ? 1 : 0) +
                     " ButtonStart:" + (ctl->miscStart() ? 1 : 0);

    // Send the formatted message via Serial2
    Serial2.println(message);
}


void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, 16, 17);  // RX=16, TX=17 (ESP32 Serial2)
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    BP32.enableVirtualDevice(true);

    // Wait a bit to stabilize Bluetooth connection
    vTaskDelay(1000);  // Delay for 1 second
}

void loop() {
    // This call fetches all the controllers' data.
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();
    
    vTaskDelay(1);  // To avoid watchdog timeout
}

