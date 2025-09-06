#include <SPI.h>
// #include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN 23
#define CSN_PIN 22

RF24 radio(CE_PIN, CSN_PIN);
const uint64_t address = 0xF0F0F0F0E1LL;  // Match transmitter


struct XboxData {
  int LX, LY, RX, RY;
  bool A, B, X, Y, LB, RB,L3, R3, START, BACK, XBOX, UP, DOWN, LEFT, RIGHT;
};

void setup() {
  Serial.begin(115200);

  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_LOW);
  // radio.setDataRate(RF24_250KBPS);
  radio.startListening();

  Serial.println("Receiver Ready");
}

void loop() {

  if (radio.available()) {
    XboxData receivedData;
    radio.read(&receivedData, sizeof(receivedData));


    Serial.print("Received - LX: ");
    Serial.print(receivedData.LX);
    Serial.print(" LY: ");
    Serial.print(receivedData.LY);
    Serial.print(" RX: ");
    Serial.print(receivedData.RX);
    Serial.print(" RY: ");
    Serial.print(receivedData.RY);
    Serial.print(" A: ");
    Serial.print(receivedData.A);
    Serial.print(" B: ");
    Serial.print(receivedData.B);
    Serial.print(" X: ");
    Serial.print(receivedData.X);
    Serial.print(" Y: ");
    Serial.print(receivedData.Y);
    Serial.print(" START: ");
    Serial.print(receivedData.START);
    Serial.print(" BACK: ");
    Serial.print(receivedData.BACK);
    Serial.print(" XBOX: ");
    Serial.print(receivedData.XBOX);
    Serial.print(" UP: ");
    Serial.print(receivedData.UP);
    Serial.print(" DOWN: ");
    Serial.print(receivedData.DOWN);
    Serial.print(" LEFT: ");
    Serial.print(receivedData.LEFT);
    Serial.print(" RIGHT: ");
    Serial.print(receivedData.RIGHT);
    Serial.print(" LB: ");
    Serial.print(receivedData.LB);
    Serial.print(" RB: ");
    Serial.print(receivedData.RB);
    Serial.print(" L3: ");
    Serial.print(receivedData.L3);
    Serial.print(" R3: ");
    Serial.println(receivedData.R3);
  }
}