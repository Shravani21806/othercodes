#include <CytronMotorDriver.h>
int F;
int R;
int B;
int L;
int S;
// Motor 1 (Left or A): PWM = 9, DIR = 4
CytronMD motor1(PWM_DIR, 9, 5);
// Motor 2 (Right or B): PWM = 10, DIR = 5
CytronMD motor2(PWM_DIR, 10, 4);

char command;

void setup() {
  Serial.begin(9600);
}

void loop() {
  if (Serial.available()) {
    command = Serial.read();
// if (command==F){
//           motor1.setSpeed(120);
//         motor2.setSpeed(120);

// }
// if(command==B){
//     motor1.setSpeed(-120);
//         motor2.setSpeed(-120);
// }
// if(command==L){
//        motor1.setSpeed(-90);
//         motor2.setSpeed(100);
// }
// if(command==R){
//       motor1.setSpeed(100);
//         motor2.setSpeed(-90);
// }
    switch (command) {
      case 'F':
        motor1.setSpeed(120);
        motor2.setSpeed(120);
        break;

      case 'B':
        motor1.setSpeed(-120);
        motor2.setSpeed(-120);
        break;

      case 'L':
        motor1.setSpeed(-90);
        motor2.setSpeed(100);
        break;

      case 'R':
        motor1.setSpeed(100);
        motor2.setSpeed(-90);
        break;

      case 'S':
        motor1.setSpeed(0);
        motor2.setSpeed(0);
        break;
    }

    Serial.print("Command: ");
    Serial.println(command);
  }
}
