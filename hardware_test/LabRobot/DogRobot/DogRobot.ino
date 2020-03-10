#include "QuadrupedRobot.h"

QuadrupedRobot Robot;

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(10);
  while (!Serial);
  // wait for ready
  while (Serial.available() && Serial.read()); // empty buffer
  delay(1000);

  PTL("Please select from menu: ");
  PTL("  1. switch on");
  PTL("  2. stand");
  PTL("  3. walk");
  PTL("  4. trot");
  PTL("  0. switch off");
}

int inp = "";
int code = 999;
void loop() {
  if (Serial.available() > 0) {
    if (Serial.peek() != '\n') {
      inp = Serial.parseInt();
    } else {
      Serial.read();
      Serial.print("Instruction received: ");
      Serial.println(inp);

      code = inp;
      Robot.lock = false; // unlock the robot
      inp = "";
    }
  }

  switch(code) {
    case 1:
      Robot.switch_on();
      // lock the robot, unless the periodic motion code
      Robot.lock = true;
      break;
    case 2:
      Robot.bot_stand();
      Robot.lock = true;
      break;
    case 3:
      Robot.bot_walk();
//      Robot.lock = true; // for test
      break;
    case 4:
      Robot.bot_trot();
      break;
    case 0:
      Robot.switch_off();
      Robot.lock = true;
      break;
    case 999:
      // waiting for input ...
      break;
    default:
      PTL("wrong code");
      code = 999;
  }

}
