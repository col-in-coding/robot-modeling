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
  PTL("  0. switch off");
}

String inString = "";
void loop() {
if (Serial.available() > 0) {
    if (Serial.peek() != '\n') {
      inString += (char)Serial.read();
    } else {
      Serial.read();
      Serial.print("Instruction received: ");
      
      int code = inString.toInt();
      Serial.println(code);
      switch(code) {
        case 1:
          Robot.switch_on();
          break;
        case 2:
          Robot.bot_stand();
          break;
        case 0:
          Robot.switch_off();
          break;
        default:
          PTL("wrong code");
      }

      inString = "";
    }
  }

}
