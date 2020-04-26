#include "QuadrupedRobot.h"

QuadrupedRobot Robot;

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(10);
  while (!Serial);
  // wait for ready
  while (Serial.available() && Serial.read()); // empty buffer
  delay(1000);

  Serial.print(
    "Please select from menu: \n"
    "  1. switch on\n"
    "  2. stand\n"
    "  3. walk\n"
    "  4. turn left\n"
    "  5. turn right\n"
    "  0. switch off\n"
    "  998. adjust angles\n"
  );
}

void loop() {
  Robot.interrupt();
  Robot.parse_code_run();
}
