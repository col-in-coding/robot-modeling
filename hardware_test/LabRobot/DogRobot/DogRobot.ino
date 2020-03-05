#include "QuadrupedRobot.h"

QuadrupedRobot Robot;

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(10);
  while (!Serial);
  // wait for ready
  while (Serial.available() && Serial.read()); // empty buffer
  delay(100);

  // servo
  Robot.switch_on();

}

void loop() {


}
