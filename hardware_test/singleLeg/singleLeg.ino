#include "Robot3R.h"

Robot3R robot = Robot3R(2, 3, 4);

void setup() {
  Serial.begin(9600);
  robot.start();
}

void loop() {
  robot.run_period();
}
