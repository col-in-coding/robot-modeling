#include "Robot3R.h"

Robot3R robot = Robot3R(5, 6, 7);

void setup() {
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  Serial.begin(9600);
  robot.start();
}

void loop() {
  robot.run_period();
}
