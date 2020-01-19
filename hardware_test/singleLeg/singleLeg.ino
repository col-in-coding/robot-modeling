#include "Robot3R.h"

Robot3R Robot;

void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("Robot start");
  Robot.start();

}

void loop() {
  // put your main code here, to run repeatedly:

}
