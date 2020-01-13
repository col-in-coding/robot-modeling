#include "QuadrupedRobot.h"

QuadrupedRobot Robot;

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(10);
  while (!Serial);
  // wait for ready
  while (Serial.available() && Serial.read()); // empty buffer
  delay(100);
  byte te = 10;
  PTLF("\n* Start *");

//   assignSkillAddressToOnboardEeprom();

  // servo
  {
//    pwm.begin();
//    pwm.setPWMFreq(60 * PWM_FACTOR); // Analog servos run at ~60 Hz updates
      Robot.initServos();
//    delay(200);
//
    Robot.awake();

  }
}

void loop() {


}
