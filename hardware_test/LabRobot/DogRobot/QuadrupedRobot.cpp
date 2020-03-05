#include "instinct.h"
#include "QuadrupedRobot.h"
#include "Arduino.h"


QuadrupedRobot::QuadrupedRobot() {

}

QuadrupedRobot::~QuadrupedRobot() {
  
}

void QuadrupedRobot::switch_on() {
  PTL("Robot Start");
  setup_servos();
}

void QuadrupedRobot::switch_off() {
  
}

void QuadrupedRobot::setup_servos() {
  PTL("Setup Servos");
  for (size_t i = 0; i < DOF; i++) {
    if(servos[i] != nullptr) servos[i] = new Servo();
    servos[i]->attach(pins[i], SERVOMIN, SERVOMAX);
  }
}

void QuadrupedRobot::shut_servos() {
  for (size_t i = 0; i < DOF; i++) {
    if (servos[i] != nullptr) {
      servos[i]->detach();
    }
  }
}
