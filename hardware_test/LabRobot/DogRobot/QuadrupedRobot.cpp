#include "QuadrupedRobot.h"
#include "Arduino.h"


void QuadrupedRobot::setup_servos() {
  PTL("Setup Servos");
  for (size_t i = 0; i < DOF; i++) {
    if(servos[i] == nullptr) servos[i] = new Servo();
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

void QuadrupedRobot::servo_write_angs(double angs[DOF]) {
  PTL("Servo Write Angles");
  double real_ang {};
  double pulsewidth {};
  for (size_t i = 0; i < DOF; i++) {
    real_ang = init_angs[i] + dir[i] * angs[i];
    pulsewidth = 500 + real_ang / 90.0 * 1000;
    if (!servos[i]->attached()) {
      servos[i]->attach(pins[i], SERVOMIN, SERVOMAX);
    }
    servos[i]->writeMicroseconds(pulsewidth);
  }
  delayMicroseconds(3000);
}

QuadrupedRobot::QuadrupedRobot() {

}

QuadrupedRobot::~QuadrupedRobot() {
  
}

void QuadrupedRobot::switch_on() {
  PTL("Robot Switch On");
  setup_servos();
  bot_rest();
  delay(1000);
  shut_servos();
}

void QuadrupedRobot::switch_off() {
  PTL("Robot Switch Off");
  shut_servos();
}

void QuadrupedRobot::bot_rest() {
  PTL("Robot Rest");
  double rest_angs[12] {0, -55, 130, 0, -55, 130, 0, 55, -130, 0, 55, -130};
  servo_write_angs(rest_angs);
}

void QuadrupedRobot::bot_stand() {
  PTL("Robot Stand");
  double stand_angs[12] {0, -30, 60, 0, -30, 60, 0, 30, -60, 0, 30, -60};
  servo_write_angs(stand_angs);
}
