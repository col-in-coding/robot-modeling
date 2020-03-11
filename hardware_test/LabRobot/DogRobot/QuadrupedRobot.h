#ifndef QUADRUPED_ROBOT_H
#define QUADRUPED_ROBOT_H

#include "Arduino.h"
#include <Servo.h>

#define PT(s) Serial.print(s)
#define PTL(s) Serial.println(s)
#define PTF(s) Serial.print(F(s)) //trade flash memory for dynamic memory with F() function
#define PTLF(s) Serial.println(F(s))

// Servo Pin configs
#define rFL_PIN 3
#define sFL_PIN 4
#define kFL_PIN 5
#define rFR_PIN 6
#define sFR_PIN 7
#define kFR_PIN 8
#define rHR_PIN 9
#define sHR_PIN 10
#define kHR_PIN 11
#define rHL_PIN 12
#define sHL_PIN 13
#define kHL_PIN 14

//servo constants
#define SERVOMIN 500
#define SERVOMAX 2500

#define DOF 12

// CPG Oscillator data output time period: CPG_DT * CPG_ITERATES
#define CPG_DT 0.005
#define CPG_ITERATES 10

class QuadrupedRobot
{
private:
  Servo *servos[DOF] = {
      nullptr, nullptr, nullptr,
      nullptr, nullptr, nullptr,
      nullptr, nullptr, nullptr,
      nullptr, nullptr, nullptr};
  byte pins[DOF]{
      rFL_PIN, sFL_PIN, kFL_PIN,
      rFR_PIN, sFR_PIN, kFR_PIN,
      rHR_PIN, sHR_PIN, kHR_PIN,
      rHL_PIN, sHL_PIN, kHL_PIN};

  // Servo rotation configs
  uint8_t init_angs[12]{35, 90, 180, 130, 83, 0, 35, 90, 180, 130, 83, 0};
  int8_t dir[12]{1, -1, -1, -1, 1, 1, -1, 1, 1, 1, -1, -1};
  // Angles in callibrated coordinate
  double current_angs[12] {};

  /**
   * CPG Gait Configs
   */
  uint8_t alpha{1000};
  double beta{0.5};
  double Ah{0.2};
  double Ak{0.2};
  uint8_t a{10};
  double omega_sw{5*M_PI};

  // oscillator signals
  double x[4] {1, 0, 0, 0};
  double y[4] {0, 0, 0, 0};
  // x_dot = dx / dt
  double x_dot[4] {};
  double y_dot[4] {};
  // r_square = x^2 + y^2
  double r_square[4] {};

  double omega_st {};
  double mu {};
  double omega[4] {};
  double theta[4][4] {};

  double rad2deg(double rad);
  void setup_servos();
  void shut_servos();
  void servo_write_angs(double angs[12], bool angs_in_rad=false);
  void cpg_signal();

public:
  QuadrupedRobot();
  ~QuadrupedRobot();

  // lock any robot motion
  bool lock{false};

  void switch_on();
  void switch_off();
  void bot_rest();
  void bot_stand();
  void bot_walk();
  void bot_trot();
};

#endif
