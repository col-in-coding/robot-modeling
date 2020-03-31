#ifndef QUADRUPED_ROBOT_H
#define QUADRUPED_ROBOT_H

#include <Arduino.h>
#include <Servo.h>

#define PT(s) Serial.print(s)
#define PTL(s) Serial.println(s)
#define PTF(s) Serial.print(F(s)) //trade flash memory for dynamic memory with F() function
#define PTLF(s) Serial.println(F(s))

//servo constants
#define SERVOMIN 500
#define SERVOMAX 2500

#define DOF 12

class QuadrupedRobot
{
private:
  // For any non-periodic command, set it to -1 in the end of the action
  int command_code{-1};
  Servo *servos_ptr[DOF]{
      nullptr, nullptr, nullptr,
      nullptr, nullptr, nullptr,
      nullptr, nullptr, nullptr,
      nullptr, nullptr, nullptr};
  // HipZ, HipX, Knee
  byte servo_pins[DOF]{
      3, 4, 5,
      6, 7, 8,
      9, 10, 11,
      12, 13, 14};
  // Time taken by each sequence, used in servo_write_angs()
  unsigned long timestep = 500;

  // Servo rotation configs
  const float init_servo_deg[12]{35, 90, 180, 130, 83, 0, 35, 85, 180, 130, 83, 0};
  const int8_t servo_dir[12]{1, -1, -1, -1, 1, 1, -1, 1, 1, 1, -1, -1};
  
  // float joint_angs_now[12]{}; //Current joint angles, in degrees
  float joint_angs_pre[12]{}; //Previous joint angles

  // Angles in callibrated coordinate
  const float stand_angs[12]{0, -30, 60, 0, -30, 60, 0, 30, -60, 0, 30, -60};
  const float rest_angs[12]{0, -55, 130, 0, -55, 130, 0, 55, -130, 0, 55, -130};
  const float adjust_angs[12] {};

  void setup_servos();
  void shut_servos();
  void servo_move(float *angs_ptr);
  void servo_write_angs(float *angs_ptr);
  void clear_cmd();

public:
  ~QuadrupedRobot();
  bool interrupt();
  void parse_code_run();

  void switch_on();
  void switch_off();

  void adjust();
  void bot_rest();
  void bot_stand();
  void bot_walk();
};

#endif
