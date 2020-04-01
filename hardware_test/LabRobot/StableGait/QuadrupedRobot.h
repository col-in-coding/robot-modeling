#ifndef QUADRUPED_ROBOT_H
#define QUADRUPED_ROBOT_H

#include <Arduino.h>
#include <Servo.h>
#include <math.h>

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
  // Time taken by each sequence, used in servo_move()
  unsigned long timestep = 500;
  int steplen = 80;

  /**
   * Servo rotation configs
   * Real servo coord to calibrated coord: X-roll, Y-yaw, Z-pitch
   */
  const float init_servo_deg[12]{35, 90, 180, 125, 83, 5, 40, 85, 180, 127, 83, 0};
  const int8_t servo_dir[12]{-1, -1, -1, -1, 1, 1, 1, 1, 1, 1, -1, -1};
  const float toe_out0{30}; // outward distance of toe during stand, in mm
  const float dist_ag{30};  // distance between alfa and gamma axis, in mm
  const float thigh{107};
  const float calf{90};
  /**
   * vleg_len: virtual leg length (composite with 2 limbs)
   * alfa, gamma, beta is conresponding to angles on HipZ, Knee, HipX axises
   */
  const float vleg_len0{160};
  const float height0{sqrt(pow(vleg_len0 + dist_ag, 2) - pow(toe_out0, 2))};
  const float alfa0{acos((thigh * thigh + vleg_len0 * vleg_len0 - calf * calf) / (2 * thigh * vleg_len0))};
  const float beta0{M_PI - acos((thigh * thigh + calf * calf - vleg_len0 * vleg_len0) / (2 * thigh * calf))};
  const float gamma0{asin(toe_out0 / (vleg_len0 + dist_ag))};
  /**
   * State Variables
   * angles in radiums
   * Foot position, order: FLxyz, FRxyz, RLxyz, RRxyz
   */
  float joint_angs_pre[12]{}; //Previous joint angles
  float joint_angs_new[12]{};
  float vlegs_len[4]{};
  float foot_pos[12] {};
  /**
   * Configed States
   * Angles in callibrated coordinate
   */
  const float stand_angs[12]{0, -30, 60, 0, -30, 60, 0, 30, -60, 0, 30, -60};
  const float rest_angs[12]{0, -55, 130, 0, -55, 130, 0, 55, -130, 0, 55, -130};
  const float adjust_angs[12] {};

  // 
  // X points to forward, Z points to upward
  
  
  void setup_servos();
  void shut_servos();
  void servo_move(float *angs_ptr);
  void servo_write_angs(float *angs_ptr, bool is_in_degrees);
  void clear_cmd();

  void body_xyz(float x, float y, float z);
  void body_move_xyz(float dx, float dy, float dz);
  void foot_step(int i, float x, float z);
  void foot_move_xyz(int i, float dx, float dy, float dz);

  void inverse_kinematics();
  float gamma_left(float dy, float dz);
  float gamma_right(float dy, float dz);
  float vleg_left(float dx, float dy, float gamma);
  float vleg_right(float dx, float dy, float gamma);
  float beta_front(float vleg_len);
  float beta_rear(float vleg_len);
  float alfa_front(float dx, float beta, float vleg_len);
  float alfa_rear(float dx, float beta, float vleg_len);

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
