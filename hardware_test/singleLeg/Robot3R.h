/********************************************************************
 * Author: Haoxiang Li (Colin)
 * 3R Robot
 * Header file
 *******************************************************************/

#include <Servo.h>
#include <Arduino.h>

// servos
Servo hip0;
Servo hip1;
Servo knee;

const int servo_us_min {500};
const int servo_us_max {2500};
// hip joint 0, hip joint 1, knee joint
const int servo_pins [3] {11, 10, 9};
const float us_per_deg {11.1111111};

class Robot3R
{
private:
  const int dof {3};
  // servos configs: servo angles to align the coordinate
  int init_servo_angles [3] {45, 90, 45};
  bool motor_rot_reverse [3] {1, 0, 0};


public:
  Robot3R(); // constructor
  ~Robot3R(); // distructor

  void Robot3R::servo_set_us(int servo_id, int us);

  /**
   * assign a single line task
   */
  void set_joint_angles(int angles [3]);

  /**
   * assign a new task sequence
   */
  void set_joint_angles_sequence();

  /**
   * actuation
   */
  void run();

  void start();
};
