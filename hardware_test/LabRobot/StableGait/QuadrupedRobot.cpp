
#include <Arduino.h>
#include "QuadrupedRobot.h"

/**
 * Switch on the robot
 */
void QuadrupedRobot::switch_on()
{
  Serial.println("Robot Switch On");
  setup_servos();
  bot_rest();
  delay(2000);
  shut_servos();
  clear_cmd();
}

/**
 * Switch off the robot
 */
void QuadrupedRobot::switch_off()
{
  Serial.println("Robot Switch Off");
  shut_servos();
  clear_cmd();
}

void QuadrupedRobot::clear_cmd()
{
  command_code = -1;
}

void QuadrupedRobot::setup_servos()
{
  for (size_t i = 0; i < DOF; i++)
  {
    if (servos_ptr[i] == nullptr)
      servos_ptr[i] = new Servo();
    servos_ptr[i]->attach(servo_pins[i], SERVOMIN, SERVOMAX);
  }
}

void QuadrupedRobot::shut_servos()
{
  for (size_t i = 0; i < DOF; i++)
  {
    if (servos_ptr[i] != nullptr)
    {
      servos_ptr[i]->detach();
    }
  }
}

/**
 * Check Interrupts
 * New Command from Serial Port
 */
bool QuadrupedRobot::interrupt()
{
  String serial_in_str {""};
  while(Serial.available() > 0) {
    char inChar = Serial.read();
    serial_in_str += (char)inChar;
    // wait for input buffer read
    delay(10);
  }
  if(serial_in_str != "")
  {
    Serial.print("Command received: ");
    Serial.println(serial_in_str);
    command_code = serial_in_str.toInt();
  }
}

void QuadrupedRobot::parse_code_run()
{
  switch(command_code) {
    case 1:
      switch_on();
      break;
    case 2:
      bot_stand();
      break;
    case 3:
      bot_walk();
      break;
    case 0:
      switch_off();
      break;
    case 998:
      adjust();
      break;
    default:
      ;// doing nothing
  }
}

/**
 * Synchronous Servo Move
 * Every servo should move to the desired position at the end of the timestep
 */
void QuadrupedRobot::servo_move(float *angs_ptr)
{
  unsigned long starttime = millis();
  unsigned long timenow = starttime;
  unsigned long servo_time[DOF] {}; // Timestamp of servo
  float joint_angs_now[DOF] {}; 
  float joint_degs_speed[DOF] {};
  for (size_t i = 0; i < DOF; i++)
  {
    joint_degs_speed[i] = (angs_ptr[i] - joint_angs_pre[i]) / timestep;
    joint_angs_now[i] = joint_angs_pre[i]; // Reset Joint angles
    servo_time[i] = starttime; // Reset Servo Time
  }
  while((timenow - starttime) < timestep) {
    for (size_t i = 0; i < DOF; i++)
    {
      timenow = millis();
      joint_angs_now[i] += joint_degs_speed[i] * (timenow - servo_time[i]);
    }
    servo_write_angs(joint_angs_now);
    timenow = millis();
  }
  // last run to assure that all servo has been desired position
  for (size_t i = 0; i < DOF; i++)
  {
    servo_write_angs(angs_ptr);
  }
  
  
}

/**
 * Servo Write Current Angles
 * Current Angles is in Callibrated Coordinate,
 * which should be transfered to real angle degrees before using.
 */
void QuadrupedRobot::servo_write_angs(float *angs_ptr)
{
  float real_ang{};
  float pulsewidth{};
  for (size_t i = 0; i < DOF; i++)
  {
    real_ang = init_servo_deg[i] + servo_dir[i] * angs_ptr[i];
    pulsewidth = 500 + real_ang / 90.0 * 1000;
    if (!servos_ptr[i]->attached())
    {
      servos_ptr[i]->attach(servo_pins[i], SERVOMIN, SERVOMAX);
    }
    servos_ptr[i]->writeMicroseconds(pulsewidth);
    // update current angles buffer
    joint_angs_pre[i] = angs_ptr[i];
  }
}

/**
 * Only used for adjusting legs
 */
void QuadrupedRobot::adjust()
{
  servo_write_angs(adjust_angs);
  clear_cmd();
}

/**
 * Stand up
 * Non-periodic
 */
void QuadrupedRobot::bot_stand()
{
  servo_write_angs(stand_angs);
  clear_cmd();
}

/**
 * Sleep mode
 * Non-periodic
 */
void QuadrupedRobot::bot_rest()
{
  servo_write_angs(rest_angs);
  clear_cmd();
}

/**
 * walk
 * Periodic
 */
void QuadrupedRobot::bot_walk()
{
  Serial.print("walk");
}

QuadrupedRobot::~QuadrupedRobot()
{
  for (size_t i = 0; i < DOF; i++)
  {
    delete servos_ptr[i];
  }
}
