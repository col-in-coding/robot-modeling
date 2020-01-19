/********************************************************************
 * Author: Haoxiang Li (Colin)
 * 3R Robot
 * Source Code
 *******************************************************************/

#include "Robot3R.h"
#include "Arduino.h"
#include <Servo.h>

Robot3R::Robot3R() {
  for (int i = 0; i < dof; i++) {
    hip0.attach(11);
    hip1.attach(10);
    knee.attach(9);
  }
}

Robot3R::~Robot3R() {
}

void Robot3R::servo_set_us(int servo_id, int us) {
  switch (servo_id) {
   case 0:
    hip0.writeMicroseconds(us);
    break;
   case 1:
    hip1.writeMicroseconds(us);
    break;
   case 2:
    knee.writeMicroseconds(us);
    break;
  }
  delay(1000);
}

void Robot3R::set_joint_angles(int theta [3]) {
  for (int i = 0; i < dof; i++) {
    int servo_ang = theta[i] + init_servo_angles[i];
    int us = servo_us_min + servo_ang * us_per_deg;
    Serial.println(us);
    servo_set_us(i, us);
  }
}

void Robot3R::set_joint_angles_sequence() {

}

void Robot3R::run() {
  
}

void Robot3R::start(){
  int setup_ang [3] {0, 0, 0};
  set_joint_angles(setup_ang);
}
