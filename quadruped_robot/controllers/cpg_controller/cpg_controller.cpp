// File:          cpg_controller.cpp
// Date:          Feb-19-2020
// Description:
// Author:        Colin
// Modifications: Feb-19-2020

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <cmath>

#define TIME_STEP 16
#define T 1


using namespace webots;

double deg_2_rad(double x) {
  return x / 180.0 * M_PI;
}

int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  
  Motor *servos[12];
  char servo_names[12][12] {
     "servo_hip1x", "servo_hip1z", "servo_knee1",
     "servo_hip2x", "servo_hip2z", "servo_knee2",
     "servo_hip3x", "servo_hip3z", "servo_knee3",
     "servo_hip4x", "servo_hip4z", "servo_knee4"
  };
  for (int i = 0; i < 12; i++) {
    servos[i] = robot->getMotor(servo_names[i]);
  }

  // Params
  double Ah = 10;
  double Ak = 15;
  double t = 0.0;

  
  const double init_angs[12] {
    0, -15, 30, 0, -15, 30, 0, 15, -30, 0, 15, -30
  };
  
  double angs[12] {};
  for (int i = 0; i < 12; i++) {
    angs[i] = init_angs[i];
    servos[i]->setPosition(deg_2_rad(angs[i]));
  } 
  

 
  while (robot->step(TIME_STEP) != -1) {
    angs[0] = 0 + init_angs[0];
    angs[1] = Ah * sin(2 * M_PI / T * t - M_PI / 2) + init_angs[1];
    
    double knee1 = Ak * sin(2 * M_PI / T * t );
    double knee2 = Ak * sin(2 * M_PI / T * t + M_PI);
    double knee3 = Ak * sin(2 * M_PI / T * t);
    double knee4 = Ak * sin(2 * M_PI / T * t + M_PI);
    
    if (knee1 < 0) knee1 = 0;
    if (knee2 < 0) knee2 = 0;
    if (knee3 < 0) knee3 = 0;
    if (knee4 < 0) knee4 = 0;
    
    angs[2] = knee1 + init_angs[2];
    angs[3] = 0 + init_angs[3];
    angs[4] = Ah * sin(2 * M_PI / T * t + M_PI / 2) + init_angs[4];
    angs[5] = knee2 + init_angs[5];
    angs[6] = 0 + init_angs[6];
    angs[7] = Ah * sin(2 * M_PI / T * t - M_PI / 2) + init_angs[7];
    angs[8] = -knee3 + init_angs[8];
    angs[9] = 0 + init_angs[9];
    angs[10] = Ah * sin(2 * M_PI / T * t + M_PI / 2) + init_angs[10];
    angs[11] = -knee4 + init_angs[11];

    for (int i = 0; i < 12; i++) {
      servos[i]->setPosition(deg_2_rad(angs[i]));
    }    
    
    
    t += (double)TIME_STEP / 1000.0;
  };


  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
