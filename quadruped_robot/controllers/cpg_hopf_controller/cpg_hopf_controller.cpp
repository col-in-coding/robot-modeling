// File:          cpg_hopf_controller.cpp
// Date:          Feb-20-2020
// Description:
// Author:        Colin
// Modifications: Feb-20-2020
                                   
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <cmath>

#define alpha 100
#define beta 0.5
#define Ah 0.2
#define Ak 0.1
#define a 10
#define omega_sw 3 * M_PI
#define phi_LF 0
#define phi_RF 0.5
#define phi_LH 0.5
#define phi_RH 0


// All the webots classes are defined in the "webots" namespace
using namespace webots;

double deg_2_rad(double x) {
  return x / 180.0 * M_PI;
}

int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

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

  const double init_angs[12] {
    0, -15, 30, 0, -15, 30, 0, 15, -30, 0, 15, -30
  };
  double angs[12] {};
  for (int i = 0; i < 12; i++) {
    angs[i] = init_angs[i];
    servos[i]->setPosition(deg_2_rad(angs[i]));
  } 

  /**
   * intermediate params
   */
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
  double phi[4] {phi_LF, phi_RF, phi_RH, phi_LH};
  double omega[4] {};
  double theta[4][4] {};
  double dt {};

  dt = (double)timeStep / 2000;
  omega_st = omega_sw * (1 - beta) / beta;
  mu = Ah * Ah;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      theta[i][j] = 2 * M_PI * (phi[i] - phi[j]);
    }
  }

  // Main Loop
  while (robot->step(timeStep) != -1) {

    for (int i = 0; i < 4; i++) {
      r_square[i] = x[i] * x[i] + y[i] * y[i];

      // Frequency of the oscillator
      omega[i] = omega_st / (exp(- a * y[i]) + 1) + omega_sw / (exp(a * y[i]) + 1);
      
      // HOPF Oscillator
      x_dot[i] = alpha * (mu - r_square[i]) * x[i] - omega[i] * y[i];
      y_dot[i] = alpha * (mu - r_square[i]) * y[i] + omega[i] * x[i];
      
      /**
       * coupling terms
       * [
       *   R11 R21 R31 R41
       *   R12 R22 R32 R42
       *   R13 R23 R33 R43
       *   R14 R24 R34 R44
       * ]
       */
      x_dot[i] += cos(theta[0][i]) * x[0] - sin(theta[0][i]) * y[0];
      y_dot[i] += sin(theta[0][i]) * x[0] + cos(theta[0][i]) * y[0];
      x_dot[i] += cos(theta[1][i]) * x[1] - sin(theta[1][i]) * y[1];
      y_dot[i] += sin(theta[1][i]) * x[1] + cos(theta[1][i]) * y[1];
      x_dot[i] += cos(theta[2][i]) * x[2] - sin(theta[2][i]) * y[2];
      y_dot[i] += sin(theta[2][i]) * x[2] + cos(theta[2][i]) * y[2];
      x_dot[i] += cos(theta[3][i]) * x[3] - sin(theta[3][i]) * y[3];
      y_dot[i] += sin(theta[3][i]) * x[3] + cos(theta[3][i]) * y[3];
 
     // update the signal values
      x[i] += x_dot[i] * dt;
      y[i] += y_dot[i] * dt;
      
      // Hip Joint Angels
      angs[i] = 0 + deg_2_rad(init_angs[i]);
      angs[i + 1] = x[i] + deg_2_rad(init_angs[i + 1]);
      // Knee Joint Angel
      if (y[i] > 0) {
        angs[i + 2] = deg_2_rad(init_angs[i + 2]);
      } else {
        angs[i + 2] = y[i] + deg_2_rad(init_angs[i + 2]);
      }
      
    }

    for (int i = 0; i < 12; i++) {
      servos[i]->setPosition(angs[i]);
    }
    
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
