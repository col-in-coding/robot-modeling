
#include <Arduino.h>
#include "QuadrupedRobot.h"


void QuadrupedRobot::setup_servos()
{
  PTL("Setup Servos");
  for (size_t i = 0; i < DOF; i++)
  {
    if (servos[i] == nullptr)
      servos[i] = new Servo();
    servos[i]->attach(pins[i], SERVOMIN, SERVOMAX);
  }
}

void QuadrupedRobot::shut_servos()
{
  for (size_t i = 0; i < DOF; i++)
  {
    if (servos[i] != nullptr)
    {
      servos[i]->detach();
    }
  }
}

void QuadrupedRobot::servo_write_angs(double angs[DOF], bool angs_in_rad=false)
{
//  PTL("Servo Write Angles");
  double real_ang{};
  double pulsewidth{};
  for (size_t i = 0; i < DOF; i++)
  {
    if (angs_in_rad) {
      angs[i] = angs[i] * 180 / M_PI;
    }
    real_ang = init_angs[i] + dir[i] * angs[i];
    pulsewidth = 500 + real_ang / 90.0 * 1000;
    if (!servos[i]->attached())
    {
      servos[i]->attach(pins[i], SERVOMIN, SERVOMAX);
    }
    servos[i]->writeMicroseconds(pulsewidth);
  }
  delay(5);
}

QuadrupedRobot::QuadrupedRobot()
{
  // init data for CPG
  omega_st = omega_sw * (1 - beta) / beta;
  mu = Ah * Ah;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      theta[i][j] = 2 * M_PI * (phi[i] - phi[j]);
    }
  }
}

QuadrupedRobot::~QuadrupedRobot()
{
}

void QuadrupedRobot::switch_on()
{
  if(lock) return;
  PTL("Robot Switch On");
  setup_servos();
  bot_rest();
  delay(1000);
  shut_servos();
}

void QuadrupedRobot::switch_off()
{
  if(lock) return;
  PTL("Robot Switch Off");
  shut_servos();
}

void QuadrupedRobot::bot_rest()
{
  if(lock) return;
  PTL("Robot Rest");
  double rest_angs[12]{0, -55, 130, 0, -55, 130, 0, 55, -130, 0, 55, -130};
  servo_write_angs(rest_angs);
}

void QuadrupedRobot::bot_stand()
{
  if(lock) return;
  PTL("Robot Stand");
  double stand_angs[12]{0, -30, 60, 0, -30, 60, 0, 30, -60, 0, 30, -60};
  servo_write_angs(stand_angs);
}

void QuadrupedRobot::bot_walk()
{
  if(lock) return;
  double angs[12] {};
  double stand_angs[12]{0, -30, 60, 0, -30, 60, 0, 30, -60, 0, 30, -60};
  // dt = 0.0001s, dt * 100 = 0.01s
  for (size_t itr = 0; itr < 100; itr++) {
    for (int i = 0; i < 1; i++) {
      r_square[i] = x[i] * x[i] + y[i] * y[i]; // 1
   
      // Frequency of the oscillator
      omega[i] = omega_st / (exp(- a * y[i]) + 1) + omega_sw / (exp(a * y[i]) + 1); // 9.42
      
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
  //    x_dot[i] += cos(theta[0][i]) * x[0] - sin(theta[0][i]) * y[0];
  //    y_dot[i] += sin(theta[0][i]) * x[0] + cos(theta[0][i]) * y[0];
  //    x_dot[i] += cos(theta[1][i]) * x[1] - sin(theta[1][i]) * y[1];
  //    y_dot[i] += sin(theta[1][i]) * x[1] + cos(theta[1][i]) * y[1];
  //    x_dot[i] += cos(theta[2][i]) * x[2] - sin(theta[2][i]) * y[2];
  //    y_dot[i] += sin(theta[2][i]) * x[2] + cos(theta[2][i]) * y[2];
  //    x_dot[i] += cos(theta[3][i]) * x[3] - sin(theta[3][i]) * y[3];
  //    y_dot[i] += sin(theta[3][i]) * x[3] + cos(theta[3][i]) * y[3];
  
      // update the signal values
      x[i] += x_dot[i] * dt;
      y[i] += y_dot[i] * dt;
      
      // Hip Joint Angels
      angs[i] = 0 + stand_angs[i];
      angs[i + 1] = x[i] * 180 / M_PI + stand_angs[i + 1];

      // Knee Joint Angel
      if (y[i] > 0) {
        angs[i + 2] = stand_angs[i + 2];
      } else {
        angs[i + 2] = - y[i] * Ak * 180 / (M_PI * Ah) + stand_angs[i + 2];
      }
    }
  }

//  PTL("signals: ");
//  PTL(x[0]);
//  PTL(y[0]);
//  PTL("Angles: ");
//  PTL(angs[1]);
//  PTL(angs[2]);

  servo_write_angs(angs);
}
