#include <Arduino.h>
#include "Actuator.h"
#include "Adafruit_PWMServoDriver.h"


/**
 * 
 *         ^ Y
 *         |
 * back    |---> X   forward
 *       Z
 * 
 */
Actuator::Actuator()
{
  // init servo driver, default address 0x40
  pwm = new Adafruit_PWMServoDriver();

}


void Actuator::init()
{
  Serial.println("init pwm driver");
  pwm->begin();
  pwm->setPWMFreq(freq);
  Serial.println("init legs");
  legFL = new Leg();
  legFR = new Leg();
  legRR = new Leg();
  legRL = new Leg();
  LegPtr legs[4] {legFL, legFR, legRR, legRL};
  // 1 the same with axis, -1 the opposite
  const int8_t servo_dir[12]{1, -1, -1, 1, 1, 1, -1, 1, 1, -1, -1, -1};
  const float init_ang[12]{35, 90, 180, 125, 83, 5, 40, 85, 180, 127, 83, 0};
  for (uint8_t i = 0; i < 4; i++)
  {
    legs[i]->sRoll = new Servo(i * 3, servo_dir[i * 3], init_ang[i * 3]);
    legs[i]->sPitch = new Servo(i * 3 + 1, servo_dir[i * 3 + 1], init_ang[i * 3 + 1]);
    legs[i]->sKnee = new Servo(i * 3 + 2, servo_dir[i * 3 + 2], init_ang[i * 3 + 2]);
  }
}


void Actuator::set_servo_pulse(ServoPtr servo, double pulse)
{
  double pulseleng;
  pulseleng = 1000000; // 1,000,000 per second
  pulseleng /= freq;
  pulseleng /= resolution;
  pulse *= 1000; // convert to us
  pulse /= pulseleng;
  pwm->setPWM(servo->id, 0, pulse);
}


void Actuator::write_servo_ang(ServoPtr servo , float angle)
{
  double pulse;
  pulse = 0.5 + angle / 90.0;
  set_servo_pulse(servo, pulse);
  servo->angle = angle;
}


void Actuator::adjust() {
  // adjust front left leg
  write_servo_ang(legFL->sRoll, legFL->sRoll->init_ang);
  write_servo_ang(legFL->sPitch, legFL->sPitch->init_ang);
  write_servo_ang(legFL->sKnee, legFL->sKnee->init_ang);

  write_servo_ang(legFR->sRoll, legFR->sRoll->init_ang);
  write_servo_ang(legFR->sPitch, legFR->sPitch->init_ang);
  write_servo_ang(legFR->sKnee, legFR->sKnee->init_ang);

  write_servo_ang(legRR->sRoll, legRR->sRoll->init_ang);
  write_servo_ang(legRR->sPitch, legRR->sPitch->init_ang);
  write_servo_ang(legRR->sKnee, legRR->sKnee->init_ang);
  
  write_servo_ang(legRL->sRoll, legRL->sRoll->init_ang);
  write_servo_ang(legRL->sPitch, legRL->sPitch->init_ang);
  write_servo_ang(legRL->sKnee, legRL->sKnee->init_ang);
}


Actuator::~Actuator()
{
  delete pwm;
  pwm = nullptr;
}
