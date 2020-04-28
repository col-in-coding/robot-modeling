#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <Arduino.h>
#include "Actuator.h"
#include "Adafruit_PWMServoDriver.h"


struct Servo
{
  uint8_t id;
  int8_t dir;
  float init_ang; // in degrees
  float angle; // current angle in degrees
  Servo(uint8_t _id, int8_t _dir, float _init_ang) {
    id = _id;
    dir = _dir;
    init_ang = _init_ang;
    angle = _init_ang;
  }
};
typedef Servo* ServoPtr;


struct Leg
{
  ServoPtr sRoll;
  ServoPtr sPitch;
  ServoPtr sKnee;
};
typedef Leg* LegPtr;


class Actuator
{
private:
  const uint8_t dof{12};
  const uint8_t freq{50}; // Hz
  const uint16_t resolution{4096}; // 12 bits
  Adafruit_PWMServoDriver *pwm;
  LegPtr legFL;
  LegPtr legFR;
  LegPtr legRR;
  LegPtr legRL;

  void set_servo_pulse(ServoPtr servo, double pulse);
  void write_servo_ang(ServoPtr servo, float ang);

public:
  Actuator(/* args */);
  ~Actuator();
  void init();
  void adjust();

};

#endif
