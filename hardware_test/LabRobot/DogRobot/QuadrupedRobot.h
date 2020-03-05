#ifndef QUADRUPED_ROBOT_H
#define QUADRUPED_ROBOT_H

#include"Arduino.h"
#include <Servo.h>

// print
#define PT(s)  Serial.print(s)
#define PTL(s) Serial.println(s)
#define PTF(s)  Serial.print(F(s)) //trade flash memory for dynamic memory with F() function
#define PTLF(s) Serial.println(F(s))

#define rFL_PIN   13   //  front left shoulder roll
#define rFR_PIN   14   //  front right shoulder roll
#define rHR_PIN   3   //  hind right shoulder roll
#define rHL_PIN   4   //  hind left shouuld roll
#define sFL_PIN 5     // front left shoulder pitch
#define sFR_PIN 7     // front right shoulder pitch
#define sHR_PIN 9     // hind right shoulder pitch
#define sHL_PIN 11    // hind left shouuld pitch
#define kFL_PIN 6     // front left knee
#define kFR_PIN 8     // front right knee
#define kHR_PIN 10     // hind right knee
#define kHL_PIN 12    // hind left knee

//servo constants
#define SERVOMIN 500
#define SERVOMAX 2500
#define PWM_RANGE (SERVOMAX - SERVOMIN)

#define DOF 12


class QuadrupedRobot
{
  private:
    Servo *servos[DOF] = {
      nullptr, nullptr, nullptr, nullptr,
      nullptr, nullptr, nullptr, nullptr,
      nullptr, nullptr, nullptr, nullptr
    };
    byte pins[DOF] = {     
      rFL_PIN, rFR_PIN, rHR_PIN, rHL_PIN,
      sFL_PIN, sFR_PIN, sHR_PIN, sHL_PIN,
      kFL_PIN, kFR_PIN, kHR_PIN, kHL_PIN
    };

    char* duty_angles = NULL;

  public:
    QuadrupedRobot();
    ~QuadrupedRobot();

    void switch_on();
    void switch_off();

    void setup_servos();

    void shut_servos();

};

#endif
