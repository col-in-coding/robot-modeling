#ifndef QUADRUPED_ROBOT_H
#define QUADRUPED_ROBOT_H

#include"Arduino.h"
#include <Servo.h>

#define PT(s)  Serial.print(s)
#define PTL(s) Serial.println(s)
#define PTF(s)  Serial.print(F(s)) //trade flash memory for dynamic memory with F() function
#define PTLF(s) Serial.println(F(s))

// Servo Pin configs
#define rFL_PIN 3
#define sFL_PIN 4
#define kFL_PIN 5
#define rFR_PIN 6
#define sFR_PIN 7
#define kFR_PIN 8
#define rHR_PIN 9
#define sHR_PIN 10
#define kHR_PIN 11
#define rHL_PIN 12
#define sHL_PIN 13
#define kHL_PIN 14

//servo constants
#define SERVOMIN 500
#define SERVOMAX 2500
#define PWM_RANGE (SERVOMAX - SERVOMIN)

#define DOF 12


class QuadrupedRobot
{
  private:
    Servo *servos[DOF] = {
      nullptr, nullptr, nullptr,
      nullptr, nullptr, nullptr,
      nullptr, nullptr, nullptr,
      nullptr, nullptr, nullptr
    };
    byte pins[DOF] = {     
      rFL_PIN, sFL_PIN, kFL_PIN,
      rFR_PIN, sFR_PIN, kFR_PIN,
      rHR_PIN, sHR_PIN, kHR_PIN,
      rHL_PIN, sHL_PIN, kHL_PIN
    };

    char* duty_angles = NULL;

    // Servo rotation configs
    uint8_t init_angs[12] = {45, 90, 180, 130, 83, 0, 45, 90, 180, 130, 83, 0};
    int8_t dir[12] = {1, -1, -1, -1, 1, 1, -1, 1, 1, 1, -1, -1};

    void setup_servos();
    void shut_servos();
    void servo_write_angs(double angs[12]);

  public:
    QuadrupedRobot();
    ~QuadrupedRobot();

    void switch_on();
    void switch_off();
    void bot_rest();
    void bot_stand();
};

#endif
