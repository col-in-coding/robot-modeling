#ifndef QUADRUPED_ROBOT_H
#define QUADRUPED_ROBOT_H

#include"Arduino.h"
#include <Servo.h>
#include <EEPROM.h>

//abbreviations
#define PT(s)  Serial.print(s)  //makes life easier
#define PTL(s) Serial.println(s)
#define PTF(s)  Serial.print(F(s)) //trade flash memory for dynamic memory with F() function
#define PTLF(s) Serial.println(F(s))

#define CMD_LEN 10

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

//on-board EEPROM addresses
#define ADDR_MELODY 1023 //melody will be saved at the end of the 1KB EEPROM, and is read reversely. That allows some flexibility on the melody length. 
#define ADDR_PIN 0                 // 16 byte array
#define ADDR_CALIB 16              // 16 byte array
#define ADDR_MID_SHIFT 32          // 16 byte array
#define ADDR_ROTATION_DIRECTION 48 // 16 byte array
#define ADDR_SERVO_ANGLE_RANGE 64  // 16 byte array
#define ADDR_MPUCALIB 80           // 16 byte array
#define ADDR_FAST 96               // 16 byte array
#define ADDR_SLOW 112              // 16 byte array
#define ADDR_LEFT 128              // 16 byte array
#define ADDR_RIGHT 144             // 16 byte array

#define ADDR_ADAPT_PARAM 160          // 16 x NUM_ADAPT_PARAM byte array
#define NUM_ADAPT_PARAM  2    // number of parameters for adaption
#define ADDR_SKILLS 200         // 1 byte for skill name length, followed by the char array for skill name
// then followed by i(nstinct) on progmem, or n(ewbility) on progmem


//servo constants
#define DOF 16
#define PWM_FACTOR 4
#define MG92B_MIN 170*PWM_FACTOR
#define MG92B_MAX 550*PWM_FACTOR
#define MG92B_RANGE 150

#define SG90_RANGE 150
#define SERVOMIN MG92B_MIN
#define SERVOMAX MG92B_MAX
#define SERVO_ANG_RANGE MG92B_RANGE

#define PWM_RANGE (SERVOMAX - SERVOMIN)

#define DEVICE_ADDRESS 0x50    //I2C Address of AT24C32D eeprom chip
#define WIRE_BUFFER 30 //Arduino wire allows 32 byte buffer, with 2 byte for address.
#define WIRE_LIMIT 16 //That leaves 30 bytes for data. use 16 to balance each writes
#define PAGE_LIMIT 32 //AT24C32D 32-byte Page Write Mode. Partial Page Writes Allowed
#define EEPROM_SIZE (65536/8)
#define SKILL_HEADER 3

//inline byte pin(byte idx) {
//  return EEPROM.read(ADDR_PIN + idx);
//}
//inline byte remapPin(byte offset, byte idx) {
//  return EEPROM.read(offset + idx);
//}
//inline byte servoAngleRange(byte idx) {
//  return 
//  return EEPROM.read(ADDR_SERVO_ANGLE_RANGE + idx);
//}
//inline int8_t middleShift(byte idx) {
//  return EEPROM.read(ADDR_MID_SHIFT + idx);
//}
//
//inline int8_t rotationDirection(byte idx) {
//  return EEPROM.read(ADDR_ROTATION_DIRECTION + idx);
//}
//inline int8_t servoCalib(byte idx) {
//  return EEPROM.read(ADDR_CALIB + idx);
//}

class QuadrupedRobot
{
  private:
    byte pins[16] = {
            0,       0,       0,       0,      
      rFL_PIN, rFR_PIN, rHR_PIN, rHL_PIN,
      sFL_PIN, sFR_PIN, sHR_PIN, sHL_PIN,
      kFL_PIN, kFR_PIN, kHR_PIN, kHL_PIN
    }; // 电机引脚
  
    //remap pins for different walking modes, pin4 ~ pin15
    byte fast[12] = {
      4, 4, 7, 7,
      8, 8, 11, 11,
      12, 12, 15, 15
    };
    
    byte slow[12] = {
      5, 5, 8, 8,
      9, 9, 10, 10,
      13, 13, 14, 14
    };

    byte left[12] = {
      5, 4, 7, 6,
      9, 8, 11, 10,
      13, 12, 15, 14
    };

    byte right[12] = {
      4, 5, 6, 7,
      8, 9, 10, 11,
      12, 13, 14, 15
    };

    byte melody[22] = {
      8, 13, 10, 13, 8,  0,  5,  8,  3,  5, 8,
      8, 8,  32, 32, 8, 32, 32, 32, 32, 32, 8,
    }; // 蜂鸣器

    float pulsePerDegree[DOF] = {};
    int8_t servoCalibs[DOF] = {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0};
    char currentAng[DOF] = {};
    int calibratedDuty0[DOF] = {};
    Servo *servos[DOF] = {
      nullptr, nullptr, nullptr, nullptr,
      nullptr, nullptr, nullptr, nullptr,
      nullptr, nullptr, nullptr, nullptr,
      nullptr, nullptr, nullptr, nullptr
    };

    int8_t middleShifts[DOF] = {
      0, 15, 0, 0,
      -45, -45, -45, -45,
      0, 0, 0, 0,
      0, 0, 0, 0
    };

    int8_t rotationDirections[DOF] = {
      1, -1, 1, 1,
      1, -1, 1, -1,
      1, -1, -1, 1,
      1, -1, -1, 1 //-1, 1, 1, -1
    };

    byte servoAngleRanges[DOF] =  {
      SG90_RANGE, SG90_RANGE, SG90_RANGE, SG90_RANGE,
      SG90_RANGE, SG90_RANGE, SG90_RANGE, SG90_RANGE,
      SG90_RANGE, SG90_RANGE, SG90_RANGE, SG90_RANGE,
      SG90_RANGE, SG90_RANGE, SG90_RANGE, SG90_RANGE
    };

//    uint8_t period = 0;
//    int8_t expectedRollPitch[2] = {0, 0};
    char* dutyAngles = NULL;
    
    // For test
    uint8_t period = 1;
    int8_t expectedRollPitch[2] = {0, 0};

    char token;
    char *lastCmd = new char[CMD_LEN];
    char *newCmd = new char[CMD_LEN];
    byte newCmdIdx = 0;
    byte hold = 0;

  public:

    QuadrupedRobot();
    ~QuadrupedRobot();

    //This function will write a 2 byte integer to the eeprom at the specified address and address + 1
    void EEPROMWriteInt(int p_address, int p_value);

    //This function will read a 2 byte integer from the eeprom at the specified address and address + 1
    int EEPROMReadInt(int p_address);

    int lookupAddressByName(char* skillName);

    void loadDataFromProgmem(unsigned int pgmAddress);

    void loadDataByOnboardEepromAddress(int onBoardEepromAddress);

    /**
     * 根据skillName，获取period和dutyAngles
     */
    void loadBySkillName(char* skillName);

    void assignSkillAddressToOnboardEeprom();

    void initServos();

    void shutServos();

    /**
     * 四条腿状态初始化
     */
    void awake();

    void writeServoMicroseconds(int i, int duty);

    void saveCalib(int8_t *var);

    void calibratedPWM(byte i, float angle);

    void allCalibratedPWM(char * dutyAng);

    void transform(char * target,  float speedRatio = 1, byte offset = 0);

    void behavior(int n, char** skill, float *speedRatio, int *pause);

};

#endif
