#include "instinct.h"
#include "QuadrupedRobot.h"
#include "Arduino.h"

#include <EEPROM.h>

QuadrupedRobot::QuadrupedRobot() {

}

QuadrupedRobot::~QuadrupedRobot() {
  
}

void QuadrupedRobot::EEPROMWriteInt(int p_address, int p_value) {
  byte lowByte = ((p_value >> 0) & 0xFF);
  byte highByte = ((p_value >> 8) & 0xFF);
  EEPROM.update(p_address, lowByte);
  EEPROM.update(p_address + 1, highByte);
}

int QuadrupedRobot::EEPROMReadInt(int p_address) {
 byte lowByte = EEPROM.read(p_address);
 byte highByte = EEPROM.read(p_address + 1);
 return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}

int QuadrupedRobot::lookupAddressByName(char* skillName) {
  PTLF("====> lookupAddressByName");
  int shift = 0;
  for (byte s = 0; s < NUM_SKILLS; s++) {//save skill info to on-board EEPROM, load skills to SkillList
    byte nameLen = EEPROM.read(ADDR_SKILLS + shift++);
 
    PT(nameLen);
    
    char* readName = new char[nameLen + 1];
    for (byte l = 0; l < nameLen; l++) {
      readName[l] = EEPROM.read(ADDR_SKILLS + shift++);
    }
    
    PT(readName);
    
    readName[nameLen] = '\0';
    if (!strcmp(readName, skillName)) {
      delete[]readName;
      return ADDR_SKILLS + shift;
    }
    delete[]readName;
    shift += 3;//1 byte type, 1 int address
  }
  PTLF("wrong key!");
  return -1;
}

void QuadrupedRobot::loadDataFromProgmem(unsigned int pgmAddress) {
 period = pgm_read_byte(pgmAddress);//automatically cast to char*
 for (int i = 0; i < 2; i++)
   expectedRollPitch[i] = pgm_read_byte(pgmAddress + 1 + i);
 byte frameSize = period > 1 ? WalkingDOF : 16;
 int len = period * frameSize;
 //delete []dutyAngles; //check here
 dutyAngles = new char[len];
 for (int k = 0; k < len; k++) {
   dutyAngles[k] = pgm_read_byte(pgmAddress + SKILL_HEADER + k);
   //PTL((int)dutyAngles[k]);
 }
}

void QuadrupedRobot::loadDataByOnboardEepromAddress(int onBoardEepromAddress) {
 char skillType = EEPROM.read(onBoardEepromAddress);
 unsigned int dataArrayAddress = EEPROMReadInt(onBoardEepromAddress + 1);
 delete[] dutyAngles;
 loadDataFromProgmem(dataArrayAddress) ;
 /*PTF("free memory: ");
   PTL(freeMemory());*/
}

void QuadrupedRobot::loadBySkillName(char* skillName) {
  PTLF("===> loadBySkillName: ");
  PT(skillName);
//  int onBoardEepromAddress = lookupAddressByName(skillName);
//  PT(onBoardEepromAddress);
// if (onBoardEepromAddress == -1)
//   return;
// loadDataByOnboardEepromAddress(onBoardEepromAddress);
}

void QuadrupedRobot::assignSkillAddressToOnboardEeprom() {
 int shift = 0;
 for (byte s = 0; s < sizeof(progmemPointer) / 2; s++) { //save skill info to on-board EEPROM, load skills to SkillList
   byte nameLen = EEPROM.read(ADDR_SKILLS + shift++); //without last type character
   shift += nameLen;
   char skillType = EEPROM.read(ADDR_SKILLS + shift++);
   EEPROMWriteInt(ADDR_SKILLS + shift, (int)progmemPointer[s]);
   shift += 2;
 }
}

void QuadrupedRobot::initServos() {
 for (int i = 0; i < DOF; i++) {
   if (pins[i] != 0) {
     if (servos[i] == nullptr) {
       servos[i] = new Servo();
       servos[i]->attach(pins[i], SERVOMIN, SERVOMAX);
     } else {
       servos[i]->attach(pins[i], SERVOMIN, SERVOMAX);
     }
   } else {
     servos[i] = nullptr;
   }
 }
}

void QuadrupedRobot::shutServos() {
 for (int8_t i = DOF - 1; i >= 0; i--) {
   if (servos[i] != nullptr) {
     servos[i]->detach();
   }
 }
}

void QuadrupedRobot::awake() {
  strcpy(lastCmd, "rest");
  loadBySkillName("rest");  // get period, dutyAngles
  
  for (int8_t i = DOF - 1; i >= 0; i--) {
    pulsePerDegree[i] = float(PWM_RANGE) / servoAngleRanges[i];
    calibratedDuty0[i] =  SERVOMIN + PWM_RANGE / 2 + float(middleShifts[i] + servoCalibs[i]) * pulsePerDegree[i]  * rotationDirections[i];
    PTL(calibratedDuty0[i]);
    calibratedPWM(i, dutyAngles[i]);
    delay(100);
    
  }
//  randomSeed(analogRead(0));//use the fluctuation of voltage caused by servos as entropy pool
  shutServos();
  token = 'd';
}

void QuadrupedRobot::writeServoMicroseconds(int i, int duty) {
 if (servos[i] != nullptr) {
   if (!servos[i]->attached()) {
     servos[i]->attach(pins[i], SERVOMIN, SERVOMAX);
   }
   servos[i]->writeMicroseconds(duty);
   delayMicroseconds(1000);
 }
}

void QuadrupedRobot::saveCalib(int8_t *var) {
 for (byte i = 0; i < DOF; i++) {
   EEPROM.update(ADDR_CALIB + i, var[i]);
   calibratedDuty0[i] = SERVOMIN + PWM_RANGE / 2 + float(middleShifts[i] + var[i]) * pulsePerDegree[i] * rotationDirections[i];
 }
}

void QuadrupedRobot::calibratedPWM(byte i, float angle) {
 /*float angle = max(-SERVO_ANG_RANGE/2, min(SERVO_ANG_RANGE/2, angle));
   if (i > 3 && i < 8)
   angle = max(-5, angle);*/
 if (i <= 2) {
   angle = -angle; //revert the angle of head,neck and tail servos
 }
 currentAng[i] = angle;
 int duty = calibratedDuty0[i] + angle * pulsePerDegree[i] * rotationDirections[i];
 duty = max(SERVOMIN , min(SERVOMAX , duty));
 //pwm.setPWM(pin(i), 0, duty);
 writeServoMicroseconds(i,duty);
 /*
 angle = map(duty, SERVOMIN, SERVOMAX, 0, 180);
 writeServo(i, angle);
 */
}

void QuadrupedRobot::allCalibratedPWM(char * dutyAng) {
 for (int8_t i = DOF - 1; i >= 0; i--) {
   calibratedPWM(i, dutyAng[i]);
 }
}

void QuadrupedRobot::transform(char * target,  float speedRatio = 1, byte offset = 0) {
 char *diff = new char[DOF - offset], maxDiff = 0;
 for (byte i = offset; i < DOF; i++) {
   diff[i - offset] =   currentAng[i] - target[i - offset];
   maxDiff = max(maxDiff, abs( diff[i - offset]));
 }
 byte steps = byte(round(maxDiff / 1.0/*degreeStep*/ / speedRatio));//default speed is 1 degree per step
 for (byte s = 0; s <= steps; s++)
   for (byte i = offset; i < DOF; i++) {
     float dutyAng = (target[i - offset] + (steps == 0 ? 0 : (1 + cos(M_PI * s / steps)) / 2 * diff[i - offset]));
     calibratedPWM(i,  dutyAng);
   }
 delete [] diff;
}

void QuadrupedRobot::behavior(int n, char** skill, float *speedRatio, int *pause) {
 for (byte i = 0; i < n; i++) {
   loadBySkillName(skill[i]);
   transform(dutyAngles, speedRatio[i]);
   delay(pause[i]);
 }
}
