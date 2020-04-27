#include <Wire.h>
#include "Adafruit_PWMServoDriver.h"

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 50;   // 50 Hz
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000;
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

void writeServo(uint8_t n,uint8_t angle){
  double pulse;
  pulse=0.5+angle/90.0;
  setServoPulse(n,pulse);
}

void setup() {
  Serial.begin(9600);
  Serial.println("16 channel PWM test!");
  pwm.begin();
  pwm.setPWMFreq(50);  // This is the maximum PWM frequency
}

String inString = "";
void loop() {
  if (Serial.available() > 0) {
    if (Serial.peek() != '\n') {
      inString += (char)Serial.read();
    } else {
      Serial.read();
      Serial.print("Instruction received: ");
      Serial.println(inString);
      switch(inString.toInt()) {
        case 1:
          writeServo(0, 45);
          writeServo(1, 90);
          writeServo(2, 180);
          break;
        case 2:
          writeServo(4, 145);
          writeServo(5, 90);
          writeServo(6, 0);
          break;
        case 3:
          writeServo(8, 45);
          writeServo(9, 90);
          writeServo(10, 180);
          break;
        case 4:
          writeServo(12, 135);
          writeServo(13, 90);
          writeServo(14, 0);
          break;
        case 0:
          pwm.setPWM(0, 0, 0);
          pwm.setPWM(1, 0, 0);
          pwm.setPWM(2, 0, 0);
          pwm.setPWM(4, 0, 0);
          pwm.setPWM(5, 0, 0);
          pwm.setPWM(6, 0, 0);
          pwm.setPWM(8, 0, 0);
          pwm.setPWM(9, 0, 0);
          pwm.setPWM(10, 0, 0);
          pwm.setPWM(12, 0, 0);
          pwm.setPWM(13, 0, 0);
          pwm.setPWM(14, 0, 0);
          break;
      }
      inString = "";
    }
  }
}
