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

  writeServo(0, 45);
  writeServo(1, 100);
  writeServo(2, 180);
  writeServo(3, 135);
  writeServo(4, 90);
  writeServo(5, 0);
}

void loop() {
  

}
