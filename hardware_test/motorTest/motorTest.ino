
#include <Wire.h>
#include "Adafruit_PWMServoDriver.h"

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// our servo # counter
uint8_t servonum = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");
  pwm.begin();
  pwm.setPWMFreq(50);  // Analog servos run at ~60 Hz updates
}

// you can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. its not precise!
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
String inString = "";

void loop() {  
  if (Serial.available() > 0) {
    if (Serial.peek() != '\n') {
      inString += (char)Serial.read();
    } else {
      Serial.read();
      Serial.print("Instruction received: ");
      Serial.println(inString);
      writeServo(servonum,inString.toInt());
      inString = "";
    }
  }
}
