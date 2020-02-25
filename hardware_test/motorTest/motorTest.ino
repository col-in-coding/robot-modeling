#include <Adafruit_PWMServoDriver.h>

#define SERVOMIN 110
#define SERVOMAX 512

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int convert2pulse(int ang) {
  int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);
  return pulse;
}

void setup() {

  Serial.begin(9600);
  Serial.println("Servo test! please input angle: ");

  pwm.begin();
  pwm.setPWMFreq(50);
  delay(50);
}

void loop() {
  String inString = "";
  while (Serial.available() > 0) {
    char inChar = Serial.read();
    if (inChar != "\n") {
      inString += (char)inChar;
    }
    delay(10);
  }
  if (inString != "") {
      Serial.print("I received: ");
      Serial.println(inString);
      pwm.setPWM(0, 0, convert2pulse(inString.toInt()));
  }
}
