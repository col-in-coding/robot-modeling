#include <Servo.h>

// our servo # counter
uint8_t servonum = 9;
Servo s;

void setup() {
  Serial.begin(9600);
  Serial.println("Lab robot servo test!");
  s.attach(servonum, 500, 2500);
}

void writeServo(uint8_t angle) {
  double pulse;
  pulse = (0.5 + angle/90.0) * 1000;
  Serial.print("pulse:");
  Serial.println(pulse);
  s.writeMicroseconds(pulse);
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
      writeServo(inString.toInt());
      inString = "";
    }
  }
}
