#include <Servo.h>

Servo myservo;
int received = 0;

void setup() {
  
  // put your setup code here, to run once:
  myservo.attach(11);
  myservo.writeMicroseconds(1000);
  delayMicroseconds(1000);

  Serial.begin(9600);
  Serial.print("Enter microceconds: ");
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    // read the incoming byte:
    received = Serial.parseInt();
    if (received > 0) {
      // say what you got:
      Serial.println("I received: ");
      Serial.println(received);
      myservo.writeMicroseconds(received);
      delay(1000);
    }
  }
}
