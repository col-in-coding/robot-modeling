#include <Servo.h>

Servo myservo;
int received {0};

void setup() {
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);

  myservo.attach(2);
  // 500 - 2500
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
