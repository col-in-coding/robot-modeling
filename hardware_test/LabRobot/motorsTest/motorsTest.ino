#include <Servo.h>

Servo s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11, s12;

double ang2pulse(uint8_t angle) {
  double pulse;
  pulse = (0.5 + angle/90.0) * 1000;
  return pulse;
}

void setup_servos() {
  Serial.println("Setup Servos");
  //  limb1
  s1.attach(3, 500, 2500);
  s2.attach(4, 500, 2500);
  s3.attach(5, 500, 2500);
  // limb2
  s4.attach(6, 500, 2500);
  s5.attach(7, 500, 2500);
  s6.attach(8, 500, 2500);
  // limb3
  s7.attach(9, 500, 2500);
  s8.attach(10, 500, 2500);
  s9.attach(11, 500, 2500);
  // limb4
  s10.attach(12, 500, 2500);
  s11.attach(13, 500, 2500);
  s12.attach(14, 500, 2500);
}

void shut_servos() {
  Serial.println("Shut Servos");
  s1.detach();
  s2.detach();
  s3.detach();
  s4.detach();
  s5.detach();
  s6.detach();
  s7.detach();
  s8.detach();
  s9.detach();
  s10.detach();
  s11.detach();
  s12.detach();
}

void run_servos() {
  Serial.println("Run Servos");
  s1.writeMicroseconds(ang2pulse(45));
  s2.writeMicroseconds(ang2pulse(90));
  s3.writeMicroseconds(ang2pulse(180));

  s4.writeMicroseconds(ang2pulse(130));
  s5.writeMicroseconds(ang2pulse(83));
  s6.writeMicroseconds(ang2pulse(0));
  
  s7.writeMicroseconds(ang2pulse(45));
  s8.writeMicroseconds(ang2pulse(90));
  s9.writeMicroseconds(ang2pulse(180));
  
  s10.writeMicroseconds(ang2pulse(130));
  s11.writeMicroseconds(ang2pulse(83));
  s12.writeMicroseconds(ang2pulse(0));



//  s1.writeMicroseconds(ang2pulse(45));
//  s2.writeMicroseconds(ang2pulse(90));
//  s3.writeMicroseconds(ang2pulse(180));

  delay(1000);
}

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(10);
  while (!Serial);
  // wait for ready
  while (Serial.available() && Serial.read()); // empty buffer

  Serial.println("Lab robot servo test!");
  Serial.println("Please select code:");
  Serial.println("1. Setup Servo");
  Serial.println("2. Run");
  Serial.println("3. Shut Servo");
}

String inString = "";
void loop() {
  if (Serial.available() > 0) {
    if (Serial.peek() != '\n') {
      inString += (char)Serial.read();
    } else {
      Serial.read();
      Serial.print("Instruction received: ");
      
      int code = inString.toInt();
      Serial.println(code);
      switch(code) {
        case 1:
          setup_servos();
          break;
        case 2:
          run_servos();
          break;
        case 3:
          shut_servos();
          break;
      }
      inString = "";
    }
  }

}
