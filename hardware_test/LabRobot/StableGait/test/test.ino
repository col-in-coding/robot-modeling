#include "Actuator.h"

Actuator Act;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.setTimeout(10);
  while (!Serial);
  // wait for ready
  while (Serial.available() && Serial.read()); // empty buffer
  delay(1000);

  Act.init();

  Serial.print(
    "Please select from menu: \n"
    "  1. adjust\n"
    "  2. stand\n"
  );
}

void loop() {
  // put your main code here, to run repeatedly:
  String serial_in_str {""};
  while(Serial.available() > 0) {
    char inChar = Serial.read();
    serial_in_str += (char)inChar;
    // wait for input buffer read
    delay(10);
  }
  if(serial_in_str != "")
  {
    Serial.print("Command received: ");
    Serial.println(serial_in_str);
    int command_code = serial_in_str.toInt();

    Act.adjust();
        
    
  }
}
