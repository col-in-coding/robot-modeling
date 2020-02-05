#include <Servo.h>

const int SERVO_US_MIN {500};
const int SERVO_US_MAX {2500};
// set init angles as the base angles
const int INIT_ANG[3] {45, 90, 180};

Servo hip0_servo;
Servo hip1_servo;
Servo knee_servo;

int current_theta [3] {0, 0, 0};
int task[100][3]
{
  {0,16,36},
  {0,17,36},
  {0,17,36},
  {0,18,36},
  {0,19,36},
  {0,19,35},
  {0,20,35},
  {0,20,35},
  {0,21,34},
  {0,22,34},
  {0,22,33},
  {0,22,32},
  {0,23,31},
  {0,23,31},
  {0,23,30},
  {0,24,29},
  {0,24,27},
  {0,24,26},
  {0,24,25},
  {0,24,23},
  {0,24,21},
  {0,24,19},
  {0,23,17},
  {0,23,14},
  {0,22,9},
  {0,22,9},
  {0,27,22},
  {0,30,30},
  {0,33,36},
  {0,35,41},
  {0,37,46},
  {0,38,50},
  {0,39,54},
  {0,40,57},
  {0,41,60},
  {0,42,63},
  {0,42,66},
  {0,43,68},
  {0,43,70},
  {0,43,73},
  {0,43,74},
  {0,43,76},
  {0,42,78},
  {0,42,79},
  {0,41,80},
  {0,41,81},
  {0,40,82},
  {0,39,82},
  {0,38,83},
  {0,37,83},
  {0,35,83},
  {0,34,83},
  {0,33,82},
  {0,31,82},
  {0,30,81},
  {0,29,80},
  {0,27,79},
  {0,25,78},
  {0,24,76},
  {0,22,74},
  {0,21,73},
  {0,19,70},
  {0,17,68},
  {0,15,66},
  {0,14,63},
  {0,12,60},
  {0,10,57},
  {0,8,54},
  {0,6,50},
  {0,4,46},
  {0,1,41},
  {0,0,36},
  {0,-3,30},
  {0,-7,22},
  {0,-13,9},
  {0,-13,9},
  {0,-10,14},
  {0,-8,17},
  {0,-6,19},
  {0,-5,21},
  {0,-3,23},
  {0,-2,25},
  {0,0,26},
  {0,0,27},
  {0,1,29},
  {0,3,30},
  {0,4,31},
  {0,5,31},
  {0,6,32},
  {0,7,33},
  {0,8,34},
  {0,9,34},
  {0,10,35},
  {0,11,35},
  {0,12,35},
  {0,13,36},
  {0,14,36},
  {0,14,36},
  {0,15,36},
  {0,16,36},
};

/**
 * Convert Angle Radius to PWM us
 */
int convert_rad_to_pulse_us(double rad) {
  int us = SERVO_US_MIN + (SERVO_US_MAX - SERVO_US_MIN) * rad / 3.14;
  return us;
}

void servo_set_angs(int angs [3]) {
  int ang0 = INIT_ANG[0] + angs[0];
  int ang1 = INIT_ANG[1] + angs[1];
  int ang2 = INIT_ANG[2] - angs[2];
  hip0_servo.writeMicroseconds(convert_ang_to_pulse_us(ang0));
  hip1_servo.writeMicroseconds(convert_ang_to_pulse_us(ang1));
  knee_servo.writeMicroseconds(convert_ang_to_pulse_us(ang2));
  delayMicroseconds(1000);
}

/**
 * Convert Angle Degree to PWM us
 */
int convert_ang_to_pulse_us(int ang) {
  int us = SERVO_US_MIN + static_cast<double>(SERVO_US_MAX - SERVO_US_MIN) * ang / 180;
  return us;
}

void setup() {
  // put your setup code here, to run once:
  hip0_servo.attach(11);
  hip1_servo.attach(10);
  knee_servo.attach(9);

  servo_set_angs(current_theta);
  delay(3000);

  Serial.begin(9600);
  Serial.println("start:");

  // read theta
//  int theta[3] {0,0,36};
//  servo_set_angs(theta);

  // loop the task
//  for (int i = 0; i < 100; ++i) {
//    int index_start = i * 3;
//    for (int j = 0; j < 3; ++j) {
//      current_theta[j] = task[index_start + j];
//      Serial.println(current_theta[j]);
//    }
//    
//    servo_set_angs(current_theta);
//    delay(100);
//  }

}

void loop() {
  for (int i = 0; i < 100; ++i){
    for (int j = 0; j < 3; ++j) {
      current_theta[j] = task[i][j];
    }
    
    servo_set_angs(current_theta);
    delay(50);
  }
}
