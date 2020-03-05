#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
const uint8_t SERVO_PWM_FREQ {50};    // 50 Hz

void setup() {
  Serial.begin(9600);
  Serial.println("Robot Start!");
  Serial.println("Please select from the menu: ");
  Serial.println("1. Robot stand");
  Serial.println("#############################");
  
  pwm.begin();
  delay(50);
  pwm.setPWMFreq(SERVO_PWM_FREQ);
  reset_robot();
}

void set_servo_pulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_PWM_FREQ;
  // Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  // Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000;
  pulse /= pulselength;
  // Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

void write_servo_ang(uint8_t n,uint8_t angle){
  double pulse;
  pulse=0.5+angle/90.0;
  set_servo_pulse(n,pulse);
}


/***
 * Params:
 *   alpha: Rate of convergence
 *   mu: control the amplitude of the output signals, Ah = sqrt(mu)
 *   Ah: Amplitude of the hip joint control signal
 *   Ak: Amplitude of the knee joint control signal
 *   beta: Duty cycle of the support phase (Load Factor)
 *         omega_st / omega_sw = (1 - beta) / beta
 *   omega_sw: Frequency of the swing phase
 *   omega_st: Frequency of the support phase
 *   a: rate of the change between omega_sw and omega_st
 *   u: (optional, default 0), feedback, EX: u1 <=> x1, u2<=> y2...
 * 
 * Outputs:
 *   x1 => LF
 *   x2 => RF
 *   x3 => LH
 *   x4 => RH
 * 
 * x, the output of the oscillator, is control signal for hip joint
 * the Ascending part is in the swing phase
 * the Descending part is in support phase (when the knee joint is frozen)
*/

/*
const uint16_t SERVOMIN {102};
const uint16_t SERVOMAX {512};
uint8_t alpha {100};
double Ah {0.2};
double Ak {0.1};
uint8_t a {10};
double omega_sw {3 * M_PI};

// servo init angle with model coordinate 
double servo_int_ang[12] {
  45, 100, 180, 135, 90, 0, 135, 90, 0, 45, 90, 180
};
int8_t servo_dir[12] {
  1, -1, -1, -1, 1, 1, -1, -1, 1, 1, 1, -1
};

// hip0, hip1, knee
double servo_num[12] {
  0, 1, 2,
  3, 4, 5,
  6, 7, 8,
  9, 10, 11
};
//        LF,   RF,     RH,     LF
// walk   0,    0.5,    0.25,   0.75
// trot   0,    0.5,    0,      0.5
double phi[4] {0, 0.5, 0.25, 0.75};
double beta {0.5};

// current angle rad buffer
double current_rad[12] {};
*/
const uint16_t SERVOMIN {102};
const uint16_t SERVOMAX {512};

// servo init angle with model coordinate 
double servo_int_ang[12] {
  45, 100, 180, 135, 90, 0, 135, 90, 0, 45, 90, 180
};

int8_t servo_dir[12] {
  1, -1, -1, -1, 1, 1, -1, -1, 1, 1, 1, -1
};

// hip0, hip1, knee
double servo_num[12] {
  0, 1, 2,
  3, 4, 5,
  6, 7, 8,
  9, 10, 11
};

void reset_robot(){
  pwm.setPWM(0, 0, 0);
  pwm.setPWM(1, 0, 0);
  pwm.setPWM(2, 0, 0);
  pwm.setPWM(3, 0, 0);
  pwm.setPWM(4, 0, 0);
  pwm.setPWM(5, 0, 0);
  pwm.setPWM(6, 0, 0);
}

// init STAND state
void init_robot() {
  Serial.println("init robot");

  write_servo_ang(0, 45);
  write_servo_ang(1, 100);
  write_servo_ang(2, 180);
  
  write_servo_ang(4, 135);
  write_servo_ang(5, 90);
  write_servo_ang(6, 0);

//  write_servo_ang(8, 135);
//  write_servo_ang(9, 90);
//  write_servo_ang(10, 0);
}

String input_str = "";
void loop() {
  if (Serial.available() > 0) {
    if (Serial.peek() != '\n') {
      input_str += (char)Serial.read();
    } else {
      Serial.read();
      Serial.print("Instruction received: ");
      Serial.println(input_str);
      switch (input_str.toInt()) {
        case 1:
          init_robot();
          break;
        case 2:
          reset_robot();
          break;
      }
      input_str = "";
    }
    delay(100);
  }
}
