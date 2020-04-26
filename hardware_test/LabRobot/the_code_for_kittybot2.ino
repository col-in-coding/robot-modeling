/* An IR controlled version of the KITtyBot 2.
    It uses Arduino Pro Mini and the PCB board designed by me (Fritzing sketch Kittybotmini.fzz)
    It is based on the previous robots KITtyBot and KITtyBot mini using an IR remote to control the robot
    It uses a NEC (Adafruit) remote and the IRLib2 libraries, see https://github.com/cyborg5/IRLib2.
    Download IRLib2 libraries from the repository and install them according to the instructions.
    The general dimensions are similar to the original KITtyBot but there is a displacment
    between the gamma and alfa axis of 12 mm (the servos mounted on top of each other)
    I have conitiously tweeked the gaits for walking and turning but I so far feel this has given the most stable behaviour.
    Created by Staffan Ek 2017
*/

#include <IRLibRecv.h>
#include <Servo.h>
#include <IRLibDecodeBase.h> // First include the decode base
#include <IRLib_P01_NEC.h>  // Include only the protocol you are using

#define MY_PROTOCOL NEC //Defines the IR control (NEC)

long Previous;

IRrecv My_Receiver(A1);//Receive on pin A0
IRdecodeNEC My_Decoder;

const int servonum = 12; // The amount of servos

Servo servo[servonum]; // Create servo object
const float servodeg0[12] = {90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90};
// Neutral positions for the servos adjusted from nominal 90 degrees (a calibration is needed to adjust these values)
float servodegnew[servonum]; // The desired servo position in degrees
float servodegold[servonum]; // The old (or current) servo position
// Update values below to the KITtyBot mini
const int servodir[12] = {  +1, +1, -1, -1, -1, +1, -1, -1, -1, +1, +1, +1}; // Turning direction (positive is servo counter-clockwise)
const float pi = 3.1416;
const float alfa0 = pi / 6; // The neutral position of alfa (30 deg)
const float beta0 = pi / 3; // The neutral position of beta (60 deg)
const float jointlength = 50; // The length of a leg part (both have the same length)
const float width = 120; // The width (distance between feet in y direction, with toeout0 added)
const float leng = 120; // The length (disatnce between feet in x direction)
const float distag = 12; // Distance between alfa and gamma axis
const float toeout0 = 20; // The outward distance of feet from the gamma servo centre (the distance the foot is pointed outwards)
const float leglength0 = 2 * jointlength * cos(alfa0);
const float gamma0 = asin(toeout0 / (leglength0 + distag)); // The neutral position of gamma (due to toe out 20 mm and distag 12 mm)
const float bodyradius = sqrt(pow((width / 2), 2) + pow((leng / 2), 2)); // The length of diagonal (distance from centre to foot corner)
const float phi0 = atan(width / leng); // The angle of bodyradius vs the x (forward pointing) axis
const float height0 = sqrt(pow(leglength0 + distag, 2) - pow(toeout0, 2)); // The normal height of robot (if any angles or distances are changed this must be updated)
float leglength [4] = {sqrt(pow(height0, 2) + pow(toeout0, 2)), sqrt(pow(height0, 2) + pow(toeout0, 2)),
                       sqrt(pow(height0, 2) + pow(toeout0, 2)), sqrt(pow(height0, 2) + pow(toeout0, 2))
                      };
// Start values of leglength
unsigned long timestep = 500; // Time taken by each sequence (when using servomove())
int steplength = 40; //The length of a step in x direction during walking (forward and reverse creep)
float phi = 20; // turnangle during turning (in degrees, not radians!)
// Variable for movement
float footpos[12]; // Foot positions, order LeftFrontxyz, LeftRearxyz, RightFrontxyz, RightRearxyz
float stepturn[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Foot movement in case of a turn
// The foot positions are calibrated with their respective start positions
const float jointangle0[12] = {alfa0, beta0, 0, alfa0, beta0, 0, alfa0, beta0, 0, alfa0, beta0, 0};
float jointangle[12]; //Using a vector for angles, order LeftFrontAlfaBetaGamma etc

const int voltagepin = A0; // The assigned pin for voltage meassure
int lowvolt = 0; // A variable that stops the robot if the voltage goew <7.0 V

int mode = 0; // The current ordered walking mode; forward, reverse, left, right


void setup() {
  Serial.begin(9600);
  Serial.println("KITtyBot mini"); //These lines are just to check the configuration. Can be deleted.
  Serial.print("Gamma0: ");
  Serial.println(gamma0);
  Serial.print("Leglength0: ");
  Serial.println(leglength0);
  Serial.print("Bodyradius: ");
  Serial.println(bodyradius);
  Serial.print("Phi0: ");
  Serial.println(phi0);
  Serial.print("Height0: ");
  Serial.println(height0);

  servo[0].attach(3);
  servo[1].attach(4);
  servo[2].attach(5);
  servo[3].attach(6);
  servo[4].attach(7);
  servo[5].attach(8);
  servo[6].attach(2);
  servo[7].attach(A3);
  servo[8].attach(12);
  servo[9].attach(11);
  servo[10].attach(10);
  servo[11].attach(9);

  for (int i = 0; i < servonum; i++) { // Centre all values and the output to the serovs
    servodegnew[i] = servodeg0[i];
    servodegold[i] = servodegnew[i];
    servo[i].write(servodegnew[i]);
  }
  delay(5000);
  My_Receiver.enableIRIn(); // Start the receiver

}

void loop() {
  voltmeasure(); // Check voltage at least here
  while (lowvolt == 0) { // Proceed only if there is enough power
    bodyxyz(0, 0, 0); // Just make sure everything is centered
    servomove();
    mode = 0;
    IRread();
    Serial.print("Mode: "); // Only for monitoring in the serial console, can be deleted
    Serial.println(mode);
    switch (mode) {
      case 1: forwardcreep(); break;
      case 2: reversecreep(); break;
      case 3: leftturn(); break;
      case 4: rightturn(); break;
    }
    voltmeasure();
    delay(500);
  }

  if (lowvolt == 1) { // Got to "rest". A clear signal that battery needs charging
    bodyxyz (0, 0, -30); // Lower body, a clear signal that it has gone to rest
    servomove();
  }
}

// Below are the functions called in correct order in order to calculate new angles
void lengthangles() {
  // Front left foot
  jointangle[2] = gammaleft(footpos[1], footpos[2]);
  leglength[0] = legleft(footpos[0], footpos[2], jointangle[2]);
  jointangle[1] = beta(leglength[0]);
  jointangle[0] = alfafront(footpos[0], jointangle[1], leglength[0]);
  // Rear left foot
  jointangle[5] = gammaleft(footpos[4], footpos[5]);
  leglength[1] = legleft(footpos[3], footpos[5], jointangle[5]);
  jointangle[4] = beta(leglength[1]);
  jointangle[3] = alfarear(footpos[3], jointangle[4], leglength[1]);
  // Front rigth foot
  jointangle[8] = gammaright(footpos[7], footpos[8]);
  leglength[2] = legright(footpos[6], footpos[8], jointangle[8]);
  jointangle[7] = beta(leglength[2]);
  jointangle[6] = alfafront(footpos[6], jointangle[7], leglength[2]);
  // Rear right foot
  jointangle[11] = gammaright(footpos[10], footpos[11]);
  leglength[3] = legright(footpos[9], footpos[11], jointangle[11]);
  jointangle[10] = beta(leglength[3]);
  jointangle[9] = alfarear(footpos[9], jointangle[10], leglength[3]);
}

// Functions used to calculate IK

// Gamma, the hip servo "on top"
float gammaleft (float dy, float dz) {
  float gresult = atan((toeout0 + dy) / (height0 - dz)) - gamma0;
  return gresult;
}

float gammaright(float dy, float dz) {
  float gresult = gamma0 - atan((toeout0 - dy) / (height0 - dz));
  return gresult;
}

//Calculating leg length (distance alfa axis to toe)
float legleft(float dx, float dz, float gamma) {
  float lresult = sqrt(pow(leglength0 - (dz / cos(gamma0 + gamma)), 2) + pow(dx, 2));
  if (lresult > 2 * jointlength) lresult = 2 * jointlength; // If leglength is higher than possible some following functions become unstable
  return lresult;
}

float legright(float dx, float dz, float gamma) {
  float lresult = sqrt(pow(leglength0 - (dz / cos(gamma0 - gamma)), 2) + pow(dx, 2));
  if (lresult > 2 * jointlength) lresult = 2 * jointlength; // If leglength is higher than possible some following functions become unstable
  return lresult;
}

// Beta, the "knee joint"
float beta(float leg) {
  float bresult = 2 * acos(leg / (2 * jointlength));
  return bresult;
}

// Alfa, The other hip servo
float alfafront(float dx, float beta, float leg) {
  float aresult = (beta / 2) - asin(dx / leg);
  return aresult;
}

float alfarear(float dx, float beta, float leg) {
  float aresult = (beta / 2) + asin(dx / leg);
  return aresult;
}

// Giving foot positions based on a turning angle f (in degrees). Stepturn is the used to make footpos values
void turnpos(float f) {
  stepturn[0] = bodyradius * cos(phi0 + (f * pi / 180)) - leng / 2;
  stepturn[1] = bodyradius * sin(phi0 + (f * pi / 180)) - width / 2;
  stepturn[3] = bodyradius * cos(pi - phi0 + (f * pi / 180)) + leng / 2;
  stepturn[4] = bodyradius * sin(pi - phi0 + (f * pi / 180)) - width / 2;
  stepturn[6] = bodyradius * cos(2 * pi - phi0 + (f * pi / 180)) - leng / 2;
  stepturn[7] = bodyradius * sin(2 * pi - phi0 + (f * pi / 180)) + width / 2;
  stepturn[9] = bodyradius * cos(pi + phi0 + (f * pi / 180)) + leng / 2;
  stepturn[10] = bodyradius * sin(pi + phi0 + (f * pi / 180)) + width / 2;
}

// Calculates servo positions (in degrees) based on joint angles in the fucntion above
void servopos() {
  for (int i = 0; i < 12; i++) {
    servodegnew[i] = servodeg0[i] + servodir[i] * (jointangle[i] - jointangle0[i]) * 180 / pi;
  }
}

// The servo algorithm for controlled and syncronized movements. All servos should reach their end position at the end of a timestep
void servomove() {
  int servotimeold[servonum]; // Local variable for time of last servo position
  int servotimenew[servonum]; // Local variable for the current time when the servo i positioned
  int SERVOPULSE[servonum]; // Local variable to write to the servo driver
  float servodeg[servonum]; // Local variable for the current servo position
  float servodegspeed[servonum]; // Local variable for the desired servo speed degress per millisecond
  unsigned long starttime = millis(); // Time stamp the start of the algorithm
  unsigned long timenow = starttime; // Resetting time now
  for (int i = 0; i < servonum; i++) {
    servodegspeed[i] = (servodegnew[i] - servodegold[i]) / timestep; // Calculate the desired servo speed
    servodeg[i] = servodegold[i]; // Resetting the servo position
    servotimeold[i] = starttime; // Resetting the time
  }
  while ((timenow - starttime) < timestep) { // Loop continues until the time step is fulfilled
    for (int i = 0; i < servonum; i++) { // Iterate through each servo
      servotimenew[i] = millis(); // Get a time stamp
      servodeg[i] += servodegspeed[i] * (servotimenew[i] - servotimeold[i]);
      // Calculate a new position based on the desired speed and elapsed time
      servo[i].write(servodeg[i]); // Position servo
      servotimeold[i] = servotimenew[i]; // Resetting the old servo time for the next iteration
    }
    timenow = millis();
    // Get a time stamp after all servos has been iterated to use in the while case.
  }
  for (int i = 0; i < servonum; i++) { // Make on last iteration to assure that the servos reached their end positions
    servo[i].write(servodegnew[i]); // Position servo
    servodegold[i] = servodegnew[i]; // Resetting the current position for future iterations
  }
}

// A servomove without timing, use when no synchronous moving is needed, i.e. lifting/moving one leg
void servomovefast() {
  for (int i = 0; i < servonum; i++) { // Make on last iteration to assure that the servos reached their end positions
    servo[i].write(servodegnew[i]); // Position servo
    servodegold[i] = servodegnew[i]; // Resetting the current position for future iterations
  }
  delay(100); // Just give a reasonable time for servos to reach endpoint before moving on.
}

// Calculates a foot position (xyz coordiantes)
void footxyz(int i, float x, float y, float z) {
  footpos[3 * i] = x;
  footpos[3 * i + 1] = y;
  footpos[3 * i + 2] = z;
  lengthangles();
  servopos();
}

// Calculates foot movement, adding desired value to current position
void footmovexyz(int i, float x, float y, float z) {
  footpos[3 * i] += x;
  footpos[3 * i + 1] += y;
  footpos[3 * i + 2] += z;
  lengthangles();
  servopos();
}

// Calculates body positioning according to xyz coordinates.
void bodyxyz(float x, float y, float z ) {
  //Note: Moving body means moving the feet in the other direction, hence minus signs in all foot positions
  for (int i = 0; i < 4; i++) {
    footpos[3 * i] = -x;
    footpos[3 * i + 1] = -y;
    footpos[3 * i + 2] = -z;
  }
  lengthangles();
  servopos();
}

// Calculates body movement, adding cooridinate to existing position.
void bodymovexyz(float x, float y, float z ) {
  //Note: Body move mean moving the feet in the other direction, hence minus signs in all foot positions
  for (int i = 0; i < 4; i++) {
    footpos[3 * i] -= x;
    footpos[3 * i + 1] -= y;
    footpos[3 * i + 2] -= z;
  }
  lengthangles();
  servopos();
}

// Calculates a twist on the body the desired angle phi
void bodytwist(float f) {
  // Again here the movement is in minus driection from the foot positions
  turnpos(-f);
  for (int i = 0; i < 12; i++) {
    footpos[i] += stepturn[i];
  }
  lengthangles();
  servopos();
}

// Does a footmovement; lifts move xy and puts down foot
void footstep (int i, float x, float y) {
  footmovexyz(i, 0, 0, 30);
  servomovefast();
  footmovexyz(i, x, y, 0);
  servomovefast();
  footmovexyz(i, 0, 0, -30);
  servomovefast();
}

// Does a footmovement based on the disired turning angle, moves the foot along the turning arc
void (footstepangle(int i, float f)) {
  turnpos(f);
  footmovexyz(i, 0, 0, 30);
  servomovefast();
  footmovexyz(i, stepturn[3 * i], stepturn [3 * i + 1], 0);
  servomovefast();
  footmovexyz(i, 0, 0, -30);
  servomovefast();
}

// Checks voltage, in case of low battery lowvolt variable changes
void voltmeasure() {
  /* Note: The 7.6 V battery is conneced via a 2.2k resistance from BAT to voltagepin and 1.0k to GND
    This gives the 5 V analog input a 16 volt measure range*/
  float voltsig = analogRead(voltagepin);
  float voltage = voltsig * 16 / 1023.0;
  Serial.print("Battery: ");
  Serial.println(voltage);
  if (voltage < 7.0) {
    lowvolt = 1;
  }
  else {
    lowvolt = 0;
  }
}

// The IR read function, based on Adafruits example sketch
void IRread() {
  if (My_Receiver.getResults()) {
    Serial.print("Recieving ");
    Serial.println (My_Decoder.value);
    if (My_Decoder.decode()) {
      if (My_Decoder.value == 0xFFFFFFFF) { // Detects if the button is still pressed and keeps the value
        My_Decoder.value = Previous;
      }
      switch (My_Decoder.value) { //Detects if an arrow button is pressed and sets mode parameter
        case 0xfda05f:   mode = 1; break;
        case 0xfdb04f:   mode = 2; break;
        case 0xfd10ef:   mode = 3; break;
        case 0xfd50af:   mode = 4; break;
      }
      Previous = My_Decoder.value;
    }
    My_Receiver.enableIRIn();
  }
  else {
    mode = 0;
  }
}

// A gait for forward creeping
void forwardcreep() {
  bodymovexyz(steplength / 4, -toeout0, 0); // Starts to position for forward walking, leaning to the right
  servomove();
  footstep(1, steplength / 2, 0); // Moving rear left leg one half step length
  footstep(0, steplength / 2, 0); // And the front left
  bodymovexyz(steplength / 4, 2 * toeout0, 0); // Shifting body forward and to the left (in order to move the right feet later)
  servomove();
  while (mode == 1) {
    // Here the while loop starts, repeaetd as long as fwd is ordered (mode 1)
    footstep(3, steplength, 0); // Moving rear right forward
    footstep(2, steplength, 0); // Moving front right forward
    bodymovexyz(steplength / 2, -2 * toeout0, 0); // Shifting body forward and to the right
    servomove();
    footstep(1, steplength, 0); // Moving rear left forward
    footstep(0, steplength, 0); // Moving front left forward
    bodymovexyz(steplength / 2, 2 * toeout0, 0); // Shifting body forward and to the left
    servomove();
    // The robot has the same position as before the while loop but has moved on steplength forward.
    IRread(); // If there is still a forward command (mode ==1) the sequence should be repeated
  }
  // The while loop ends and it assumes normal postion
  /* bodymovexyz(0, 10, 0);*/
  footstep(3, steplength / 2, 0); // Taking half steps to make all legs neutral
  footstep(2, steplength / 2, 0);
  bodyxyz(0, 0, 0); // Centering body
  servomove();
  // Leaving gait mode
}

// A similar gait for reverse walking (not commented as much look at forwardcreep
void reversecreep() {
  bodymovexyz(-steplength / 4, -toeout0, 0); // Starts to position for forward walking
  servomove();
  footstep(0, -steplength / 2, 0);
  footstep(1, -steplength / 2, 0);
  bodymovexyz(-steplength / 4, 2 * toeout0, 0);
  servomove();
  while (mode == 2) {
    // Here the while loop starts, repeaetd as long as reverse is ordered (mode 2)
    footstep(2, -steplength, 0);
    footstep(3, -steplength, 0);
    bodymovexyz(-steplength / 2, -2 * toeout0, 0);
    servomove();
    footstep(0, -steplength, 0);
    footstep(1, -steplength, 0);
    bodymovexyz(-steplength / 2, 2 * toeout0, 0);
    servomove();
    IRread(); // If mode == 2 the while loop continues
  }
  // The while loop ends and it assumes normal postion
  /*  bodymovexyz(0, 10, 0);*/
  footstep(2, -steplength / 2, 0);
  footstep(3, -steplength / 2, 0);
  bodyxyz(0, 0, 0);
  servomove();
  // Leaving gait mode
}

// Doing a turn to the left the desired phi angle
void leftturn() {
  while (mode == 3) {
    // While loop as long as the left button is pressed
    bodyxyz(toeout0 / 2, toeout0, 0); // Lean left before doing anything
    servomove();
    footstepangle(3, phi); // Move rear right foot into new position
    footstepangle(2, phi); // Move front right foot into new position
    footxyz(0, -toeout0 / 2 - stepturn[0], toeout0 - stepturn[1], 0);
    footxyz(1, -toeout0 / 2 - stepturn[3], toeout0 - stepturn[4], 0);
    footxyz(2, -toeout0 / 2, toeout0, 0);
    footxyz(3, -toeout0 / 2, toeout0, 0);
    // Twisting body and lean left. Written in absolute coordinates to minmize errors.
    servomove(); // Do the actual servo command
    footstepangle(0, phi); // Move front left foot
    footstepangle(1, phi); // Move rear left foot
    IRread(); // Check is left button is still pressed (mode == 3), repeat while loop
  }
  bodyxyz(0, 0, 0); // Centre body when turning is finished
  servomove();
}

//Doing a right turn. Should be identical to left turn but with different commands. Testing both at the moment.
void rightturn() {
  while (mode == 4) {
    // While loop as long as the right button is pressed
    bodyxyz(-toeout0 / 2, toeout0, 0); // Lean left before doing anything
    servomove();
    footstepangle(2, -phi); //Move front right foot
    footstepangle(3, -phi); //Move rear right foot
    footxyz(0, toeout0 / 2 - stepturn[0], toeout0 - stepturn[1], 0);
    footxyz(1, toeout0 / 2 - stepturn[3], toeout0 - stepturn[4], 0);
    footxyz(2, toeout0 / 2, toeout0, 0);
    footxyz(3, toeout0 / 2, toeout0, 0);
    // Twisting body and lean left. Written in absolute coordinates to minmize errors.
    servomove(); // Do the actual servo command
    footstepangle(1, -phi); //Move rear left foot
    footstepangle(0, -phi); //Move front left foot
    IRread(); // Check is rightt button is still pressed (mode == 4), repeat while loop
  }
  bodyxyz(0, 0, 0);
  servomove();
}
