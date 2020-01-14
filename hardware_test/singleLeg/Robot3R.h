/********************************************************************
 * Author: Haoxian Li (Colin)
 * 3R Robot
 * Header file
 *******************************************************************/

#include <deque>
#include <Servo.h>

#define PIN_HIP0 11
#define PIN_HIP1 10
#define PIN_HIP2 9

class Robot3R
{
private:
  int initAngles[3] = [0, 0, 0]; // to change the default joint angles
  int angleLimits[3]; // max angle value from 0 (default as 0)
  deque<int> taskQ; // task queue to be run
  bool isInf = false; // if run taskQ loop circle

public:
  Robot3R(int initAngles[3], int angleLimits[3]); // constructor
  ~Robot3R(); // distructor

  /**
   * assign a single line task
   */
  void setJointStatus(int angles[]);

  /**
   * assign a new task sequence
   */
  void setJointStatusSequence(int** anglesList);


  /**
   * actuation
   */
  void run();
};