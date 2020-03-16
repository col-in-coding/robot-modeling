"""quadruped_controller controller."""

from controller import Robot
import math
import numpy as np


def setAngles(motors, angles, posSensors):
  # 0  16  36   0  35  83   0 -13   9   0  22   9
  direction = [1, -1, 1, 1, -1, 1, 1, 1, -1, 1, 1, -1]
  theta = []
  for i in range(12):
    theta.append(angles[i] * direction[i] * math.pi / 180)
  setPositions(motors, theta, posSensors)


def setPositions(motors, theta, posSensors):
  """
  set joint positions
  not feedback control
  """
  delta = 0.01
  for i in range(len(motors)):
    motors[i].setPosition(theta[i])

  # while robot.step(timestep) != -1:
    # checked = True
    # for i in range(len(motors)):
      # effective = posSensors[i].getValue()
      # target = theta[i]
      
      # if math.fabs(effective - target) > delta:
        # print('i th part: ', i)
        # print('error: ', effective - target)
        # checked = False
    # if checked:
      # break


def setVelocity(motors, v):
  """
  v: rad/s
  """ 
  for i in range(len(motors)):
    motors[i].setVelocity(v)


theta_list = [
[0,16,36],
[0,17,36],
[0,17,36],
[0,18,36],
[0,19,36],
[0,19,35],
[0,20,35],
[0,20,35],
[0,21,34],
[0,22,34],
[0,22,33],
[0,22,32],
[0,23,31],
[0,23,31],
[0,23,30],
[0,24,29],
[0,24,27],
[0,24,26],
[0,24,25],
[0,24,23],
[0,24,21],
[0,24,19],
[0,23,17],
[0,23,14],
[0,22,9],
[0,22,9],
[0,27,22],
[0,30,30],
[0,33,36],
[0,35,41],
[0,37,46],
[0,38,50],
[0,39,54],
[0,40,57],
[0,41,60],
[0,42,63],
[0,42,66],
[0,43,68],
[0,43,70],
[0,43,73],
[0,43,74],
[0,43,76],
[0,42,78],
[0,42,79],
[0,41,80],
[0,41,81],
[0,40,82],
[0,39,82],
[0,38,83],
[0,37,83],
[0,35,83],
[0,34,83],
[0,33,82],
[0,31,82],
[0,30,81],
[0,29,80],
[0,27,79],
[0,25,78],
[0,24,76],
[0,22,74],
[0,21,73],
[0,19,70],
[0,17,68],
[0,15,66],
[0,14,63],
[0,12,60],
[0,10,57],
[0,8,54],
[0,6,50],
[0,4,46],
[0,1,41],
[0,0,36],
[0,-3,30],
[0,-7,22],
[0,-13,9],
[0,-13,9],
[0,-10,14],
[0,-8,17],
[0,-6,19],
[0,-5,21],
[0,-3,23],
[0,-2,25],
[0,0,26],
[0,0,27],
[0,1,29],
[0,3,30],
[0,4,31],
[0,5,31],
[0,6,32],
[0,7,33],
[0,8,34],
[0,9,34],
[0,10,35],
[0,11,35],
[0,12,35],
[0,13,36],
[0,14,36],
[0,14,36],
[0,15,36],
[0,16,36]
]

theta_list_reversed = list(reversed(theta_list))

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# init 12 DOF motors and position sensors
motorNames = [
  'hip0x', 'hip0z', 'knee0',
  'hip1x', 'hip1z', 'knee1',
  'hip2x', 'hip2z', 'knee2',
  'hip3x', 'hip3z', 'knee3'
]
motors = [
  robot.getMotor(motorNames[i]) for i in range(len(motorNames))
]
posSensors = [
  robot.getPositionSensor(motorNames[i] + '_pos') for i in range(len(motorNames))
]
for posSensor in posSensors:
  posSensor.enable(timestep)

# init positions
# thetaInit = np.array([
  # 0, -45/180 * math.pi, 90/180 * math.pi,
  # 0, -45/180 * math.pi, 90/180 * math.pi,
  # 0, 45/180 * math.pi, -90/180 * math.pi,
  # 0, 45/180 * math.pi, -90/180 * math.pi
# ])

# setPositions(motors, thetaInit, posSensors)
setVelocity(motors, 10)

theta_list_len = len(theta_list)
lf_index = 0

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1: 
  rf_index = (lf_index + theta_list_len // 2) % theta_list_len
  lr_index = (lf_index - theta_list_len // 4) % theta_list_len
  rr_index = (lf_index - 3 * theta_list_len // 4) % theta_list_len
  lf_angles = np.array(theta_list[lf_index])
  rf_angles = np.array(theta_list[rf_index])
  lr_angles = np.array(theta_list_reversed[lr_index])
  rr_angles = np.array(theta_list_reversed[rr_index])
  op_angles = np.concatenate((rf_angles, lf_angles, rr_angles, lr_angles))
  setAngles(motors, op_angles, posSensors)

  lf_index = (lf_index + 1) % theta_list_len

  pass
