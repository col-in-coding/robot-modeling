"""init_testing controller."""

from controller import Robot
import math
import numpy as np


def setPositions(motors, theta, posSensors):
  """
  set joint positions
  not feedback control
  """
  delta = 0.01
  for i in range(len(motors)):
    motors[i].setPosition(theta[i])

  while robot.step(timestep) != -1:
    checked = True
    for i in range(len(motors)):
      effective = posSensors[i].getValue()
      target = theta[i]
      
      if math.fabs(effective - target) > delta:
        # print('i th part: ', i)
        # print('error: ', effective - target)
        checked = False
    if checked:
      break


def setVelocity(motors, v):
  """
  v: rad/s
  """ 
  for i in range(len(motors)):
    motors[i].setVelocity(v)


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
thetaInit = np.array([
  0, -45/180 * math.pi, 90/180 * math.pi,
  0, -45/180 * math.pi, 90/180 * math.pi,
  0, 45/180 * math.pi, -90/180 * math.pi,
  0, 45/180 * math.pi, -90/180 * math.pi
])
setPositions(motors, thetaInit, posSensors)
setVelocity(motors, 10)

isLeftStepUp = True
leftStepUp = [
  0, -45/180 * math.pi, 90/180 * math.pi,
  0, -55/180 * math.pi, 110/180 * math.pi,
  0, 55/180 * math.pi, -110/180 * math.pi,
  0, 45/180 * math.pi, -90/180 * math.pi
]
rightStepUp = [
  0, -55/180 * math.pi, 110/180 * math.pi,
  0, -45/180 * math.pi, 90/180 * math.pi,
  0, 45/180 * math.pi, -90/180 * math.pi,
  0, 55/180 * math.pi, -110/180 * math.pi
]


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
  if isLeftStepUp == True:
    setPositions(motors, leftStepUp, posSensors)
  else:
    setPositions(motors, rightStepUp, posSensors)
  isLeftStepUp = not isLeftStepUp
  

  pass

# Enter here exit cleanup code.




