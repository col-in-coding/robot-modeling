"""
e-puck_avoid_collision controller.
Author: Colin
"""


from controller import Robot, DistanceSensor, Motor

TIME_STEP = 64
MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# initialize devices
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]
for i in range(8):
    ps.append(robot.getDistanceSensor(psNames[i]))
    ps[i].enable(TIME_STEP)

# initialize motors
leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    # read sensors outputs
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
    # detect obstacles
    right_obstacle = psValues[0] > 70.0 or psValues[1] > 70.0 or psValues[2] > 70.0
    left_obstacle = psValues[5] > 70.0 or psValues[6] > 70.0 or psValues[7] > 70.0 
    # initialize motor speeds at 50% of MAX_SPEED
    leftSpeed = 0.5 * MAX_SPEED
    rightSpeed = 0.5 * MAX_SPEED
    # modify speeds according to obstacles
    if left_obstacle:
        # turn right
        leftSpeed += 0.5 * MAX_SPEED
        rightSpeed -= 0.5 * MAX_SPEED
    elif right_obstacle:
        # turn left
        leftSpeed -= 0.5 * MAX_SPEED
        rightSpeed += 0.5 * MAX_SPEED
    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

    pass

# Enter here exit cleanup code.
