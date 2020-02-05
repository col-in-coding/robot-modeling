import pandas as pd
import numpy as np

targetPoints = pd.read_csv('./results/targetPoints').to_numpy()
link1 = 11
link2 = 9

initAng = []

theta = []


def rad_to_ang(rad):
    return int(rad * 180 / np.pi)


# calculate the position of knee joint and R of end effect
for tx, ty in targetPoints:
    d = np.sqrt(tx ** 2 + ty ** 2)
    v = (link1 ** 2 + d ** 2 - link2 ** 2) / (2 * link1 * d)

    if v > 1:
        print('theta error')
        hTheta1 = 0
    else:
        hTheta1 = np.arccos((link1 ** 2 + d ** 2 - link2 ** 2) / (2 * link1 * d))

    theta3 = np.arccos((link2 ** 2 + d ** 2 - link1 ** 2) / (2 * link2 * d))
    hTheta2 = np.arctan(tx / ty)
    # preset theta0
    theta0 = 0
    theta1 = rad_to_ang(hTheta1 + hTheta2)
    theta2 = rad_to_ang(hTheta1 + theta3)

    theta.append([theta0, theta1, theta2])

theta = np.array(theta)
theta1 = theta[..., 1]

output = pd.DataFrame({
    'theta0': theta[..., 0],
    'theta1': theta[..., 1],
    'theta2': theta[..., 2]
})
output.to_csv('./results/thetaList', index=False)
