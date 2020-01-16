import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from sympy import *


def validate(l1, l2, tri_x, tri_y):
    if tri_x ** 2 + tri_y ** 2 >= (l1 + l2) ** 2:
        print('out of the workspace!')
    if tri_x ** 2 + tri_y ** 2 <= (l1 - l2) ** 2:
        print('out of the workspace!')


def find_circle_origin(p1, p2, p3):
    x, y = symbols('x y')
    p1x, p1y = p1
    p2x, p2y = p2
    p3x, p3y = p3
    eq1 = Eq((x - p1x) ** 2 + (y - p1y) ** 2, (x - p2x) ** 2 + (y - p2y) ** 2)
    eq2 = Eq((x - p1x) ** 2 + (y - p1y) ** 2, (x - p3x) ** 2 + (y - p3y) ** 2)
    res = linsolve((eq1, eq2), (x, y))
    return res.args[0]


# length of links (cm)
link1 = 11
link2 = 9
# angle range limits
theta1 = np.arange(0., np.pi / 2, 0.01)
theta2 = np.arange(0., np.pi, 0.01)

# point O, Hip1
o = (0, 0)
ox, oy = o

a = []
b = []

steps = 100
for theta1 in np.linspace(np.pi / 4, np.pi * 3 / 4, steps):
    for theta2 in np.linspace(0., np.pi, steps):
        # point A, Knee
        ax = ox - link1 * np.cos(theta1)
        ay = oy - link1 * np.sin(theta1)
        a.append([ax, ay])
        # point B, End effect
        bx = ax + link2 * np.cos(theta1 + theta2)
        by = ay + link2 * np.sin(theta1 + theta2)
        b.append([bx, by])

# transfer to matrix
a = np.array(a)
b = np.array(b)

fig, ax = plt.subplots()
plt.title('End Effect Task Space')

wx, wy = b.transpose()
ax.scatter(wx, wy, alpha=0.2)


# route design (manually)
# duty circle: 0.5
# form: hemisphere
# position: right under the origin with high h
# phases: 4
fh = 1
phases = 4
# step length and height
deltaS = 12
deltaH = 4
# counting stops
routeSteps = 100
phaseSteps = routeSteps // phases

# high of the robot in ready state (cm)
h = link1 + link2 - fh

# phase 1, straight line
rx1, ry1 = (ox, -h)
# phase 2 an 3, curve
rx2, ry2 = (ox - deltaS // 2, -h)
rx3, ry3 = (ox, oy - h + deltaH)
# phase 4, straight line
rx4, ry4 = (ox + deltaS // 2, -h)

rx = np.linspace(rx1, ox - deltaS // 2, phaseSteps)
ry = np.repeat(ry1, phaseSteps)

cx, cy = find_circle_origin((rx2, ry2), (rx3, ry3), (rx4, ry4))
print('cx, cy: ', cx, cy)
angHalf = np.pi - 2 * np.arctan(deltaS // 2 / deltaH)
R = sqrt((cx - rx2) ** 2 + (cy - ry2) ** 2)
for theta in np.linspace(np.pi / 2 - angHalf, np.pi / 2 + angHalf, phaseSteps * 2):
    rxNew = cx + R * np.cos(theta)
    ryNew = cy + R * np.sin(theta)
    validate(link1, link2, rxNew - ox, ryNew - oy)
    rx = np.append(rx, [rxNew])
    ry = np.append(ry, [ryNew])

rx = np.append(rx, np.linspace(ox + deltaS // 2, ox, phaseSteps))
ry = np.append(ry, np.repeat(-h, phaseSteps))

plt.plot(rx, ry, 'y')
targets = pd.DataFrame({'X': rx, 'Y': ry})
targets.to_csv('./results/targetPoints', index=False)

# plot annotations
ax.scatter([rx1, rx2, rx3, rx4], [ry1, ry2, ry3, ry4], alpha=1)
ax.scatter(ox, oy, alpha=1)

ax.annotate('O', (ox, oy + 0.5))
ax.annotate('A', (rx1, ry1 + 0.5))
ax.annotate('B', (rx2 - 0.8, ry2 + 0.5))
ax.annotate('C', (rx3, ry3 + 0.5))
ax.annotate('D', (rx4, ry4 + 0.5))

plt.show()




