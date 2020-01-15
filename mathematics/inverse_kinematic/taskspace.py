import matplotlib.pyplot as plt
import numpy as np

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

x, y = b.transpose()
ax.scatter(x, y, alpha=0.2)


# route design (manually)
# duty circle: 0.5
# form: hemisphere
# position: right under the origin with high h
# phases: 4
fh = 0.5
phases = 4
# step length
deltaS = 10
# counting stops
routeSteps = 100
phaseSteps = routeSteps // phases

# high of the robot in ready state (cm)
h = link1 + link2 - fh

# phase 1
rx1, ry1 = (ox, -h)
rx = np.linspace(rx1, ox - deltaS // 2, phaseSteps)
ry = np.repeat(ry1, phaseSteps)
# phase 2 an 3
rx2, ry2 = (ox - deltaS // 2, -h)
rx3, ry3 = (ox, oy - h + deltaS // 2)
for theta in np.linspace(0., np.pi, phaseSteps * 2):
    rx = np.append(rx, [ox - deltaS // 2 * np.cos(theta)])
    ry = np.append(ry, [oy - h + deltaS // 2 * np.sin(theta)])
# phase 4
rx4, ry4 = (ox + deltaS // 2, -h)

rx = np.append(rx, np.linspace(ox + deltaS // 2, ox, phaseSteps))
ry = np.append(ry, np.repeat(-h, phaseSteps))

plt.plot(rx, ry, 'y')


# plot annotations
ax.scatter([rx1, rx2, rx3, rx4], [ry1, ry2, ry3, ry4], alpha=1)
ax.scatter(ox, oy, alpha=1)

ax.annotate('O', (ox, oy + 0.5))
ax.annotate('A', (rx1, ry1 + 0.5))
ax.annotate('B', (rx2 - 0.8, ry2 + 0.5))
ax.annotate('C', (rx3, ry3 + 0.5))
ax.annotate('D', (rx4, ry4 + 0.5))


plt.show()
