###
# Hopf Oscillator Model
#
# Params:
#   alpha: control the rate of convergence
#   u: control the amplitude of the output signals, A = sqrt(u)
#   w: (omega) control the frequency
###

import matplotlib.pyplot as plt
import numpy as np

x0 = np.random.random()
y0 = np.random.random()

fig, ax = plt.subplots()

alpha = 100
u = 1
w = 2 * np.pi
dt = 0.01
iteration = 100

x = x0
y = y0
xt = [x0]
yt = [y0]
for t in np.arange(0, 5, dt):
    for i in np.arange(iteration):
        r_square = x ** 2 + y ** 2
        x_dot = alpha * (u - r_square) * x - w * y
        y_dot = alpha * (u - r_square) * y + w * x
        x += x_dot * dt / iteration
        y += y_dot * dt / iteration
    xt.append(x)
    yt.append(y)
ax.plot(xt, yt)

plt.show()
