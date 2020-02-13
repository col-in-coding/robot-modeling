###
# CPG controller based on 4 Hopf Oscillators
#
# Params:
#   alpha: Rate of convergence
#   mu: control the amplitude of the output signals, A = sqrt(mu)
#   beta: Duty cycle of the support phase (Load Factor)
#         omega_st / omega_sw = (1 - beta) / beta
#   omega_sw: Frequency of the swing phase
#   omega_st: Frequency of the support phase
#   a: rate of the change between omega_sw and omega_st
#   u: (optional, default 0), feedback, EX: u1 <=> x1, u2<=> y2...
#
# Outputs:
#   x1 => LF
#   x2 => RF
#   x3 => LH
#   x4 => RH
#
# x, the output of the oscillator, is control signal for hip joint
# the Ascending part is in the swing phase
# the Descending part is in support phase (when the knee joint is frozen)
###

import matplotlib.pyplot as plt
import numpy as np

alpha = 10000
beta = 0.75
mu = 1
omega_sw = 5 * np.pi
omega_st = omega_sw * (1 - beta) / beta
a = 10

# phase config of walk gait
phi_LF = 0
phi_RF = 0.5
phi_LH = 0.75
phi_RH = 0.25
# phase order
phi = [phi_LF, phi_RF, phi_LH, phi_RH]

# 5 seconds
sim_duration = 5
dt = 0.01
iteration = 100

x1, y1, x2, y2, x3, y3, x4, y4 = np.random.uniform(0, 1, 8)
Q = np.matrix([x1, y1, x2, y2, x3, y3, x4, y4]).T

# result collection
x1_t = []
y1_t = []
x2_t = []
y2_t = []
x3_t = []
y3_t = []
x4_t = []
y4_t = []
t_t = np.arange(0, sim_duration, dt)

for t in t_t:
    for _ in np.arange(iteration):
        r1_square = x1 ** 2 + y1 ** 2
        r2_square = x2 ** 2 + y2 ** 2
        r3_square = x3 ** 2 + y3 ** 2
        r4_square = x4 ** 2 + y4 ** 2

        # Frequency of the Oscillator
        omega1 = omega_st / (np.exp(- a * y1) + 1) + omega_sw / (np.exp(a * y1) + 1)
        omega2 = omega_st / (np.exp(- a * y2) + 1) + omega_sw / (np.exp(a * y2) + 1)
        omega3 = omega_st / (np.exp(- a * y3) + 1) + omega_sw / (np.exp(a * y3) + 1)
        omega4 = omega_st / (np.exp(- a * y4) + 1) + omega_sw / (np.exp(a * y4) + 1)

        FQ = np.matrix([
            alpha * (mu - r1_square) * x1 - omega1 * y1,
            alpha * (mu - r1_square) * y1 + omega1 * x1,
            alpha * (mu - r2_square) * x2 - omega2 * y2,
            alpha * (mu - r2_square) * y2 + omega2 * x2,
            alpha * (mu - r3_square) * x3 - omega3 * y3,
            alpha * (mu - r3_square) * y3 + omega3 * x3,
            alpha * (mu - r4_square) * x4 - omega4 * y4,
            alpha * (mu - r4_square) * y4 + omega4 * x4
        ]).T

        """
            Coupling Matrix of 4 Oscillators
            R = [
                R11, R21, R31, R41;
                R12, R22, R32, R42;
                R13, R23, R33, R43;
                R14, R24, R34, R44
            ]

            Rji = [
                cos(theta_ji), -sin(theta_ji);
                sin(theta_ji), cos(theta_ji)
            ]

            theta_ji = phi_j - phi_i
        """
        R = np.asmatrix(np.full((8, 8), None))
        for i in range(0, 4):
            for j in range(0, 4):
                theta_ji = 2 * np.pi * (phi[i] - phi[j])
                R[i * 2, j * 2] = np.cos(theta_ji)
                R[i * 2, j * 2 + 1] = - np.sin(theta_ji)
                R[i * 2 + 1, j * 2] = np.sin(theta_ji)
                R[i * 2 + 1, j * 2 + 1] = R[i * 2, j * 2]

        # Q_dot = F(Q) + RQ
        Q_dot = FQ + np.dot(R, Q)

        Q = Q + Q_dot * dt / iteration
        x1 = Q[0, 0]
        y1 = Q[1, 0]
        x2 = Q[2, 0]
        y2 = Q[3, 0]
        x3 = Q[4, 0]
        y3 = Q[5, 0]
        x4 = Q[6, 0]
        y4 = Q[7, 0]

    x1_t.append(x1)
    y1_t.append(y1)
    x2_t.append(x2)
    y2_t.append(y2)
    x3_t.append(x3)
    y3_t.append(y3)
    x4_t.append(x4)
    y4_t.append(y4)

plt.figure()
plt.subplot(411)
plt.plot(t_t, x1_t, label='hip')
plt.ylabel('LF')
plt.grid()

plt.subplot(412)
plt.plot(t_t, x2_t, label='hip')
plt.ylabel('RF')
plt.grid()

plt.subplot(413)
plt.plot(t_t, x3_t, label='hip')
plt.ylabel('LH')
plt.grid()

plt.subplot(414)
plt.plot(t_t, x4_t, label='hip')
plt.ylabel('RH')
plt.grid()

plt.subplots_adjust(top=0.92, bottom=0.2, hspace=0.5, wspace=0.35)
plt.legend()


plt.show()
