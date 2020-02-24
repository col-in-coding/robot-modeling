###
# Quadruped Robot gait transitions between walk and trot
# CPG controller is based on 4 HOPF Oscillators (Read cpg fold to known more)
#
# For Walk:
# beta = 0.75
# phi_LF = 0
# phi_RF = 0.5
# phi_LH = 0.75
# phi_RH = 0.25
#
# For Trot:
# beta = 0.5
# phi_LF = 0
# phi_RF = 0.5
# phi_LH = 0
# phi_RH = 0.5
#
# Then:
# Phi3 = 0.25 or 0
# Phi1 = 0, Phi2 = 0.5, Phi4 = 0.5 + Phi3
# Beta = 0.5 + Phi3
# theta21 = - pi
# theta31 = - Phi3 * 2 * pi
# theta41 = - (0.5 + Phi3) * 2 * pi
# theta12 = pi
# theta32 = (0.5 - Phi3) * 2 * pi
# theta42 = - Phi3 * 2 * pi
# theta13 = Phi3 * 2 * pi
# theta23 = (Phi3 - 0.5) * 2 * pi
# theta43 = - pi
# theta14 = (0.5 + Phi3) * 2 * pi
# theta24 = Phi3 * 2 * pi
# theta34 = pi
#
###

import matplotlib.pyplot as plt
import numpy as np

alpha = 1000
Ah = 1
Ak = 0.5
omega_sw = 5 * np.pi
a = 10
mu = Ah ** 2

phi3 = 0.25

# phase config
phi = [0, 0.5, phi3, 0.5 + phi3]
beta = 0.5 + phi3
omega_st = omega_sw * (1 - beta) / beta

# 5 seconds
sim_duration = 10
dt = 0.01
iteration = 100

x1, y1, x2, y2, x3, y3, x4, y4 = (1, 0, 0, 0, 0, 0, 0, 0)
Q1 = np.matrix([x1, y1]).T
Q2 = np.matrix([x2, y2]).T
Q3 = np.matrix([x3, y3]).T
Q4 = np.matrix([x4, y4]).T
Q = np.vstack([Q1, Q2, Q3, Q4])

# result collection
theta_h1_t = []
theta_h2_t = []
theta_h3_t = []
theta_h4_t = []
theta_k1_t = []
theta_k2_t = []
theta_k3_t = []
theta_k4_t = []

t_t = np.arange(0, sim_duration, dt)

for t in t_t:
    for _ in np.arange(iteration):
        # change the phi3 to 0 from 2s
        if t >= 4 and phi3 > 0:
            phi3 -= 0.01
            phi = [0, 0.5, phi3, 0.5 + phi3]
            beta = 0.5 + phi3
            omega_st = omega_sw * (1 - beta) / beta

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
                theta_ji = 2 * np.pi * (phi[j] - phi[i])
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

    # Signal Data Collection
    theta_h1_t.append(x1)
    theta_h2_t.append(x2)
    theta_h3_t.append(x3)
    theta_h4_t.append(x4)
    #
    # if y1 > 0:
    #     theta_k1 = 0
    # else:
    #     theta_k1 = - Ak / Ah * y1
    #
    # if y2 > 0:
    #     theta_k2 = 0
    # else:
    #     theta_k2 = - Ak / Ah * y2
    #
    # if y3 > 0:
    #     theta_k3 = 0
    # else:
    #     theta_k3 = Ak / Ah * y3
    #
    # if y4 > 0:
    #     theta_k4 = 0
    # else:
    #     theta_k4 = Ak / Ah * y4
    #
    # theta_k1_t.append(theta_k1)
    # theta_k2_t.append(theta_k2)
    # theta_k3_t.append(theta_k3)
    # theta_k4_t.append(theta_k4)

plt.figure()
plt.subplot()
plt.plot(t_t, theta_h1_t, '-', label='limb1')
plt.plot(t_t, theta_h2_t, '--', label='limb2')
plt.plot(t_t, theta_h3_t, '-.', label='limb3')
plt.plot(t_t, theta_h4_t, ':', label='limb4')
plt.ylabel('joint angle/rad')
plt.xlabel('time/s')
plt.grid()
plt.legend()
plt.show()
