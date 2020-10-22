from matplotlib import pyplot as plt
import numpy as np


plt_eq_11_l0 = np.loadtxt('plt_eq_11_l0.txt', dtype=float)
plt_eq_11_l1 = np.loadtxt('plt_eq_11_l1.txt', dtype=float)
plt_eq_11_l2 = np.loadtxt('plt_eq_11_l2.txt', dtype=float)
plt_eq_11_l3 = np.loadtxt('plt_eq_11_l3.txt', dtype=float)
plt_eq_11_l4 = np.loadtxt('plt_eq_11_l4.txt', dtype=float)
plt_eq_11_l5 = np.loadtxt('plt_eq_11_l5.txt', dtype=float)
plt_time_l = np.loadtxt('plt_time_l.txt', dtype=float)
fig, ax = plt.subplots(2, 3)

for i in range(len(plt_eq_11_l0)):
        M = [[plt_eq_11_l0[i], plt_eq_11_l1[i], plt_eq_11_l2[i]],
             [plt_eq_11_l1[i], plt_eq_11_l3[i], plt_eq_11_l4[i]],
             [plt_eq_11_l2[i], plt_eq_11_l4[i], plt_eq_11_l5[i]]]
        M = np.linalg.inv(M)
        plt_eq_11_l0[i] = M[0, 0]
        plt_eq_11_l1[i] = M[0, 1]
        plt_eq_11_l2[i] = M[0, 2]
        plt_eq_11_l3[i] = M[1, 1]
        plt_eq_11_l4[i] = M[1, 2]
        plt_eq_11_l5[i] = M[2, 2]

FIGSIZE = 3.7
LINE_WIDTH = 2.0
FONT_SIZE = 8
FONT_WEIGHT = "normal"
# set the parameters
font = {'family' : 'normal',
        'weight' : FONT_WEIGHT,
        'size'   : FONT_SIZE}
plt.rc('font', **font)
FIGURE_SIZE = ( FIGSIZE , FIGSIZE * 9.0/16.0)


ax[0][0].set_title('$[0, 0]$'.format(0))
ax[0][0].plot(plt_time_l, np.array(plt_eq_11_l0)[:], 'o', markersize=1)
ax[0][0].axhline(0.05, 0, 1, color='green')
ax[0][0].legend()

ax[0][1].set_title('$[0, 1]$'.format(0))
ax[0][1].plot(plt_time_l, np.array(plt_eq_11_l1)[:], 'o', markersize=1)
ax[0][1].axhline(0.0, 0, 1, color='green')
ax[0][1].legend()

ax[0][2].set_title('$[0, 2]$'.format(0))
ax[0][2].plot(plt_time_l, np.array(plt_eq_11_l2)[:], 'o', markersize=1)
ax[0][2].axhline(0.037, 0, 1, color='green')
ax[0][2].legend()

ax[1][0].set_title('$[1, 1]$'.format(0))
ax[1][0].plot(plt_time_l, np.array(plt_eq_11_l3)[:], 'o', markersize=1)
ax[1][0].axhline(0.043, 0, 1, color='green')
ax[1][0].legend()

ax[1][1].set_title('$[1, 2]$'.format(0))
ax[1][1].plot(plt_time_l, np.array(plt_eq_11_l4)[:], 'o', markersize=1)
ax[1][1].axhline(0.005, 0, 1, color='green')
ax[1][1].legend()

ax[1][2].set_title('$[2, 2]$'.format(0))
ax[1][2].plot(plt_time_l, np.array(plt_eq_11_l5)[:], 'o', markersize=1)
ax[1][2].axhline(0.065, 0, 1, color='green')
ax[1][2].legend()

ax[0][0].set_ylabel("Mass [Kg]")
ax[1][0].set_ylabel("Mass [Kg]")
ax[1][0].set_xlabel("Time [s]")
ax[1][1].set_xlabel("Time [s]")
ax[1][2].set_xlabel("Time [s]")
plt.savefig("mass_matrix" + ".pdf")
plt.tight_layout()
plt.show()


M = [[45, 5, -28],
     [5, 28, -8],
     [-28, -8, 33]]
a = np.linalg.eigvals(M)

M = [[0.045, -0.005, 0.043],
     [-0.005, 0.045, -0.01],
     [0.043, -0.01, 0.09]]
print(np.linalg.inv(M))
print(a)