from matplotlib import pyplot as plt
import numpy as np


plt_D_error0 = np.loadtxt('dataset2/plt_D_error0.txt', dtype=float)
plt_D_error1 = np.loadtxt('dataset2/plt_D_error1.txt', dtype=float)
plt_D_error2 = np.loadtxt('dataset2/plt_D_error2.txt', dtype=float)
plt_ND_error0 = np.loadtxt('dataset2/plt_ND_error0.txt', dtype=float)
plt_ND_error1 = np.loadtxt('dataset2/plt_ND_error1.txt', dtype=float)
plt_ND_error2 = np.loadtxt('dataset2/plt_ND_error2.txt', dtype=float)
plt_time_all_F_M = np.loadtxt('dataset2/plt_time_all_D_ND.txt', dtype=float)

plt_F0 = np.loadtxt('dataset2/plt_F0.txt', dtype=float)
plt_F1 = np.loadtxt('dataset2/plt_F1.txt', dtype=float)
plt_F2 = np.loadtxt('dataset2/plt_F2.txt', dtype=float)

plt_non_linear0 = np.loadtxt('dataset2/plt_non_linear0.txt', dtype=float)
plt_non_linear1 = np.loadtxt('dataset2/plt_non_linear1.txt', dtype=float)
plt_non_linear2 = np.loadtxt('dataset2/plt_non_linear2.txt', dtype=float)

plt_qddot_MassMatrix0 = np.loadtxt('dataset2/plt_qddot_MassMatrix0.txt', dtype=float)
plt_qddot_MassMatrix1 = np.loadtxt('dataset2/plt_qddot_MassMatrix1.txt', dtype=float)
plt_qddot_MassMatrix2 = np.loadtxt('dataset2/plt_qddot_MassMatrix2.txt', dtype=float)

plt_Error_D_M0 = np.loadtxt('dataset2/plt_Error_D_M0.txt', dtype=float)
plt_Error_D_M1 = np.loadtxt('dataset2/plt_Error_D_M1.txt', dtype=float)
plt_Error_D_M2 = np.loadtxt('dataset2/plt_Error_D_M2.txt', dtype=float)

plt_Error_ND_M0 = np.loadtxt('dataset2/plt_Error_ND_M0.txt', dtype=float)
plt_Error_ND_M1 = np.loadtxt('dataset2/plt_Error_ND_M1.txt', dtype=float)
plt_Error_ND_M2 = np.loadtxt('dataset2/plt_Error_ND_M2.txt', dtype=float)


fig, ax = plt.subplots(3, 1)
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


# ax[2].set_title('$[0, 2]$'.format(0))
plt_time_all_F_M = plt_time_all_F_M.tolist()
last_i = 0
for i in range(len(plt_time_all_F_M)):
    print(i)
    try:
        if plt_time_all_F_M[i] > plt_time_all_F_M[i + 1] or plt_time_all_F_M[i] < 0.003:
            if plt_time_all_F_M[i] > 0.003:
                if i < 300:
                    ax[0].plot(plt_time_all_F_M[last_i:i], np.array(plt_D_error0)[last_i:i], 'r', label = 'Diagonal')
                    ax[0].plot(plt_time_all_F_M[last_i:i], np.array(plt_ND_error0)[last_i:i], 'b', label = 'Non-diagonal')
                ax[0].plot(plt_time_all_F_M[last_i:i], np.array(plt_D_error0)[last_i:i], 'r')
                ax[0].plot(plt_time_all_F_M[last_i:i], np.array(plt_ND_error0)[last_i:i], 'b')
                ax[1].plot(plt_time_all_F_M[last_i:i], np.array(plt_D_error1)[last_i:i], 'r')
                ax[1].plot(plt_time_all_F_M[last_i:i], np.array(plt_ND_error1)[last_i:i], 'b')
                ax[2].plot(plt_time_all_F_M[last_i:i], np.array(plt_D_error2)[last_i:i], 'r')
                ax[2].plot(plt_time_all_F_M[last_i:i], np.array(plt_ND_error2)[last_i:i], 'b')
            last_i = i + 1
    except:
        x = 2


ax[0].legend()
ax[0].get_xaxis().set_visible(False)
ax[1].get_xaxis().set_visible(False)
ax[0].set_ylabel("$e_x [N]$")
ax[1].set_ylabel("$e_y [N]$")
ax[2].set_ylabel("$e_z [N]$")
ax[2].set_xlabel("$Time [s]$")
plt.savefig("Diagonal_Non-diagonal" + ".pdf")

fig, ax = plt.subplots(3, 1)

for i in range(len(plt_time_all_F_M)):
    print(i)
    try:
        if plt_time_all_F_M[i] > plt_time_all_F_M[i + 1] or plt_time_all_F_M[i] < 0.003:
            if plt_time_all_F_M[i] > 0.003:
                if i < 300:
                    ax[0].plot(plt_time_all_F_M[last_i:i], np.array(plt_F0)[last_i:i], 'r', label = 'Force')
                ax[0].plot(plt_time_all_F_M[last_i:i], np.array(plt_F0)[last_i:i], 'r')
                ax[1].plot(plt_time_all_F_M[last_i:i], np.array(plt_F1)[last_i:i], 'r')
                ax[2].plot(plt_time_all_F_M[last_i:i], np.array(plt_F2)[last_i:i], 'r')
            last_i = i + 1
    except:
        x = 2
ax[0].legend()
ax[0].get_xaxis().set_visible(False)
ax[1].get_xaxis().set_visible(False)

ax[0].set_ylabel("$F_x [N]$")
ax[1].set_ylabel("$F_y [N]$")
ax[2].set_ylabel("$F_z [N]$")
ax[2].set_xlabel("$Time [s]$")

fig, ax = plt.subplots(3, 1)

for i in range(len(plt_time_all_F_M)):
    print(i)
    try:
        if plt_time_all_F_M[i] > plt_time_all_F_M[i + 1] or plt_time_all_F_M[i] < 0.003:
            # if plt_time_all_F_M[i] > 0.003:
            if i < 300:
                ax[0].plot(plt_time_all_F_M[last_i:i], np.array(plt_non_linear0)[last_i:i], 'r', label = 'Nonlinear terms', linewidth=.5, markersize=1, markeredgewidth=0)
                ax[0].plot(plt_time_all_F_M[last_i:i], np.array(plt_qddot_MassMatrix0)[last_i:i], 'b', label = 'Inertia', linewidth=.5, markersize=1, markeredgewidth=0)
            ax[0].plot(plt_time_all_F_M[last_i:i], np.array(plt_non_linear0)[last_i:i], 'r', linewidth=.5, markersize=1, markeredgewidth=0)
            ax[0].plot(plt_time_all_F_M[last_i:i], np.array(plt_qddot_MassMatrix0)[last_i:i], 'b', linewidth=.5, markersize=1, markeredgewidth=0)
            ax[1].plot(plt_time_all_F_M[last_i:i], np.array(plt_non_linear1)[last_i:i], 'r', linewidth=.5, markersize=1, markeredgewidth=0)
            ax[1].plot(plt_time_all_F_M[last_i:i], np.array(plt_qddot_MassMatrix1)[last_i:i], 'b', linewidth=.5, markersize=1, markeredgewidth=0)
            ax[2].plot(plt_time_all_F_M[last_i:i], np.array(plt_non_linear2)[last_i:i], 'r', linewidth=.5, markersize=1, markeredgewidth=0)
            ax[2].plot(plt_time_all_F_M[last_i:i], np.array(plt_qddot_MassMatrix2)[last_i:i], 'b', linewidth=.5, markersize=1, markeredgewidth=0)
            last_i = i + 1
    except:
        x = 2
ax[0].legend()
ax[0].get_xaxis().set_visible(False)
ax[1].get_xaxis().set_visible(False)

ax[0].set_xlim(left=0., right=np.max(plt_time_all_F_M))
ax[1].set_xlim(left=0., right=np.max(plt_time_all_F_M))
ax[2].set_xlim(left=0., right=np.max(plt_time_all_F_M))

ax[0].set_ylabel("f$_x$ [N]")
ax[1].set_ylabel("f$_y$ [N]")
ax[2].set_ylabel("f$_z$ [N]")
ax[2].set_xlabel("Time [s]")
plt.savefig("Nonlinear_Inertia" + ".pdf")

fig, ax = plt.subplots(3, 1)

t = 0
for i in range(len(plt_time_all_F_M)):
    # print(i)
    try:
        if plt_time_all_F_M[i] > plt_time_all_F_M[i + 1] or plt_time_all_F_M[i] < 0.003:
            if plt_time_all_F_M[i] > 0.003:
                print(i)
                t = t + 1
                if i < 300:
                    ax[0].plot(plt_time_all_F_M[last_i:i], np.array(plt_Error_D_M0)[last_i:i], 'r', label = 'Diagonal - MM')
                    ax[0].plot(plt_time_all_F_M[last_i:i], np.array(plt_Error_ND_M0)[last_i:i], 'b', label = 'Non-diagonal - MM')
                ax[0].plot(plt_time_all_F_M[last_i:i], np.array(plt_Error_D_M0)[last_i:i], 'r')
                ax[0].plot(plt_time_all_F_M[last_i:i], np.array(plt_Error_ND_M0)[last_i:i], 'b')
                ax[1].plot(plt_time_all_F_M[last_i:i], np.array(plt_Error_D_M1)[last_i:i], 'r')
                ax[1].plot(plt_time_all_F_M[last_i:i], np.array(plt_Error_ND_M1)[last_i:i], 'b')
                ax[2].plot(plt_time_all_F_M[last_i:i], np.array(plt_Error_D_M2)[last_i:i], 'r')
                ax[2].plot(plt_time_all_F_M[last_i:i], np.array(plt_Error_ND_M2)[last_i:i], 'b')
            last_i = i + 1
    except:
        x = 2
print(t)
ax[0].legend()
ax[0].get_xaxis().set_visible(False)
ax[1].get_xaxis().set_visible(False)

ax[0].set_ylabel("$F_x [N]$")
ax[1].set_ylabel("$F_y [N]$")
ax[2].set_ylabel("$F_z [N]$")
ax[2].set_xlabel("$Time [s]$")

plt.tight_layout()
plt.show()