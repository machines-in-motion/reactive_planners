from matplotlib import pyplot as plt
import numpy as np


plt_left_eef_real_posxFalse = np.loadtxt('plt_left_eef_real_posxFalse.txt', dtype=float)
plt_left_eef_real_posxTrue = np.loadtxt('plt_left_eef_real_posxTrue.txt', dtype=float)
plt_left_eef_real_posyFalse = np.loadtxt('plt_left_eef_real_posyFalse.txt', dtype=float)
plt_left_eef_real_posyTrue = np.loadtxt('plt_left_eef_real_posyTrue.txt', dtype=float)
plt_left_eef_real_poszFalse = np.loadtxt('plt_left_eef_real_poszFalse.txt', dtype=float)
plt_left_eef_real_poszTrue = np.loadtxt('plt_left_eef_real_poszTrue.txt', dtype=float)
plt_left_foot_positionxFalse = np.loadtxt('plt_left_foot_positionxFalse.txt', dtype=float)
plt_left_foot_positionxTrue = np.loadtxt('plt_left_foot_positionxTrue.txt', dtype=float)
plt_left_foot_positionyFalse = np.loadtxt('plt_left_foot_positionyFalse.txt', dtype=float)
plt_left_foot_positionyTrue = np.loadtxt('plt_left_foot_positionyTrue.txt', dtype=float)
plt_left_foot_positionzFalse = np.loadtxt('plt_left_foot_positionzFalse.txt', dtype=float)
plt_left_foot_positionzTrue = np.loadtxt('plt_left_foot_positionzTrue.txt', dtype=float)
plt_next_step_locationxFalse = np.loadtxt('plt_next_step_locationxFalse.txt', dtype=float)
plt_next_step_locationxTrue = np.loadtxt('plt_next_step_locationxTrue.txt', dtype=float)
plt_next_step_locationyFalse = np.loadtxt('plt_next_step_locationyFalse.txt', dtype=float)
plt_next_step_locationyTrue = np.loadtxt('plt_next_step_locationyTrue.txt', dtype=float)
plt_right_eef_real_posxFalse = np.loadtxt('plt_right_eef_real_posxFalse.txt', dtype=float)
plt_right_eef_real_posxTrue = np.loadtxt('plt_right_eef_real_posxTrue.txt', dtype=float)
plt_right_eef_real_posyFalse = np.loadtxt('plt_right_eef_real_posyFalse.txt', dtype=float)
plt_right_eef_real_posyTrue = np.loadtxt('plt_right_eef_real_posyTrue.txt', dtype=float)
plt_right_eef_real_poszFalse = np.loadtxt('plt_right_eef_real_poszFalse.txt', dtype=float)
plt_right_eef_real_poszTrue = np.loadtxt('plt_right_eef_real_poszTrue.txt', dtype=float)
plt_right_foot_positionxFalse = np.loadtxt('plt_right_foot_positionxFalse.txt', dtype=float)
plt_right_foot_positionxTrue = np.loadtxt('plt_right_foot_positionxTrue.txt', dtype=float)
plt_right_foot_positionyFalse = np.loadtxt('plt_right_foot_positionyFalse.txt', dtype=float)
plt_right_foot_positionyTrue = np.loadtxt('plt_right_foot_positionyTrue.txt', dtype=float)
plt_right_foot_positionzFalse = np.loadtxt('plt_right_foot_positionzFalse.txt', dtype=float)
plt_right_foot_positionzTrue = np.loadtxt('plt_right_foot_positionzTrue.txt', dtype=float)


fig, ax = plt.subplots(2, 1)
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

end = 1000#1190 + 200
time = [i / 1000. for i in range(end)]

ax[0].plot(time, plt_left_foot_positionzFalse[:end] - 0.017055, label="left_des")
ax[0].plot(time, plt_right_foot_positionzFalse[:end] - 0.017055, label="right_des")
ax[0].plot(time, plt_left_eef_real_poszFalse[:end] - 0.017055, label="left")
ax[0].plot(time, plt_right_eef_real_poszFalse[:end] - 0.017055, label="right")

ax[1].plot(time, plt_left_foot_positionzTrue[:end] - 0.017055, label="left_des")
ax[1].plot(time, plt_right_foot_positionzTrue[:end] - 0.017055, label="right_des")
ax[1].plot(time, plt_left_eef_real_poszTrue[:end] - 0.017055, label="left")
ax[1].plot(time, plt_right_eef_real_poszTrue[:end] - 0.017055, label="right")

ax[0].legend(loc="upper left")
# ax[1].legend()
# ax[0].get_xaxis().set_visible(False)
ax[0].set_xlim(left=0., right=np.max(time))
ax[1].set_xlim(left=0., right=np.max(time))
ax[0].set_ylabel("z [m]")
ax[1].set_ylabel("z [m]")
ax[1].set_xlabel("Time [s]")
plt.savefig("Z" + ".pdf")

fig, ax = plt.subplots(2, 2)

# ax[0][0].get_xaxis().set_visible(False)
# ax[0][1].get_xaxis().set_visible(False)
# ax[1][1].get_yaxis().set_visible(False)

ax[0][0].plot(time, plt_left_foot_positionxFalse[:end], label="left_des")
ax[0][0].plot(time, plt_right_foot_positionxFalse[:end], label="right_des")
ax[0][0].plot(time, plt_left_eef_real_posxFalse[:end], label="left")
ax[0][0].plot(time, plt_right_eef_real_posxFalse[:end], label="right")
ax[0][0].plot(time, plt_next_step_locationxFalse[:end], label="next_step")

ax[1][0].plot(time, plt_left_foot_positionyFalse[:end], label="left_des")
ax[1][0].plot(time, plt_right_foot_positionyFalse[:end], label="right_des")
ax[1][0].plot(time, plt_left_eef_real_posyFalse[:end], label="left")
ax[1][0].plot(time, plt_right_eef_real_posyFalse[:end], label="right")
ax[1][0].plot(time, plt_next_step_locationyFalse[:end], label="next_step")

ax[0][1].plot(time, plt_left_foot_positionxTrue[:end], label="left_des")
ax[0][1].plot(time, plt_right_foot_positionxTrue[:end], label="right_des")
ax[0][1].plot(time, plt_left_eef_real_posxTrue[:end], label="left")
ax[0][1].plot(time, plt_right_eef_real_posxTrue[:end], label="right")
ax[0][1].plot(time, plt_next_step_locationxTrue[:end], label="next_step")


ax[1][1].plot(time, plt_left_foot_positionyTrue[:end], label="left_des")
ax[1][1].plot(time, plt_right_foot_positionyTrue[:end], label="right_des")
ax[1][1].plot(time, plt_left_eef_real_posyTrue[:end], label="left")
ax[1][1].plot(time, plt_right_eef_real_posyTrue[:end], label="right")
ax[1][1].plot(time, plt_next_step_locationyTrue[:end], label="next_step")

# ax[0][0].set_ylim(botom=0., top=np.max(plt_time_all_F_M))
# ax[0][1].set_ylim(botom=0., top=np.max(plt_time_all_F_M))

ax[0][0].set_ylabel("x [m]")
ax[1][0].set_ylabel("y [m]")
ax[1][0].set_xlabel("Time [s]")
ax[1][1].set_xlabel("Time [s]")
# ax[0][0].legend()
# ax[0][1].legend()
ax[1][0].legend()
# ax[1][1].legend()
ax[0][0].set_xlim(left=0., right=np.max(time))
ax[0][1].set_xlim(left=0., right=np.max(time))
ax[1][0].set_xlim(left=0., right=np.max(time))
ax[1][1].set_xlim(left=0., right=np.max(time))
ax[1][1].set_ylim(bottom=np.min(plt_next_step_locationyFalse) - 0.05, top=np.max(plt_next_step_locationyTrue) + 0.05)
ax[1][0].set_ylim(bottom=np.min(plt_next_step_locationyFalse) - 0.05, top=np.max(plt_next_step_locationyTrue) + 0.05)
plt.savefig("XY" + ".pdf")


plt.tight_layout()
plt.show()