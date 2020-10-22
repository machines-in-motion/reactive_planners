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
plt_is_left_in_contactFalse = np.loadtxt('plt_is_left_in_contactFalse.txt', dtype=float)
plt_is_left_in_contactTrue = np.loadtxt('plt_is_left_in_contactTrue.txt', dtype=float)


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

end = 100000
time = [i / 1000. for i in range(end)]
#Z
ground_height = 0.018
for i in range(end - 1):
        if plt_is_left_in_contactFalse[i] != plt_is_left_in_contactFalse[i + 1]:
                if plt_is_left_in_contactFalse[i]:
                        minimum = plt_right_eef_real_poszFalse[i]
                        for j in range(50):
                                minimum = min(minimum, plt_right_eef_real_poszFalse[i - j])
                        if minimum > ground_height:
                                print("P:   LEFT IS LATE")
                else:
                        minimum = plt_left_eef_real_poszFalse[i]
                        for j in range(50):
                                minimum = min(minimum, plt_left_eef_real_poszFalse[i - j])
                        if minimum > ground_height:
                                print("P   RIGHT IS LATE")

        if plt_is_left_in_contactTrue[i] != plt_is_left_in_contactTrue[i + 1]:
                if plt_is_left_in_contactTrue[i]:
                        minimum = plt_right_eef_real_poszTrue[i]
                        for j in range(50):
                                minimum = min(minimum, plt_right_eef_real_poszTrue[i - j])
                        if minimum > ground_height:
                                print(" LEFT IS LATE")
                else:
                        minimum = plt_left_eef_real_poszTrue[i]
                        for j in range(50):
                                minimum = min(minimum, plt_left_eef_real_poszTrue[i - j])
                        if minimum > ground_height:
                                print("RIGHT IS LATE")
# => contact happens sooner than des contact time.

poly_delay = 0
poly_delta_x = 0
poly_delta_y = 0
delay = 0
delta_x = 0
delta_y = 0
plt_poly_delay = []
plt_poly_delta_x = []
plt_poly_delta_y = []
plt_delay = []
plt_delta_x = []
plt_delta_y = []


for i in range(end - 1):
        if plt_is_left_in_contactFalse[i] != plt_is_left_in_contactFalse[i + 1]:
                if plt_is_left_in_contactFalse[i]:
                        minimum = plt_right_eef_real_poszFalse[i]
                        contact_time = 0
                        contact_x = plt_right_eef_real_posxFalse[i]
                        contact_y = plt_right_eef_real_posyFalse[i]
                        for j in range(50):
                                if ground_height >= plt_right_eef_real_poszFalse[i - j]:
                                        minimum = plt_right_eef_real_poszFalse[i - j]
                                        contact_time = j
                                        # contact_x = plt_right_eef_real_posxFalse[i - j]
                                        # contact_y = plt_right_eef_real_posyFalse[i - j]
                else:
                        minimum = plt_left_eef_real_poszFalse[i]
                        contact_time = 0
                        contact_x = plt_left_eef_real_posxFalse[i]
                        contact_y = plt_left_eef_real_posyFalse[i]
                        for j in range(50):
                                if ground_height >= plt_left_eef_real_poszFalse[i - j]:
                                        minimum = plt_left_eef_real_poszFalse[i - j]
                                        contact_time = j
                                        # contact_x = plt_left_eef_real_posxFalse[i - j]
                                        # contact_y = plt_left_eef_real_posyFalse[i - j]
                print("Poly: ", i, contact_time)
                poly_delay += contact_time
                poly_delta_x += abs(plt_next_step_locationxFalse[i - contact_time - 1] - contact_x) * 1000
                poly_delta_y += abs(plt_next_step_locationyFalse[i - contact_time - 1] - contact_y) * 1000
                plt_poly_delay += [contact_time]
                plt_poly_delta_x += [abs(plt_next_step_locationxFalse[i - contact_time - 1] - contact_x) * 1000]
                plt_poly_delta_y += [abs(plt_next_step_locationyFalse[i - contact_time - 1] - contact_y) * 1000]

        if plt_is_left_in_contactTrue[i] != plt_is_left_in_contactTrue[i + 1]:
                if plt_is_left_in_contactTrue[i]:
                        minimum = plt_right_eef_real_poszTrue[i]
                        contact_time = 0
                        contact_x = plt_right_eef_real_posxTrue[i]
                        contact_y = plt_right_eef_real_posyTrue[i]
                        for j in range(50):
                                if ground_height >= plt_right_eef_real_poszTrue[i - j]:
                                        minimum = plt_right_eef_real_poszTrue[i - j]
                                        contact_time = j
                                        # contact_x = plt_right_eef_real_posxTrue[i - j]
                                        # contact_y = plt_right_eef_real_posyTrue[i - j]
                else:
                        minimum = plt_left_eef_real_poszTrue[i]
                        contact_time = 0
                        contact_x = plt_left_eef_real_posxTrue[i]
                        contact_y = plt_left_eef_real_posyTrue[i]
                        for j in range(50):
                                if ground_height >= plt_left_eef_real_poszTrue[i - j]:
                                        minimum = plt_left_eef_real_poszTrue[i - j]
                                        contact_time = j
                                        # contact_x = plt_left_eef_real_posxTrue[i - j]
                                        # contact_y = plt_left_eef_real_posyTrue[i - j]
                print("MPC: ", i, contact_time)
                delay += contact_time
                delta_x += abs(plt_next_step_locationxTrue[i - contact_time - 1] - contact_x) * 1000
                delta_y += abs(plt_next_step_locationyTrue[i - contact_time - 1] - contact_y) * 1000
                plt_delay += [contact_time]
                plt_delta_x += [abs(plt_next_step_locationxTrue[i - contact_time - 1] - contact_x) * 1000]
                plt_delta_y += [abs(plt_next_step_locationyTrue[i - contact_time - 1] - contact_y) * 1000]

print("poly_delay: ", poly_delay)
print("poly_delta_x: ", poly_delta_x)
print("poly_delta_y: ", poly_delta_y)
print("delay: ", delay)
print("delta_x: ", delta_x)
print("delta_y: ", delta_y)


# ax[0].plot(time, plt_left_foot_positionzFalse[:end] - 0.017055, label="left_des")
# ax[0].plot(time, plt_right_foot_positionzFalse[:end] - 0.017055, label="right_des")
# ax[0].plot(time, plt_left_eef_real_poszFalse[:end] - 0.017055, label="left")
# ax[0].plot(time, plt_right_eef_real_poszFalse[:end] - 0.017055, label="right")
#
# ax[1].plot(time, plt_left_foot_positionzTrue[:end] - 0.017055, label="left_des")
# ax[1].plot(time, plt_right_foot_positionzTrue[:end] - 0.017055, label="right_des")
# ax[1].plot(time, plt_left_eef_real_poszTrue[:end] - 0.017055, label="left")
# ax[1].plot(time, plt_right_eef_real_poszTrue[:end] - 0.017055, label="right")
#
# ax[0].legend(loc="upper left")
# # ax[1].legend()
# ax[0].get_xaxis().set_visible(False)
# ax[0].set_xlim(left=0., right=np.max(time))
# ax[1].set_xlim(left=0., right=np.max(time))
# ax[0].set_ylabel("z [m]")
# ax[1].set_ylabel("z [m]")
# ax[1].set_xlabel("Time [s]")
# plt.savefig("Z" + ".pdf")
#
# fig, ax = plt.subplots(2, 2)
#
# ax[0][0].get_xaxis().set_visible(False)
# ax[0][1].get_xaxis().set_visible(False)
#
# ax[0][0].plot(time, plt_left_foot_positionxFalse[:end], label="left_des")
# ax[0][0].plot(time, plt_right_foot_positionxFalse[:end], label="right_des")
# ax[0][0].plot(time, plt_left_eef_real_posxFalse[:end], label="left")
# ax[0][0].plot(time, plt_right_eef_real_posxFalse[:end], label="right")
# ax[0][0].plot(time, plt_next_step_locationxFalse[:end], label="next_step")
#
# ax[1][0].plot(time, plt_left_foot_positionyFalse[:end], label="left_des")
# ax[1][0].plot(time, plt_right_foot_positionyFalse[:end], label="right_des")
# ax[1][0].plot(time, plt_left_eef_real_posyFalse[:end], label="left")
# ax[1][0].plot(time, plt_right_eef_real_posyFalse[:end], label="right")
# ax[1][0].plot(time, plt_next_step_locationyFalse[:end], label="next_step")
#
# ax[0][1].plot(time, plt_left_foot_positionxTrue[:end], label="left_des")
# ax[0][1].plot(time, plt_right_foot_positionxTrue[:end], label="right_des")
# ax[0][1].plot(time, plt_left_eef_real_posxTrue[:end], label="left")
# ax[0][1].plot(time, plt_right_eef_real_posxTrue[:end], label="right")
# ax[0][1].plot(time, plt_next_step_locationxTrue[:end], label="next_step")
#
#
# ax[1][1].plot(time, plt_left_foot_positionyTrue[:end], label="left_des")
# ax[1][1].plot(time, plt_right_foot_positionyTrue[:end], label="right_des")
# ax[1][1].plot(time, plt_left_eef_real_posyTrue[:end], label="left")
# ax[1][1].plot(time, plt_right_eef_real_posyTrue[:end], label="right")
# ax[1][1].plot(time, plt_next_step_locationyTrue[:end], label="next_step")
#
# # ax[0][0].set_ylim(botom=0., top=np.max(plt_time_all_F_M))
# # ax[0][1].set_ylim(botom=0., top=np.max(plt_time_all_F_M))
#
# ax[0][0].set_ylabel("x [m]")
# ax[1][0].set_ylabel("y [m]")
# ax[1][0].set_xlabel("Time [s]")
# ax[1][1].set_xlabel("Time [s]")
# ax[0][0].legend()
# # ax[0][1].legend()
# # ax[1][0].legend()
# # ax[1][1].legend()
# ax[0][0].set_xlim(left=0., right=np.max(time))
# ax[0][1].set_xlim(left=0., right=np.max(time))
# ax[1][0].set_xlim(left=0., right=np.max(time))
# ax[1][1].set_xlim(left=0., right=np.max(time))
# plt.savefig("XY" + ".pdf")
#
# fig, ax = plt.subplots(2, 1)
end_delta = min(len(plt_poly_delay), len(plt_delay))
ax[0].plot(plt_poly_delta_x[:end_delta], label="Polynomial")
ax[0].plot(plt_delta_x[:end_delta], 'r', label="MPC")
ax[0].axhline(poly_delta_x / len(plt_poly_delta_x), 0, 1, color='darkblue', ls='--', linewidth=2)
ax[0].axhline(delta_x / len(plt_delta_x), 0, 1, color='maroon', ls='--', linewidth=2)

ax[1].plot(plt_poly_delta_y[:end_delta], label="Polynomial")
ax[1].plot(plt_delta_y[:end_delta], 'r', label="MPC")
ax[1].axhline(poly_delta_y / len(plt_poly_delta_y), 0, 1, color='darkblue', ls='--', linewidth=2)
ax[1].axhline(delta_y / len(plt_delta_y), 0, 1, color='maroon', ls='--', linewidth=2)

ax[0].get_xaxis().set_visible(False)
ax[0].set_ylabel("e$_x$ [mm]")
ax[1].set_ylabel("e$_y$ [mm]")
ax[1].set_xlabel("Step number")
ax[0].legend()
ax[0].set_xlim(left=0., right=len(plt_delta_y))
ax[1].set_xlim(left=0., right=len(plt_delta_y))
plt.savefig("Polynomial_MPC" + ".pdf")

fig, ax = plt.subplots(1, 1)
ax.plot(plt_poly_delay[:end_delta], label="Polynomial")
ax.plot(plt_delay[:end_delta], 'r', label="MPC")
ax.axhline(poly_delay * 1.0  / len(plt_poly_delay), 0, 1, color='darkblue', ls='--', linewidth=2)
ax.axhline(delay * 1.0  / len(plt_delay), 0, 1, color='maroon', ls='--', linewidth=2)


ax.set_xlim(left=0., right=len(plt_delta_y))
ax.set_ylabel("e$_{time}$ [ms]")
ax.set_xlabel("Step number")
ax.legend()
plt.savefig("Polynomial_MPC_time" + ".pdf")
# plt.savefig("Polynomial_MPC_time" + ".pdf") - contact_time - 1
plt.tight_layout()
plt.show()