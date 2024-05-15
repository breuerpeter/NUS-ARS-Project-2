import numpy as np
import matplotlib.pyplot as plt
import csv

x_true = []
x_est = []
y_true = []
y_est = []
z_true = []
z_est = []
a_true = []
a_est = []

with open('/home/pbreuer/ROS1/ee4308/Project-2/EKF_data.txt', 'r') as datafile:
    plotting = csv.reader(datafile, delimiter=',')

    for ROWS in plotting:
        x_true.append(float(ROWS[0]))
        x_est.append(float(ROWS[1]))
        y_true.append(float(ROWS[2]))
        y_est.append(float(ROWS[3]))
        z_true.append(float(ROWS[4]))
        z_est.append(float(ROWS[5]))
        a_true.append(float(ROWS[6]))
        a_est.append(float(ROWS[7]))

x_mse = ((np.asarray(x_true) - np.asarray(x_est))**2).mean()
print("x_mse:", x_mse)

x_mse_trials = [0.03767, 0.03428, 0.03556, 0.02590, 0.1040, 0.08011, 0.01859, 0.02082, 0.008147, 0.03419, 0.01716, 0.03159]
print("x_mse (1)", np.asarray(x_mse_trials)[0:3].mean())
print("x_mse (10)", np.asarray(x_mse_trials)[4:7].mean())
print("x_mse (100)", np.asarray(x_mse_trials)[8:11].mean())

x_means = [0.03584, 0.06757, 0.01983]

y_mse = ((np.asarray(y_true) - np.asarray(y_est))**2).mean()
print("y_mse:", y_mse)

y_mse_trials = [0.03478, 0.01646, 0.1102, 0.02373, 0.03989, 0.01223, 0.005087, 0.005789, 0.03369, 0.02628, 0.02780, 0.01629]
print("y_mse (1)", np.asarray(y_mse_trials)[0:3].mean())
print("y_mse (10)", np.asarray(y_mse_trials)[4:7].mean())
print("y_mse (100)", np.asarray(y_mse_trials)[8:11].mean())

y_means = [0.05381, 0.01907, 0.02926]

z_mse = ((np.asarray(z_true) - np.asarray(z_est))**2).mean()
print("z_mse:", z_mse)

z_mse_trials =  [0.1089, 0.01190, 0.01045, 0.004158, 0.01323, 0.02462, 0.04129, 0.01279, 0.03424, 0.0004149, 0.02883, 0.001961]
print("z_mse (1)", np.asarray(z_mse_trials)[0:3].mean())
print("z_mse (10)", np.asarray(z_mse_trials)[4:7].mean())
print("z_mse (100)", np.asarray(z_mse_trials)[8:11].mean())

z_means = [0.04375, 0.02638, 0.02116]

a_mse = ((np.asarray(a_true) - np.asarray(a_est))**2).mean()
print("a_mse:", a_mse)

a_mse_trials =  [0.3931, 0.7743, 0.5275, 0.5241, 0.1386, 0.5228, 0.6430, 1.02438, 0.64169, 0.1379, 0.5205, 0.8968]
print("a_mse (1)", np.asarray(a_mse_trials)[0:3].mean())
print("a_mse (10)", np.asarray(a_mse_trials)[4:7].mean())
print("a_mse (0.1)", np.asarray(a_mse_trials)[8:11].mean())

a_means = [0.5650, 0.4348, 0.4336]

x_xyz = [1, 10, 100]
x_a = [0.1, 1, 10]

plt.figure()
plt.plot(x_xyz, x_means, c='g', label='x')
plt.plot(x_xyz, y_means, c='b', label='y')
plt.plot(x_xyz, z_means, c='y', label='z')
# plt.plot(x_a, a_means, c='r', label='yaw')
plt.xlabel('variance [-]')
plt.ylabel('mean squard error (avg of 4 trials) [-]')
plt.title('IMU Variance Tuning')
plt.legend()
plt.savefig(fname='/home/pbreuer/ROS1/ee4308/Project-2/imu_var.pdf', format='pdf')
plt.show()

plt.figure()
plt.plot(x_a, a_means, c='r', label='yaw')
plt.xlabel('variance [-]')
plt.ylabel('mean squard error (avg of 4 trials) [-]')
plt.title('IMU Variance Tuning')
plt.legend()
plt.savefig(fname='/home/pbreuer/ROS1/ee4308/Project-2/a_var.pdf', format='pdf')
plt.show()






""" rc_fonts = {
    "text.usetex": True,
}
plt.rcParams.update(rc_fonts) """

plt.figure()
plt.plot(x_true, y_true, c='g', label='ground truth')
plt.plot(x_est, y_est, c='r', label='estimate')
plt.xlim(min([min(x_true), min(x_est)]), max([max(x_true), max(x_est)]))
plt.ylim(min([min(y_true), min(y_est)]), max([max(y_true), max(y_est)]))
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.title('XY Position')
plt.legend()
plt.savefig(fname='/home/pbreuer/ROS1/ee4308/Project-2/xy.pdf', format='pdf')
plt.show()

plt.figure()
plt.plot(z_true, c='g', label='ground truth')
plt.plot(z_est, c='r', label='estimate')
plt.ylim(min([min(z_true), min(z_est)]), max([max(z_true), max(z_est)]))
plt.xlabel('time step [-]')
plt.ylabel('z [m]')
plt.title('Z Position')
plt.legend()
plt.savefig(fname='/home/pbreuer/ROS1/ee4308/Project-2/z.pdf', format='pdf')
plt.show()

plt.figure()
plt.plot(a_true, c='g', label='ground truth')
plt.plot(a_est, c='r', label='estimate')
plt.ylim(min([min(a_true), min(a_est)]), max([max(a_true), max(a_est)]))
plt.xlabel('time step [-]')
plt.ylabel('yaw [rad]')
plt.title('Yaw Angle')
plt.legend()
plt.savefig(fname='/home/pbreuer/ROS1/ee4308/Project-2/yaw.pdf', format='pdf')
plt.show()
