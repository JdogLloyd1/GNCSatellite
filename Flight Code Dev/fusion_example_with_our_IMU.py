import imufusion
import matplotlib.pyplot as pyplot
import numpy
import sys

# Import sensor data
data = numpy.genfromtxt("IMU sample data.csv", delimiter=",", skip_header=1)

sample_rate = 100  # 100 Hz

timestamp = data[:, 0]
gyroscope = data[:, 1:4]
accelerometer = data[:, 4:7]
magnetometer = data[:, 7:10]

# Instantiate algorithms
offset = imufusion.Offset(sample_rate)
ahrs = imufusion.Ahrs()

ahrs.settings = imufusion.Settings(0.5,  # gain
                                   10,  # acceleration rejection
                                   20,  # magnetic rejection
                                   5 * sample_rate)  # rejection timeout = 5 seconds

# Process sensor data
delta_time = numpy.diff(timestamp, prepend=timestamp[0])

euler = numpy.empty((len(timestamp), 3))
#numpy.empty((len(timestamp), 3))
euler_diff_x = []
euler_diff_y = []
euler_diff_z = []

euler_diff_time = []#numpy.empty([1,1])
internal_states = numpy.empty((len(timestamp), 6))
flags = numpy.empty((len(timestamp), 5))

for index in range(len(timestamp)):
    gyroscope[index] = offset.update(gyroscope[index])

    ahrs.update(gyroscope[index], accelerometer[index], magnetometer[index], delta_time[index])

    euler[index] = ahrs.quaternion.to_euler()

    ahrs_internal_states = ahrs.internal_states
    internal_states[index] = numpy.array([ahrs_internal_states.acceleration_error,
                                          ahrs_internal_states.accelerometer_ignored,
                                          ahrs_internal_states.acceleration_rejection_timer,
                                          ahrs_internal_states.magnetic_error,
                                          ahrs_internal_states.magnetometer_ignored,
                                          ahrs_internal_states.magnetic_rejection_timer])

    ahrs_flags = ahrs.flags
    flags[index] = numpy.array([ahrs_flags.initialising,
                                ahrs_flags.acceleration_rejection_warning,
                                ahrs_flags.acceleration_rejection_timeout,
                                ahrs_flags.magnetic_rejection_warning,
                                ahrs_flags.magnetic_rejection_timeout])
    
for index in range(len(timestamp)):
    if timestamp[index] > 3:
        euler_diff_x.append(euler[index,0]-euler[300,0]) #= numpy.append(euler_diff, [[euler[index][0] - euler[300][0], euler[index][1] - euler[300][1], euler[index][2] - euler[300][2]]], axis=0)
        euler_diff_y.append(euler[index,1]-euler[300,1])
        euler_diff_z.append(euler[index,2]-euler[300,2])
        euler_diff_time.append(timestamp[index]-3)
       #euler_diff_time = numpy.append(euler_diff_time, [[timestamp[index]-3]], axis=0)


def plot_bool(axis, x, y, label):
    axis.plot(x, y, "tab:cyan", label=label)
    pyplot.sca(axis)
    pyplot.yticks([0, 1], ["False", "True"])
    axis.grid()
    axis.legend()


pyplot.figure(1)
pyplot.plot(timestamp, euler[:, 0], "tab:red", label="Roll")
pyplot.plot(timestamp, euler[:, 1], "tab:green", label="Pitch")
pyplot.plot(timestamp, euler[:, 2], "tab:blue", label="Yaw")
pyplot.legend()

pyplot.figure(2)
pyplot.plot(euler_diff_time, euler_diff_x, "tab:red", label="Roll diff from init")
pyplot.plot(euler_diff_time, euler_diff_y, "tab:green", label="Pitch diff from init")
pyplot.plot(euler_diff_time, euler_diff_z, "tab:blue", label="Yaw diff from init")
pyplot.legend()

pyplot.show()
