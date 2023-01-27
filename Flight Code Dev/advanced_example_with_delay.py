import imufusion
import matplotlib.pyplot as pyplot
import numpy
import sys

# Import sensor data
data = numpy.genfromtxt("sensor_data.csv", delimiter=",", skip_header=1)

sample_rate = 100  # 100 Hz

n = 1000 #the number of data points the script is behind by

timestamp = data[:, 0]
gyroscope = data[0:n, 1:4]
accelerometer = data[0:n, 4:7]
magnetometer = data[0:n, 7:10]

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
internal_states = numpy.empty((len(timestamp), 6))
flags = numpy.empty((len(timestamp), 5))

euler_live_x = []
euler_live_y = []
euler_live_z = []
live_time = []



for index in range(len(timestamp)):
    #timestamp = np.append(timestamp, [[data[index,0]]], axis = 0)
    if index < len(timestamp) - n: #don't want to append at the end
        accelerometer = numpy.append(accelerometer, [data[index+n, 1:4]], axis = 0)
        gyroscope = numpy.append(gyroscope, [data[index+n, 4:7]], axis = 0)
        magnetometer = numpy.append(magnetometer, [data[index+n, 7:10]], axis = 0)
    
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
                                
    euler_live_x.append(euler[index,0])
    euler_live_y.append(euler[index,1])
    euler_live_z.append(euler[index,2])
    live_time.append(timestamp[index])


def plot_bool(axis, x, y, label):
    axis.plot(x, y, "tab:cyan", label=label)
    pyplot.sca(axis)
    pyplot.yticks([0, 1], ["False", "True"])
    axis.grid()
    axis.legend()


# Plot Euler angles
pyplot.figure(1)
pyplot.plot(live_time, euler_live_x, "tab:red", label="Roll")
pyplot.plot(live_time, euler_live_y, "tab:green", label="Pitch")
pyplot.plot(live_time, euler_live_z, "tab:blue", label="Yaw")
pyplot.legend()

if len(sys.argv) == 1:  # don't show plots when script run by CI
    pyplot.show()
