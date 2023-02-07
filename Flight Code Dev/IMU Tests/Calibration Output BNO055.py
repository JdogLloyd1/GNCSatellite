# Continously output sensor calibration status while chip calibrates

import time 
import board
import busio
import adafruit_bno055 

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

last_val = 0xFFFF
print("Quaternion: {}".format(sensor.quaternion))

# Calibration Process

# Gyroscope - hold chip still for a few seconds

# Accelerometer - Hold sensor in about 6 different positions for a few 
# seconds like the sides of a cube. 

# Magnetometer - Pick up the sensor and move it in a figure 8 pattern
# until readout is 3. Hold sensor away from any large metal objects, or 
# the calibration will get messed up. 

# Once all sensors are calibrated, let the chip sit for a few seconds to 
# finish system calibration

# Calibration for each sensor is on a scale from 0-3, 3 being the 
# highest. Calibration values should jump to 1 after a few calibration 
# steps if it is working properly

while True:
	
	print("Sys/Gyro/Acc/Mag")
	print(sensor.calibration_status)
	print(sensor.calibrated)
	print("")
	time.sleep(1)
