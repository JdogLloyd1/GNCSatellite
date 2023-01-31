# Script built to read data from 9 axis IMU. 3.3V input, I2C from GPIO pins 2 and 3
# https://maker.pro/raspberry-pi/tutorial/how-to-interface-an-imu-sensor-with-a-raspberry-pi
# https://piwheels.org/project/fabo9axis-mpu9250-python3/
# https://github.com/adityanarayanan03/MPU9250


import time
import sys
import RPi.GPIO as GPIO
import FaBo9Axis_MPU9250

mpu9250 = FaBo9Axis_MPU9250.MPU9250()

# Pin Setup
GPIO.setmode(GPIO.BCM)
# GPIO.setup()
# SCL = GPIO3
# SDA = GPIO2
# VCC = GPIO1
# GND = GPIO6

try:

    while True:

        accel = mpu9250.readAccel()

        print( " ax = " , ( accel['x'] ))

        print( " ay = " , ( accel['y'] ))

        print( " az = " , ( accel['z'] ))
 

        gyro = mpu9250.readGyro()

        print( " gx = " , ( gyro['x'] ))

        print( " gy = " , ( gyro['y'] ))

        print( " gz = " , ( gyro['z'] ))

 
        mag = mpu9250.readMagnet()

        print( " mx = " , ( mag['x'] ))

        print( " my = " , ( mag['y'] ))

        print( " mz = " , ( mag['z'] ))

        print

 
        time.sleep(0.1)

 

except KeyboardInterrupt:

    sys.exit()




GPIO.cleanup()

