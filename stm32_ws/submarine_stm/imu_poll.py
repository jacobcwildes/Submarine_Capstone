import serial
import time
import board
import math
from copy import copy
import warnings
import numpy as np
from numpy.linalg import norm
import numbers

from adafruit_lsm6ds.lsm6ds3 import LSM6DS3 as LSM6DS
from adafruit_lis3mdl import LIS3MDL

from madgwickahrs import MadgwickAHRS

i2c = board.I2C() 
accel_gyro = LSM6DS(i2c)
mag = LIS3MDL(i2c)

imu_fusion = MadgwickAHRS(sampleperiod=0.01)


roll = 0
pitch = 0
yaw = 0

x_bias = 0
z_bias = 0
bias_checks = 50
current_check = 0
x_sum = 0
z_sum = 0

for current_check in range(bias_checks):
    accel = accel_gyro.acceleration
    x_accel = accel[0]
    z_accel = accel[2]
    x_sum += x_accel
    z_sum += z_accel
x_bias = x_sum / (current_check + 1)
z_bias = z_sum / (current_check + 1)

while True:
    accel = accel_gyro.acceleration
    gyro = accel_gyro.gyro
    magnet = mag.magnetic
    y_mag = magnet[1]
    x_mag = magnet[0]
    x_accel = accel[0] - x_bias
    z_accel = -accel[2] + z_bias
    
    imu_fusion.update_imu(gyro, accel)
    
    roll, pitch, yaw = imu_fusion.quaternion.to_euler123()
    
    roll *= (180/math.pi)
    if roll < 0: roll += 360
    
    north = math.atan2(y_mag,x_mag)*(180/math.pi)
    if north < 0: north += 360
    north = north - 30
    if north < 0: north += 360
    
    print("ROLL: {}".format(roll))
    print("DEGREE: {}".format(north))
    print("FORWARD: {}".format(x_accel))
    print("UP: {}".format(z_accel))
    print("-----------------------------")
    
    time.sleep(0.01)
