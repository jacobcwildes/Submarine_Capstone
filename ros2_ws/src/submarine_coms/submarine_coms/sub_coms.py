import rclpy
from rclpy.node import Node
import message_filters
from message_filters import Subscriber
import string
from com_interfaces.msg import ComInfo
from com_interfaces.msg import DataInfo
from rclpy import qos

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

#In my experience importing a module from another file can be... finnicky. This
#solution has worked every time for me
import sys
import os
sys.path.insert(0, 'src/submarine_coms/submarine_coms')
from imu import imu

class sub_data():
	def __init__(self):
		self.degreesNorth = 0
		self.forwardAccel = 0
		self.upwardAccel = 0
		self.batteryVoltage = 0
		self.ballastRight = 0
		self.ballastLeft = 0
		self.error = ""
	
	def package_data(self):
		data = DataInfo()
		data.degrees_north = self.degreesNorth
		data.forward = self.forwardAccel
		data.upward = self.upwardAccel
		data.voltage_battery = self.batteryVoltage
		data.ballast_left = self.ballastLeft
		data.ballast_right = self.ballastRight
		data.error = self.error
		
		return data
			

class stm_send():
	def __init__(self):
		
		self.SerialObj = serial.Serial('/dev/ttyACM0')
		self.SerialObj.baudrate = 115200  # set Baud rate to 115200
		self.SerialObj.bytesize = 8   # Number of data bits = 8
		self.SerialObj.parity  ='N'   # No parity
		self.SerialObj.stopbits = 1   # Number of Stop bits = 1
		
		self.depthUp = 0
		self.depthDown = 0
		self.forwardThrust = 10
		self.turnThrust = 0
		self.camUpDown = 0
		self.camLeftRight = 0
		self.roll = None
		
	def handshake(self, con):
		if self.roll != None:
			print(self.roll)
			print("-----------------------------")	
			try:
				forwardBinary = bin(self.forwardThrust).split('b')[1]
				turnBinary = bin(self.turnThrust).split('b')[1]
				camUpDownBinary = bin(self.camUpDown).split('b')[1]
				camLeftRightBinary = bin(self.camLeftRight).split('b')[1]
				rollBinary = bin(self.roll).split('b')[1]

				bitfield = str(self.depthUp) + str(self.depthDown) + str(forwardBinary).zfill(8) + str(turnBinary).zfill(8) + str(camUpDownBinary).zfill(8) + str(camLeftRightBinary).zfill(8) + str(rollBinary).zfill(8)
				
				self.SerialObj.write(bitfield.encode()) #42 bits
				print("Sending: " + bitfield)
				received = str(self.SerialObj.readline().decode('ascii')).replace('\r','').replace('\n','').replace('\x00', '')
				received_split = received.split(',')

				if len(received_split) != 6: raise Exception
				print("Received: " + str(received_split))
				
				con.batteryVoltage = int(received_split[0])/10
				con.ballastRight = received_split[1]
				con.ballastLeft = received_split[2]
				
				errors = []
				if not (int(received_split[3])): errors.append("Left Driver Fault")
				if not (int(received_split[4])): errors.append("Prop Driver Fault")
				if not (int(received_split[5])): errors.append("Right Driver Fault")
				if (self.roll < 45 or self.roll > 135): errors.append("TOO MUCH ROLL!")
				
				con.error = ','.join(errors)
				
				
			except:
				print("BAD DATA")
			
			
			

class imu():
	def __init__(self):
		self.i2c = board.I2C() 
		self.accel_gyro = LSM6DS(self.i2c)
		self.mag = LIS3MDL(self.i2c)
		
		self.x_bias = 0
		self.z_bias = 0
		self.bias_checks = 50
		self.x_sum = 0
		self.z_sum = 0
		self.imu_fusion = MadgwickAHRS()
		
		for current_check in range(self.bias_checks):
			accel = self.accel_gyro.acceleration
			x_accel = accel[0]
			z_accel = accel[2]
			self.x_sum += x_accel
			self.z_sum += z_accel
			time.sleep(0.02)
		self.x_bias = self.x_sum / (self.bias_checks)
		self.z_bias = self.z_sum / (self.bias_checks)
	
	def measure(self, con, sub):
		accel = self.accel_gyro.acceleration
		gyro = self.accel_gyro.gyro
		magnet = self.mag.magnetic
		y_mag = magnet[1]
		x_mag = magnet[0]
		x_accel = accel[0] - self.x_bias
		z_accel = -accel[2] + self.z_bias
		
		self.imu_fusion.update_imu(gyro, accel)
			
		roll, pitch, yaw = self.imu_fusion.quaternion.to_euler123()
		
		roll = int(roll*(180/math.pi))
		if abs(roll) > 90: roll *= (90/abs(roll)) # -90 <-> 90
		sub.roll = roll + 90 # 0 <-> 180
			
		north = math.atan2(y_mag,x_mag)*(180/math.pi)
		if north < 0: north += 360
		north = north - 30
		if north < 0: north += 360
			
		con.degreesNorth = north
		con.forwardAccel = x_accel
		con.upwardAccel = z_accel 
		
		
class Submarine(Node):

	def __init__(self):
		super().__init__('submarine_coms')
			
		#Command sub (UDP)
		self.subscription = self.create_subscription(ComInfo, 'com_info', self.com_callback, qos.qos_profile_services_default)
		self.subscription
		#Data publisher (UDP)
		self.data_pub = self.create_publisher(DataInfo, 'data_info', qos.qos_profile_sensor_data)
			
		self.motion = imu()
		self.toController = sub_data()
		self.toStm = stm_send()
				
	def com_callback(self, command):
			
		self.toStm.depthUp = command.sub_up
		self.toStm.depthDown = command.sub_down
		self.toStm.forwardThrust = command.left_toggle_ud
		self.toStm.turnThrust = command.left_toggle_lr
		self.toStm.camUpDown = command.right_toggle_ud
		self.toStm.camLeftRight = command.right_toggle_lr
		
		motion.measure(toController, toStm)
		toStm.handshake(toController)
		
		data_msg = toController.package_data()
		self.data_pub.pub(data_msg)
		
		

def main(args=None):
	rclpy.init(args=args)
	sub_obj = Submarine()
	rclpy.spin(sub_obj)
	rclpy.shutdown()
		
if __name__ == '__main__':
	main()
