'''
Dyllon Dunton
12/11/2023

This handles IMU polling and communication between the controller and submarine,
as well as the information between the submarine pi and the submarine stm board
'''

import time
import math
import sys

import board
import warnings
import numpy as np
from numpy.linalg import norm
import numbers

import serial

from adafruit_lsm6ds.lsm6ds3 import LSM6DS3 as LSM6DS
from adafruit_lis3mdl import LIS3MDL

import rclpy
from rclpy.node import Node
import message_filters
from message_filters import Subscriber
import string
from com_interfaces.msg import ComInfo
from com_interfaces.msg import DataInfo
from rclpy import qos

sys.path.insert(0, '/home/ubuntu/Submarine_Capstone/ros2_ws/src/submarine_coms/submarine_coms')
from madgwickahrs import MadgwickAHRS

class SubData():
	'''
	Object to hold information going to the controller
	'''
	def __init__(self):
		self.degrees_north = 0
		self.forward_accel = 0
		self.upward_accel = 0
		self.battery_voltages = [0*x for x in range(10)]
		self.battery_voltage = 0
		self.ballast_right = 0
		self.ballast_left = 0
		self.error = ""

	def package_data(self):
		'''
		Compile all the information into a '/data_info' message to be sent to 
		the controller
		'''
		data = DataInfo()
		data.degrees_north = self.degrees_north
		data.forward = self.forward_accel
		data.upward = self.upward_accel
		data.voltage_battery = float(self.battery_voltage)
		data.ballast_left = self.ballast_right
		data.ballast_right = self.ballast_left
		data.error = self.error

		return data


class StmSend():
	'''
	This Class handles serial communication to the STM board. It compiles data from 
	the controller and sends it over to the STM board. It then waits for the response
	and then puts the data into the object going back to the controller
	'''
	def __init__(self):

		self.serial_obj = serial.Serial('/dev/ttyACM0')
		self.serial_obj.baudrate = 115200  # set Baud rate to 115200
		self.serial_obj.bytesize = 8   # Number of data bits = 8
		self.serial_obj.parity  ='N'   # No parity
		self.serial_obj.stopbits = 1   # Number of Stop bits = 1

		self.depth_up = 0
		self.depth_down = 0
		self.forward_thrust = 10
		self.turn_thrust = 0
		self.cam_up_down = 0
		self.cam_left_right = 0
		self.roll = None

	def handshake(self, con):
		'''
		This function, after checking if IMU data has come in, will compile the data for the 
		stm board and send it over, then store the response in the object going back to 
		the controller
		'''
		if self.roll is not None:
			print(self.roll)
			print("-----------------------------")
			try:
				#Format the commands into binary
				forward_binary = bin(self.forward_thrust).split('b')[1]
				turn_binary = bin(self.turn_thrust).split('b')[1]
				cam_up_down_binary = bin(self.cam_up_down).split('b')[1]
				cam_left_right_binary = bin(self.cam_left_right).split('b')[1]
				roll_binary = bin(self.roll).split('b')[1]

				#string it all together to send
				bitfield = f"""{str(self.depth_up)}{str(self.depth_down)}
				{str(forward_binary).zfill(8)}
				{str(turn_binary).zfill(8)}
				{str(cam_up_down_binary).zfill(8)}
				{str(cam_left_right_binary).zfill(8)}
				{str(roll_binary).zfill(8)}"""

				#Send the information over serial to the STM board
				self.serial_obj.write(bitfield.encode()) #42 bits
				print("Sending: " + bitfield)

				#Receive the information back from the STM board
				received = str(self.serial_obj.readline().decode('ascii'))
				received = received.replace('\r','').replace('\n','').replace('\x00', '')
				print(received)
				received_split = received.split(',')

				#If didnt get the information correctly, raise Exception
				if len(received_split) != 8: raise Exception
				print("Received: " + str(received_split))

				#Parse information into the object to go to the Controller
				con.ballast_right = int(received_split[1])
				con.ballast_left = int(received_split[2])
				battery_voltage = (int(received_split[0])/10.0)
				for i in range(1,len(con.batteryVoltages)-1):
					con.batteryVoltages[i] = con.batteryVoltages[i+1]
				con.batteryVoltages[-1] = battery_voltage
				con.battery_voltage = float(sum(con.batteryVoltages)/len(con.batteryVoltages))
				errors = []
				if not (int(received_split[3])):
					errors.append("Left Fault | ")
				if not (int(received_split[4])):
					errors.append("Prop Fault | ")
				if not (int(received_split[5])):
					errors.append("Right Fault |")
				errors.append(f"Roll: {self.roll} | ")
				errors.append(f"Volt: {con.battery_voltage:.1f} | ")
				errors.append(f"Left: {int(received_split[6])} | ")
				errors.append(f"Right: {int(received_split[7])} | ")
				if (self.roll < 45 or self.roll > 135):
					errors.append("TOO MUCH ROLL!")
				con.error = ','.join(errors)

			except Exception as error:
				print(f"BAD DATA: {error}")

class IMU():
	'''
	This Class handles all IMU polling for measurements. It also connects to the 
	madgwickAHRS module to get the relevant information (roll, x_accel, z_accel, heading)
	'''
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

		for _ in range(self.bias_checks):
			accel = self.accel_gyro.acceleration
			x_accel = accel[0]
			z_accel = accel[2]
			self.x_sum += x_accel
			self.z_sum += z_accel
			time.sleep(0.02)
		self.x_bias = self.x_sum / (self.bias_checks)
		self.z_bias = self.z_sum / (self.bias_checks)

	def measure(self, con, sub):
		'''
		This method handles IMU measurements and fuses the data together using the 
		MadgwickAHRS algorithm module to get roll, heading, x_accel, and z_accel 
		and store them in the object going to the controller
		'''
		roll = 90
		north = 0
		x_accel = 0
		z_accel = 0
		try:
			accel = self.accel_gyro.acceleration
			gyro = self.accel_gyro.gyro
			magnet = self.mag.magnetic
			y_mag = magnet[1]
			x_mag = magnet[0]
			x_accel = accel[0] - self.x_bias
			z_accel = -accel[2] + self.z_bias

			self.imu_fusion.update_imu(gyro, accel)

			roll, _, _ = self.imu_fusion.quaternion.to_euler123()
		except Exception as error:
			print(f"IMU read error: {error}")

		roll = int(roll*(180/math.pi))
		if abs(roll) > 90:
			roll *= (90/abs(roll)) # -90 <-> 90
		sub.roll = roll + 90 # 0 <-> 180

		north = math.atan2(y_mag,x_mag)*(180/math.pi)
		if north < 0:
			north += 360
		north = north - 30
		if north < 0:
			north += 360

		con.degrees_north = int(north)
		con.forward_accel = x_accel
		con.upward_accel = z_accel


class Submarine(Node):
	'''
	This is the ROS2 node that handles communication with the controller over the ethernet cable
	This subscribes to '/com_info' messages with the command. This data will then be parsed into 
	a StmSend() object. Next the IMU object polls the IMU + stores the information to be sent both
	to the stm board for active leveling and the controller for viewing. Then, the handshake method
	runs which gives the command and data to the stm board, then waits to receive the sensor data.
	This is stored in the SubData() object. The publisher will then compile the information in the 
	SubData() object and create a '/data_info' message to send back to the controller
	'''
	def __init__(self):
		super().__init__('submarine_coms')

		#Command sub (UDP)
		self.subscription = self.create_subscription(ComInfo, 'com_info', self.com_callback, 10)
		self.subscription
		#Data publisher (UDP)
		self.data_pub = self.create_publisher(DataInfo, 'data_info', 10)

		self.motion = IMU()
		self.to_controller = SubData()
		self.to_stm = StmSend()

	def com_callback(self, command):
		'''
		This method parses the command from the controller, then gets IMU information, 
		sends the command to the stm board, receives sensor data, then sends the data 
		back up to the controller
		'''
		self.toStm.depth_up = command.sub_up
		self.toStm.depth_down = command.sub_down
		self.toStm.forward_thrust = command.left_toggle_ud
		self.toStm.turn_thrust = command.left_toggle_lr
		self.toStm.cam_up_down = command.right_toggle_ud
		self.toStm.cam_left_right = command.right_toggle_lr

		self.motion.measure(self.to_controller, self.to_stm)
		self.toStm.handshake(self.to_controller)

		data_msg = self.toController.package_data()
		self.data_pub.publish(data_msg)



def main(args=None):
	'''
	Spin up the ROS2 node
	'''
	rclpy.init(args=args)
	sub_obj = Submarine()
	rclpy.spin(sub_obj)
	rclpy.shutdown()

if __name__ == '__main__':
	main()
