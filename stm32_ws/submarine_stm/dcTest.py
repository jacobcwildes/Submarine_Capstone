#Test to send coms info to stm board to test DC on pwm timers

import serial
import time
import board

from adafruit_lsm6ds.lsm6dsox import LSM6DSOX as LSM6DS
from adafruit_lis3mdl import LIS3MDL

i2c = board.I2C() 
accel_gyro = LSM6DS(i2c)
mag = LIS3MDL(i2c)

SerialObj = serial.Serial('/dev/ttyACM0')
SerialObj.baudrate = 115200  # set Baud rate to 115200
SerialObj.bytesize = 8   # Number of data bits = 8
SerialObj.parity  ='N'   # No parity
SerialObj.stopbits = 1   # Number of Stop bits = 1

#Data that will come from controller
depthUp = 0
depthDown = 0
forwardThrust = 167
turnThrust = 25
camUpDown = 0
camLeftRight = 128
pitch = 200
depth = 240

class sub_data():
	def __init__(self):
		self.degreesNorth = 0
		self.speedScalar = 0
		self.depthApprox = 0
		self.batteryVoltage = 0
		self.ballastRight = 0
		self.ballastLeft = 0
		self.error = ""
		
	def 	
	

class imu_measurement():
	def __init__(self):
		self.x_lin_a = 0
		self.y_lin_a = 0
		self.z_lin_a = 0
		self.x_ang_a = 0
		self.y_ang_a = 0
		self.z_ang_a = 0
		self.x_mag_a = 0
		self.y_mag_a = 0
		self.z_mag_a = 0
		
		self.x_lin_v = 0
		self.y_lin_v = 0
		self.z_lin_v = 0
		self.x_ang_v = 0
		self.y_ang_v = 0
		self.z_ang_v = 0
		self.x_mag_v = 0
		self.y_mag_v = 0
		self.z_mag_v = 0
		
		self.x_lin_p = 0
		self.y_lin_p = 0
		self.z_lin_p = 0
		self.x_ang_p = 0
		self.y_ang_p = 0
		self.z_ang_p = 0
		self.x_mag_p = 0
		self.y_mag_p = 0
		self.z_mag_p = 0
		
	def measure(self):
		acceleration = accel_gyro.acceleration
    gyro = accel_gyro.gyro
    magnetic = mag.magnetic
    self.x_lin_a = acceleration[0]
		self.y_lin_a = acceleration[1]
		self.z_lin_a = acceleration[2]
		self.x_ang_a = gyro[0]
		self.y_ang_a = gyro[1]
		self.z_ang_a = gyro[2]
		self.x_mag_a = magnetic[0]
		self.y_mag_a = magnetic[1]
		self.z_mag_a = magnetic[2]
		
		#NEED TO ADD IN DOUBLE INTEGRATION
		
		



delayOne = imu_measurement()
now = imu_measurement()







print("---------------------------------------")
print("")


while(True):
	depth += 1
	if depth > 255: depth = 0
	try:
		forwardBinary = bin(forwardThrust).split('b')[1]
		turnBinary = bin(turnThrust).split('b')[1]
		camUpDownBinary = bin(camUpDown).split('b')[1]
		camLeftRightBinary = bin(camLeftRight).split('b')[1]
		pitchBinary = bin(pitch).split('b')[1]
		depthBinary = bin(depth).split('b')[1]

		bitfield = str(depthUp) + str(depthDown) + str(forwardBinary).zfill(8) + str(turnBinary).zfill(8) + str(camUpDownBinary).zfill(8) + str(camLeftRightBinary).zfill(8) + str(pitchBinary).zfill(8) + str(depthBinary).zfill(8)
		
		SerialObj.write(bitfield.encode()) #50 bits
		print("Sending: " + bitfield)
		received = str(SerialObj.readline().decode('ascii')).replace('\r','').replace('\n','').replace('\x00', '')
		received_split = received.split(',')

		if len(received_split) != 8: raise Exception
		print("Received: " + str(received_split))
	
	
		'''
		print("leftThrust: " + received_split[0])
		print("rightThrust: " + received_split[1])
		print("forThrust: " + received_split[2])
		print("backThrust: " + received_split[3])
		print("leftPropThrust: " + received_split[4])
		print("rightPropThrust: " + received_split[5])
		'''
		print("---------------------------------------")
		print("")
	except:
		print("UH OH")	
		print("---------------------------------------")
		print("")
	
	time.sleep(.01)

