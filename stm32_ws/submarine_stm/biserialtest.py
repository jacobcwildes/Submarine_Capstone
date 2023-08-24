#Test to send bitfield of commands and recieve dataInfo

import serial
import time

SerialObj = serial.Serial('/dev/ttyACM0')
SerialObj.baudrate = 115200  # set Baud rate to 115200
SerialObj.bytesize = 8   # Number of data bits = 8
SerialObj.parity  ='N'   # No parity
SerialObj.stopbits = 1   # Number of Stop bits = 1

#Data that will come from controller
depthUp = 0
depthDown = 0
captureImage = 0
forwardThrust = 128
turnThrust = 128
camUpDown = 128
camLeftRight = 128

forwardBinary = bin(forwardThrust).split('b')[1]
turnBinary = bin(turnThrust).split('b')[1]
camUpDownBinary = bin(camUpDown).split('b')[1]
camLeftRightBinary = bin(camLeftRight).split('b')[1]

bitfield = str(depthUp) + str(depthDown) + str(captureImage) + str(forwardBinary) + str(turnBinary) + str(camUpDownBinary) + str(camLeftRightBinary)



while (True):
	
	SerialObj.write(bitfield.encode()) #35 bits
	print("Sending: " + bitfield)
	received = SerialObj.readline().decode('ascii').strip().strip('\x00')
	print("Received: " + received)
	
	received_split = received.split(',')
	
	print("DegreesNorth: " + received_split[0])
	print("SpeedScalar: " + received_split[1])
	print("DepthApprox: " + received_split[2])
	print("Roll: " + received_split[3])
	print("Pitch: " + received_split[4])
	print("Yaw: " + received_split[5])
	print("Voltage: " + received_split[6])
	print("---------------------------------------")
	print("")
	
	time.sleep(2)
	
