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

bitfield = str(depthUp) + str(depthDown) + str(captureImage) + str(forwardBinary).zfill(8) + str(turnBinary).zfill(8) + str(camUpDownBinary).zfill(8) + str(camLeftRightBinary).zfill(8)

print("---------------------------------------")
print("")

while (True):
	
	SerialObj.write(bitfield.encode()) #35 bits
	print("Sending: " + bitfield)
	received = SerialObj.readline()
	#print("Rec: " + received)
	rec = received.decode('ascii').strip().strip('\x00')
	print("Received: " + rec)
	
	received_split = rec.split(',')
	
	print("DegreesNorth: " + received_split[0])
	print("SpeedScalar: " + str(int(received_split[1])/10))
	print("DepthApprox: " + received_split[2])
	print("Roll: " + received_split[3])
	print("Pitch: " + received_split[4])
	print("Yaw: " + received_split[5])
	print("Voltage: " + str(int(received_split[6])/10))
	print("---------------------------------------")
	print("")
	
	time.sleep(2)
	
