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

print("---------------------------------------")
print("")

while (True):
	
	try:
		SerialObj.write(bitfield.encode()) #35 bits
		print("Sending: " + bitfield)
		received = SerialObj.readline().decode('ascii').strip().strip('\x00')
		print("Received: " + received)
		
		received_split = received.split(',')
		
		
		print("ADCS:")
		print("Left Position: " + str(int(received_split[0])/10))
		print("Right Position: " + str(int(received_split[1])/10))
		print("Battery Voltage: " + str(int(received_split[2])/10))
		
		
		print("\nFAULTS")
		print("Left Fault: " + received_split[3])
		print("Right Fault: " + received_split[4])
		print("Propellor Fault: " + received_split[5])
		
		print("\nIMU VALUES")
		print("Ang X: " + received_split[6])
		print("Ang Y: " + received_split[7])
		print("Ang Z: " + received_split[8])
		print("Accel X: " + received_split[9])
		print("Accel Y: " + received_split[10])
		print("Accel Z: " + received_split[11])
		print("Magnet X: " + received_split[12])
		print("Magnet Y: " + received_split[13])
		print("Magnet Z: " + received_split[14])
		
		print("\nIMU POSITIONS")
		print("Roll: " + received_split[15])
		print("Pitch: " + received_split[16])
		print("Yaw: " + received_split[17])
		print("XPos: " + received_split[18])
		print("YPos: " + received_split[19])
		
		print("\nIMU VALUES")
		print("Speed Scalar: " + received_split[20])
		print("Degrees North: " + received_split[21])
		print("Approximate Depth: " + received_split[22])
		
		print("\nPLANNER VALUES")
		print("Depth Up: " + received_split[23])
		print("Depth Down: " + received_split[24])
		print("Capture Image: " + received_split[25])
		print("Forward Thrust: " + received_split[26])
		print("Turn Thrust: " + received_split[27])
		print("Cam Horizontal: " + received_split[28])
		print("Cam Vertical: " + received_split[29])
		
		print("\nSTEPPER VALUES")
		print("LEFT A1: " + received_split[30])
		print("LEFT A2: " + received_split[31])
		print("LEFT B1: " + received_split[32])
		print("LEFT B2: " + received_split[33])
		print("RIGHT A1: " + received_split[34])
		print("RIGHT A2: " + received_split[35])
		print("RIGHT B1: " + received_split[36])
		print("RIGHT B2: " + received_split[37])
		
		print("\nTHRUST VALUES")
		print("Left thrust: " + received_split[38])
		print("Right thrust: " + received_split[39])
		
	except:
		print("UH OH")
	
	print("---------------------------------------")
	print("")
	
	time.sleep(2)
	
