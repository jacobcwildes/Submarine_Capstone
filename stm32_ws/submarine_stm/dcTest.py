#Test to send coms info to stm board to test DC on pwm timers

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
forwardThrust = 167
turnThrust = 25
camUpDown = 0
camLeftRight = 128
pitch = 300
depth = 8



print("---------------------------------------")
print("")


while(True):
	depthUp += 1
	try:
		forwardBinary = bin(forwardThrust).split('b')[1]
		turnBinary = bin(turnThrust).split('b')[1]
		camUpDownBinary = bin(camUpDown).split('b')[1]
		camLeftRightBinary = bin(camLeftRight).split('b')[1]
		pitchBinary = bin(pitch).split('b')[1]
		depthBinary = bin(depth).split('b')[1]

		#print(str(forwardBinary).zfill(8))
		#print(str(turnBinary).zfill(8))
		#print(str(camUpDownBinary).zfill(8))
		#print(str(camLeftRightBinary).zfill(8))

		bitfield = str(depthUp) + str(depthDown) + str(forwardBinary).zfill(8) + str(turnBinary).zfill(8) + str(camUpDownBinary).zfill(8) + str(camLeftRightBinary).zfill(8) + str(pitchBinary).zfill(8) + str(depthBinary).zfill(8)
		
		SerialObj.write(bitfield.encode()) #35 bits
		print("Sending: " + bitfield)
		received = SerialObj.readline().decode('ascii').strip().strip('\x00')
		received_split = received.split(',')
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
	
	time.sleep(0.01)

