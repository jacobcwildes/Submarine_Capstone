import serial
import time

SerialObj = serial.Serial('/dev/ttyACM0')
SerialObj.baudrate = 115200  # set Baud rate to 115200
SerialObj.bytesize = 8   # Number of data bits = 8
SerialObj.parity  ='N'   # No parity
SerialObj.stopbits = 1   # Number of Stop bits = 1


while (True):
	#print("Enter Command: ")
	#command = input()
	com = "1"
	SerialObj.write(com.encode())
	time.sleep(2)
