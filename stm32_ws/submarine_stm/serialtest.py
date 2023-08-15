import serial

SerialObj = serial.Serial('/dev/ttyACM0')
SerialObj.baudrate = 9600  # set Baud rate to 9600
SerialObj.bytesize = 8   # Number of data bits = 8
SerialObj.parity  ='N'   # No parity
SerialObj.stopbits = 1   # Number of Stop bits = 1


while (True):
	recieved = SerialObj.readline()
	print(recieved)
