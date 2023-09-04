import rclpy
from rclpy.node import Node
import message_filters
from message_filters import Subscriber
import string
from com_interfaces.msg import ComInfo
from com_interfaces.msg import DataInfo

import serial
import time
import numpy as np


class Submarine(Node):

    def __init__(self):
        super().__init__('submarine_coms')
        
        #Command sub (UDP)       
        self.subscription = self.create_subscription(ComInfo, 'com_info', self.com_callback, 10)
        self.subscription
        #Data publisher (UDP)
        self.data_pub = self.create_publisher(DataInfo, 'data_info', 10)
        self.timer = self.create_timer(0.5, self.data_callback)
        
        self.serialport = serial.Serial('/dev/ttyACM0')
        self.serialport.baudrate = 115200  # set Baud rate to 115200
        self.serialport.bytesize = 8   # Number of data bits = 8
        self.serialport.parity  ='N'   # No parity
        self.serialport.stopbits = 1   # Number of Stop bits = 1
        
        self.dataReceived = False

        #COMS
        self.depthUp = None
        self.depthDown = None
        self.captureImage = None
        self.forwardThrust = None
        self.turnThrust = None
        self.camUpDown = None
        self.camLeftRight = None
        self.forwardBinary = None
        self.turnBinary = None
        self.camUpDownBinary = None
        self.camLeftRightBinary = None
        self.bitfield = None

        #DATA
        self.degreesNorth = None
        self.speedScalar = None
        self.depthApprox = None
        self.roll = None
        self.pitch = None
        self.yaw = None
        self.voltageBattery = None


    def data_callback(self):
        if self.dataReceived:
            msg = DataInfo()
            msg.degrees_north = int(self.degreesNorth)
            msg.speed_scalar = float(self.speedScalar)
            msg.depth_approx = int(self.depthApprox)
            msg.roll = int(self.roll)
            msg.pitch = int(self.pitch)
            msg.yaw = int(self.yaw)
            msg.voltage_battery = float(self.voltageBattery)

            self.data_pub.publish(msg)
            self.get_logger().info('Publishing: ' + msg.data)
        
    def com_callback(self, command):
        self.depthUp = command.sub_up
        self.depthDown = command.sub_down
        self.captureImage = command.screenshot
        self.forwardThrust = command.left_toggle_ud
        self.turnThrust = command.left_toggle_lr
        self.camUpDown = command.right_toggle_ud
        self.camLeftRight = command.right_toggle_lr    

        self.forwardBinary = bin(self.forwardThrust).split('b')[1]
        self.turnBinary = bin(self.turnThrust).split('b')[1]
        self.camUpDownBinary = bin(self.camUpDown).split('b')[1]
        self.camLeftRightBinary = bin(self.camLeftRight).split('b')[1]
				
        self.bitfield = str(self.depthUp) + str(self.depthDown) + str(self.captureImage) + str(self.forwardBinary).zfill(8) + str(self.turnBinary).zfill(8) + str(self.camUpDownBinary).zfill(8) + str(self.camLeftRightBinary).zfill(8)

        print(str(self.depthUp))
        print(str(self.depthDown))
        print(str(self.captureImage))
        print(self.forwardThrust)
        print(self.turnThrust)
        print(self.camUpDown)
        print(self.camLeftRight)

        self.serialport.write(self.bitfield.encode()) #35 bits
        print("Sending: " + self.bitfield)
        received = self.serialport.readline().decode('ascii').strip().strip('\x00')
        print("Received: " + received)

        received_split = received.split(',')
        
        print("DegreesNorth: " + received_split[0])
        print("SpeedScalar: " + str(int(received_split[1])/10))
        print("DepthApprox: " + received_split[2])
        print("Roll: " + received_split[3])
        print("Pitch: " + received_split[4])
        print("Yaw: " + received_split[5])
        print("Voltage: " + str(int(received_split[6])/10))
        print("---------------------------------------")
        print("")

        self.degreesNorth = int(received_split[0])
        self.speedScalar = float(received_split[1])/10.0
        self.depthApprox = int(received_split[2])
        self.roll = int(received_split[3])
        self.pitch = int(received_split[4])
        self.yaw = int(received_split[5])
        self.voltageBattery = float(received_split[6])/10.0
        
        dataReceived = True

def main(args=None):
    rclpy.init(args=args)
    sub_obj = Submarine()
    rclpy.spin(sub_obj)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
