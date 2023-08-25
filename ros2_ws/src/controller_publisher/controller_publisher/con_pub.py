#ROS2 Necessities
import rclpy
from rclpy.node import Node

#Import custom message type
from com_interfaces.msg import ComInfo

import serial

class ControllerOutput(Node):
    
    def __init__(self):
        #Command publication
        super().__init__('controller_publisher')
        #Make a publisher - first arg is custom defined message, second
        #is topic name, third is how many things to queue to publish
        self.com_pub = self.create_publisher(ComInfo, 'com_info', 10)
        
        #Serial read setup
        self.serialport = serial.Serial("/dev/ttyACM0", 115200, timeout=0.5)
        self.serialLine = None
        
        #Publish data every thousandth of a second
        timer_period = 0.001
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):            
        #Read serial data and save to variables
        self.serialLine = self.serialport.readline()
        self.serialLine = self.serialLine.decode('ascii')
        localDat = self.serialLine.strip().strip('\x00')
        localDat = localDat.split(',')
        #print(localDat)

        try: 
            if len(localDat) == 7:
                #Set message to custom datatype
                msg = ComInfo()
               
                #Put data in each field of datatype
                msg.left_toggle_ud = int(localDat[0])
                msg.left_toggle_lr = int(localDat[1])
                msg.right_toggle_ud = int(localDat[2])
                msg.right_toggle_lr = int(localDat[3])
                msg.sub_up = int(localDat[4])
                msg.sub_down = int(localDat[5])
                msg.screenshot = int(localDat[6])
                
                self.com_pub.publish(msg)
                self.get_logger().info('Publishing Left Toggle UD: "%d' % msg.left_toggle_ud) 
                self.get_logger().info('Publishing Left Toggle LR: "%d' % msg.left_toggle_lr) 
                self.get_logger().info('Publishing Right Toggle UD: "%d' % msg.right_toggle_ud) 
                self.get_logger().info('Publishing Right Toggle LR: "%d' % msg.right_toggle_lr) 
                self.get_logger().info('Publishing SubUp: "%d' % msg.sub_up)
                self.get_logger().info('Publishing SubDown: "%d' % msg.sub_down)  
                self.get_logger().info('Publishing Screenshot: "%d' % msg.screenshot) 
                
        except ValueError:
            pass

def main(args=None):
    rclpy.init(args=args)
    con_obj = ControllerOutput()
    rclpy.spin(con_obj)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()   
