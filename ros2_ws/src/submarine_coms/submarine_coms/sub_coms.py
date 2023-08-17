import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import message_filters
from message_filters import Subscriber
import string

import numpy as np


class Submarine(Node):

    def __init__(self):
        super().__init__('submarine_coms')
        
        #Command sub (UDP)
        self.com_sub = Subscriber(self, Image, "camera/image")        
        self.subscription = self.create_subscription(Image, 'camera/image', self.com_callback, 10)
        self.subscription
        #Data publisher (UDP)
        self.data_pub = self.create_publisher(String, 'topic', 10)
        self.timer - self.create_timer(0.5, self.data_callback)
        
    #This will eventually be time synchronized with incoming sub metrics
    def com_callback(self, com_sub): #Data will be passed here too
        #Convert from ROS2 message to OpenCV image format
        print()

    def data_callback(self):
        msg = String()
        msg.data = "hello\n"
        self.data_pub.publish(msg)
        self.get_logger().info('Publishing: ' + msg.data)


def main(args=None):
    rclpy.init(args=args)
    sub_obj = Submarine()
    rclpy.spin(sub_obj)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()