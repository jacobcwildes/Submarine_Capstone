import rclpy
from rclpy.node import Node

#Import the custom message type for publishing
from com_interfaces.msg import ComInfo

#Import random to scatter data
import random

class random_explore(Node):
    def __init__(self):
        super().__init__('explore')
        self.publisher = self.create_publisher(ComInfo, '/random_data', 10)
        timer_period = .0025
        self.timer = self.create_timer(timer_period, self.timer_callback)
        #Don't want to have tons of random numbers all the time, this lets me 
        #control how often random generation occurs
        self.i = 0
        self.forward = 0
        self.left = 0
        self.up = 0

    def timer_callback(self):
        '''
        These conditionals serve the purpose of controlling how often
        the random number is generated. For example, I want to go 
        forward at a certain rate for longer than I would want to turn

        In the message structure:
        left_toggle_ud = forward/backward
        left_toggle_lr = left/right
        sub_up = ascend
        sub_down = descend
        '''
        if self.i%1 == 0:
            #0 is stop, 1 is go forward
            self.forward = random.randint(0, 1)

        if self.i%1 == 0:
             #-1 turns left, 0 is forward, 1 turns right
            self.left = random.randint(-1, 1)
        
        if self.i%1 == 0:
            #-1 goes down, 0 stays level, 1 goes up
            self.up = random.randint(-1, 1)

        msg = ComInfo()
        msg.left_toggle_ud = self.forward
        msg.left_toggle_lr = self.left
        msg.sub_up = self.up
        self.publisher.publish(msg)
        self.get_logger().info("Publishing data")
        self.i += 1

def main(args=None):
    '''
    Spin up the ROS node to publish
    '''
    rclpy.init(args=args)
    pub = random_explore()
    rclpy.spin(pub)
    rclpy.shutdown()


if __name__ == '__main__':
    main()