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
        timer_period = .5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        #Don't want to have tons of random numbers all the time, this lets me 
        #control how often random generation occurs
        self.i = 0
        self.forward = 0
        self.left_right = 0
        self.up = 0
        self.down = 0
        self.tiebreaker = None

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
            #128 is the "0" of the controller. Anything less is a reverse
            #For now I want to only go forward. I want to have a maximum of 255,
            #and be able to reach any integer between those numbers
            #self.forward = random.randrange(128, 255, 28)
            self.forward = random.randint(0, 1)

        if self.i%1 == 0:
             #128 is the "0" of the controller. Anything less is a reverse
            #For now I want to only go forward. I want to have a maximum of 255,
            #and be able to reach any integer between those numbers
            #self.left_right = random.randrange(0, 255, 12)
            self.left_right = random.randint(0, 2)
        
        if self.i%1 == 0:
            #up and down are binaries, so only want to be a zero or 1
            self.up = random.randint(0, 1)
            self.down = random.randint(0, 1)

            if self.up == self.down:
                self.tiebreaker = random.randint(0, 1)

                if self.tiebreaker == 0:
                    self.up = 1
                    self.down = 0
                else:
                    self.up = 0
                    self.down = 1

        msg = ComInfo()
        msg.left_toggle_ud = self.forward
        msg.left_toggle_lr = self.left_right
        msg.sub_up = self.up
        msg.sub_down = self.down
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