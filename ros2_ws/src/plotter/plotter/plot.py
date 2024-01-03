import rclpy
from rclpy.node import Node

import numpy as np
from matplotlib import pyplot as plt

#Import the custom message type for publishing
from com_interfaces.msg import ComInfo

class plotter(Node):
    def __init__(self):
        super().__init__('plotter')
        self.path_sub = self.create_subscription(ComInfo, '/random_data',
                                                 self.path_callback, 10)
        self.counter = 0
        #Create a NumPy array that contains 1000 indices of length 3 tuples
        self.array = np.zeros((1000, 3), dtype=int)

    def path_callback(self, path_sub):
        #print(self.array)
        #print(self.array.shape)
        
        #Want 1000 indices
        if self.counter != 1000:
            print("Filling array: %d", self.counter)
            self.array[self.counter] = [path_sub.left_toggle_ud, path_sub.left_toggle_lr, path_sub.sub_up]
            self.counter += 1
        else:
            print(self.array)
        


def main(args=None):
    '''
    Spin up the ROS node to subscribe
    '''
    rclpy.init(args=args)
    plot = plotter()
    rclpy.spin(plot)
    rclpy.shutdown()


if __name__ == '__main__':
    main()