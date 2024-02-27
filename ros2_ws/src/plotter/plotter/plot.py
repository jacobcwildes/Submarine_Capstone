import rclpy
from rclpy.node import Node

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits import mplot3d

#Import the custom message type for publishing
from com_interfaces.msg import ComInfo

class plotter(Node):
    def __init__(self):
        super().__init__('plotter')
        self.path_sub = self.create_subscription(ComInfo, '/random_data',
                                                 self.path_callback, 0)
        self.counter = 0
        #Create a NumPy array that contains 1000 indices of length 3 tuples
        self.array = np.zeros((1000, 3), dtype=int)
        self.once_flag = 0
        self.convolved_array = np.zeros((1000, 3), dtype=int)
        self.mover_flag = 0

    def path_callback(self, path_sub):
        #print(self.array)
        #print(self.array.shape)
        
        #Want 1000 indices
        if self.counter < 1000:
            print("Filling array: %d", self.counter)
            self.array[self.counter] = [path_sub.left_toggle_ud, path_sub.left_toggle_lr, path_sub.sub_up]
            print(f"Current array index: {self.counter}, value: {self.array[self.counter]}")    
            self.counter += 1
        if self.once_flag == 1 and self.mover_flag == 0:
            #print(self.array)
            for r in range(1000):
                if r == 0:
                    self.convolved_array[r] = self.array[r]
                    #print(self.convolved_array[r])
                else:
                    self.convolved_array[r] += self.convolved_array[r - 1] + self.array[r] + self.array[r - 1]   
                    print(self.convolved_array[r])

            self.mover_flag = 1

            #Plot the traversal
            fig = plt.figure()
            ax = plt.axes(projection="3d")
            graph = ax.plot3D(self.convolved_array[:, 0], self.convolved_array[:, 1], self.convolved_array[:, 2], color="r")[0]
            plt.ylim(-250, 250)
            plt.xlim(0, 1000)
            plt.show()
            
        if self.counter == 1000:
            self.once_flag = 1
        


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