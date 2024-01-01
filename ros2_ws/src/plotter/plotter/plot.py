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
        
        