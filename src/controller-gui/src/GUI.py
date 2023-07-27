#Import ROS2 libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

#Import OpenCV libraries
from cv_bridge import CvBridge
import numpy as np
import cv2 as cv

#Import GUI
from tkinter import *
from tkinter import ttk
from PIL import Image
from PIL import ImageTks

class GUI(Node):

    def __init__(self):
        super().__init__('controller-gui')
        
        #Make the object that will convert a ROS2 image message to
        #an OpenCV image format
        self.Bridge = CvBridge()
        
        #Make an object that holds subscription info for the cam
        self.cam_sub = Subscriber(self, Image, "camera/image")
        
    #This will eventually be time synchronized with incoming sub metrics
    def cam_callback(self, cam_sub):
        #Convert from ROS2 message to OpenCV image format
        convert_image = self.Bridge.imgmsg_to_cv2(cam_sub)
        
        #OpenCV stores colors in BGR. PIL uses RGB - need to convert
        RGB_img = cv.cvtColor(convert_image, cv.COLOR_BGR2RGB)
        
        #Convert images to PIL format
        PIL_img = Image.fromarray(RGB_img)
        
        #Convert from PIL to ImageTk format (what actually gets displayed)
        tk_img = ImageTk.PhotoImage(PIL_img)
        
        
