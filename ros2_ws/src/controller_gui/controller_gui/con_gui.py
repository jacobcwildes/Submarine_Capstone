#Import ROS2 libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import message_filters
from message_filters import Subscriber

#Import custom message type
from com_interfaces.msg import DataInfo

#Import OpenCV libraries
import cv2 as cv
from cv_bridge import CvBridge
import numpy as np

#Import GUI
import tkinter as tk
import PIL.Image
import PIL.ImageTk

import serial

class GUI(Node):

    def __init__(self):
        super().__init__('controller_gui')
        
        #Make an object that holds subscription info for the cam
        self.cam_sub = Subscriber(self, Image, "camera/image")
        cache = message_filters.Cache(self.cam_sub, 10)
        cache.registerCallback(self.cam_callback)
        
        self.subscription = self.create_subscription(Image, 'camera/image', 
                                                    self.cam_callback, 10)
        self.subscription
        
        
        #Data subscription will go here
        
        
        #Make the object that will convert a ROS2 image message to
        #an OpenCV image format
        self.Bridge = CvBridge()
        
        #Tkinter (GUI)
        self.root = tk.Tk()
        self.root.title("Submarine View")
        
        #Make window fullscreen
        self.root.attributes('-fullscreen', True)
        
        self.panelA = None
        
    #This will eventually be time synchronized with incoming sub metrics
    def cam_callback(self, cam_sub): #Data will be passed here too
        
        #Convert from ROS2 message to OpenCV image format
        convert_image = self.Bridge.imgmsg_to_cv2(cam_sub)
        
        #OpenCV stores colors in BGR. PIL uses RGB - need to convert
        RGB_img = cv.cvtColor(convert_image, cv.COLOR_BGR2RGB)
        
        #Convert images to PIL format
        PIL_img = PIL.Image.fromarray(RGB_img)
        
        #Convert from PIL to ImageTk format (what actually gets displayed)
        tk_img = PIL.ImageTk.PhotoImage(PIL_img)
        
        #Work to display will be shown here
        #Init panels at start
        if self.panelA is None:
            self.panelA = tk.Label(image=tk_img)
            self.panelA.image = tk_img
            #Tell Tkinter to fill both x and y axes. Additionally, if the 
            #window should change size, the frame will too
        
        #If already intitialized, update
        else:
            self.panelA.configure(image=tk_img)
            self.panelA.image = tk_img
            self.panelA.pack(fill=tk.BOTH, expand = True)
            
        self.root.update()
        
    
        
  
def main(args=None):
    rclpy.init(args=args)
    gui_obj = GUI()
    rclpy.spin(gui_obj)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
