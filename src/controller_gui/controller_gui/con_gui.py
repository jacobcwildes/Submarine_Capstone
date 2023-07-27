#Import ROS2 libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import message_filters
from message_filters import Subscriber

#Import OpenCV libraries
import cv2 as cv
from cv_bridge import CvBridge
import numpy as np

#Import GUI
from tkinter import *
from tkinter import ttk
import PIL.Image
import PIL.ImageTk

class GUI(Node):

    def __init__(self):
        super().__init__('controller_gui')
        
        #Make an object that holds subscription info for the cam
        self.cam_sub = self.create_subscription(Image, "camera/image", self.cam_callback, 10)
        
        #Data subscription will go here
        
        
        #Make the object that will convert a ROS2 image message to
        #an OpenCV image format
        self.Bridge = CvBridge()
        
        #Tkinter (GUI)
        self.root = None
        self.panelA = None
        
    #This will eventually be time synchronized with incoming sub metrics
    def cam_callback(self, cam_sub): #Data will be passed here too
        #Convert from ROS2 message to OpenCV image format
        convert_image = self.Bridge.imgmsg_to_cv2(cam_sub)
        
        #OpenCV stores colors in BGR. PIL uses RGB - need to convert
        RGB_img = cv.cvtColor(convert_image, cv.COLOR_BGR2RGB)
        
        #Convert images to PIL format
        PIL_img = Image.fromarray(RGB_img)
        
        #Convert from PIL to ImageTk format (what actually gets displayed)
        tk_img = ImageTk.PhotoImage(PIL_img)
        
    #Work to display will be shown here
    def tk(self, img):
        #Init panels at start
        if self.panelA is None:
            self.panelA.Label(image=img)
            self.panelA.image = img
            self.panelA.pack(expand=True)
        
        #If already intitialized, update
        else:
            self.panelA.configure(image=img)
            self.panelA.image = img
            
            
        self.root = Tk()
        
        #Start the GUI
        self.root.update()
        
def main(args=None):
    rclpy.init(args=args)
    gui_obj = GUI()
    rclpy.spin(gui_obj)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
