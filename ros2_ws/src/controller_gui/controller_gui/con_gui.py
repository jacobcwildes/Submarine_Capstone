#Import ROS2 libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import message_filters
from message_filters import Subscriber, TimeSynchronizer

#Import custom message type
#Note that DataInfo needed a header for ROS2 to add a timestamp to. Without it, the 
#time synchro fails because it can never retrieve a time
from com_interfaces.msg import DataInfo

#Import OpenCV libraries
import cv2 as cv
from cv_bridge import CvBridge
import numpy as np

#Import GUI
import tkinter as tk
import PIL.Image
import PIL.ImageTk

#Import time to track longetivity of the program
import time
from datetime import datetime

#In my experience importing a module from another file can be... finnicky. This
#solution has worked every time for me
import sys
import os
sys.path.insert(0, 'src/controller_gui/controller_gui')
from overlay import overlay

class GUI(Node):

    def __init__(self):
        #Track the start of the program
        self.prog_start = datetime.now()
        super().__init__('controller_gui')
        
        #Make an object that holds subscription info for the cam
        #self.cam_sub = Subscriber(self, Image, "camera/image")
        #cache = message_filters.Cache(self.cam_sub, 10)
        #cache.registerCallback(self.cam_callback)
        
        self.cam_sub = Subscriber(self, Image, 'camera/image')

        #Data subscription will go here
        #Initially going to subscribe to a dummy node so that I can
        #test the GUI without all the other nodes
        self.data_sub = Subscriber(self, DataInfo, 'dummy_data')
        
        #Synchronize the two data streams to prevent screen tearing
        queue_size = 10
        self.ts = TimeSynchronizer([self.cam_sub, self.data_sub], queue_size)
        self.ts.registerCallback(self.cam_callback)
        
        #Previous time (for FPS calc)
        self.previous_time = 0
        
        #Dead voltage of the battery. Going to use the difference on the current
        #charge and dead charge.
        self.battery_dead = 12
        
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
    def cam_callback(self, cam_sub, data_sub): #Data will be passed here too
        
        #Using monotonic time because I don't really care about real timezones. The 
        #monotonic clock naively ticks up - perfect for what I want
        current_time = time.monotonic()
        
        #Convert from ROS2 message to OpenCV image format
        convert_image = self.Bridge.imgmsg_to_cv2(cam_sub)
        
        #OpenCV stores colors in BGR. PIL uses RGB - need to convert
        RGB_img = cv.cvtColor(convert_image, cv.COLOR_BGR2RGB)
        
        #Although not exactly current at the time of display, close enough for our purposes
        prog_current = datetime.now()
        prog_time = prog_current - self.prog_start
        current_voltage = data_sub.voltage_battery - self.battery_dead
        #Overlay data onto the image
        RGB_img = overlay(RGB_img, data_sub.speed_scalar, current_voltage,
                            data_sub.depth_approx, data_sub.degrees_north, prog_time)
        
        #Make frame per second count. Isn't perfect, but it is a degree of inaccuracy I am 
        #willing to absorb since it is not in our favor
        difference = current_time - self.previous_time
        FPS = 1/difference
        self.previous_time = current_time
        cv.putText(RGB_img, str(int(FPS)), (2, 15), cv.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1, cv.LINE_AA)
        
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
