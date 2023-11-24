#Import ROS2 libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import message_filters
from message_filters import Subscriber, TimeSynchronizer
from rclpy import qos

#Import custom message type
#Note that DataInfo needed a header for ROS2 to add a timestamp to. Without it, the 
#time synchro fails because it can never retrieve a time
from com_interfaces.msg import DataInfo
from com_interfaces.msg import ComInfo

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
sys.path.insert(0, '/home/controller/Submarine_Capstone/ros2_ws/src/controller_gui/controller_gui/')
from overlay import overlay

import RPi.GPIO as gpio

class GUI(Node):

    def __init__(self):
        #Track the start of the program
        self.prog_start = datetime.now()
        super().__init__('controller_gui')
        
        #Make an object that holds subscription info for the cam (UDP) 
        self.cam_sub = self.create_subscription(Image, 'camera/image', self.cam_callback, 10)#qos.qos_profile_sensor_data)

        #Data subscription will go here (UDP)
        self.data_sub = self.create_subscription(DataInfo, 'data_info', self.data_callback, 10)#qos.qos_profile_sensor_data)
        
        #Command subscription goes here (for screenshots) UDP
        self.screenshot_sub = self.create_subscription(ComInfo, 'com_info', self.screenshot_callback, 10)#qos.qos_profile_sensor_data)
        
        #Previous time (for FPS calc)
        self.previous_time = 0
        
        #Make the object that will convert a ROS2 image message to
        #an OpenCV image format
        self.Bridge = CvBridge()
        
        #Tkinter (GUI)
        self.root = tk.Tk()
        self.root.title("Submarine View")
        
        #Force the environment variable to override if necessary
        if os.printenviron.get('DISPLAY', '') == '':
            os.environ.__setitem__('DISPLAY', ':0.0')
        
        #Make window fullscreen
        self.root.attributes('-fullscreen', True)
        
        self.panelA = None
        
        #Variables to hold data information
        self.batteryVoltage = 12
        self.ballastLeft = 0
        self.ballastRight = 0
        self.errMess = " "
        self.heading = 0
        self.depth = 0
        self.speed = 0
        
        #Screenshot?
        self.screenshot = 0
        
        #Set up FPS measuring pin
        gpio.setmode(gpio.BCM)
        gpio.setup(24, gpio.OUT)
        
    #This will eventually be time synchronized with incoming sub metrics
    def cam_callback(self, cam_sub): #Data will be passed here too
        
        #Toggle pin on
        gpio.output(24, 1)
        
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

        #Overlay data onto the image
        RGB_img = overlay(RGB_img, self.speed, self.batteryVoltage, self.ballastLeft,
                          self.ballastRight, self.depth, self.heading, prog_time)
        
        #Make frame per second count. Isn't perfect, but it is a degree of inaccuracy I am 
        #willing to absorb since it is not in our favor
        difference = current_time - self.previous_time
        FPS = 1/difference
        self.previous_time = current_time
        cv.putText(RGB_img, str(int(FPS)), (2, 15), cv.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1, cv.LINE_AA)
        
        
        #Current Program time
        prog_time = datetime.now() - self.prog_start
        
        #Remove microseconds from time delta
        prog_time = str(prog_time).split(".")[0]
        
        #Draw uptime counter
        cv.rectangle(RGB_img, (920, 560), (1024, 600), (255, 255, 255), -1)
        cv.rectangle(RGB_img, (920, 560), (1024, 600), (0, 0, 0), 2)
        cv.putText(RGB_img, str(prog_time), (930, 585), cv.FONT_HERSHEY_SIMPLEX, .65, (0, 0, 0), 1, cv.LINE_AA)
        
        #Draw Error Messages
        cv.rectangle(RGB_img, (0, 560), (900, 600), (255, 255, 255), -1)
        cv.rectangle(RGB_img, (0, 560), (900, 600), (0, 0, 0), 2)
        cv.putText(RGB_img, str(self.errMess), (25, 590), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 1, cv.LINE_AA)
        
        #Convert images to PIL format
        PIL_img = PIL.Image.fromarray(RGB_img)
        
        #Convert from PIL to ImageTk format (what actually gets displayed)
        tk_img = PIL.ImageTk.PhotoImage(PIL_img)
        
        #Save image?
        if self.screenshot:
            path = str(prog_time) + '.jpg'#'/mnt/usb/images/' + str(prog_time) +  '.jpg'
            PIL_img.save(path)

        
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
        
        #Toggle pin off
        gpio.output(24, 0)
     
    def data_callback(self, data):
        self.heading = data.degrees_north
        self.batteryVoltage = data.voltage_battery
        self.depth = data.upward
        self.errMess = data.error
        self.ballastLeft = data.ballast_left
        self.ballastRight = data.ballast_right
        self.speed = data.forward
        
    def screenshot_callback(self, com):
        self.screenshot = com.screenshot
    
        
  
def main(args=None):
    rclpy.init(args=args)
    gui_obj = GUI()
    rclpy.spin(gui_obj)
    rclpy.shutdown()
    #Shut down pin grab
    gpio.cleanup()
    
if __name__ == '__main__':
    main()
