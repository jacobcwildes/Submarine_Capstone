'''
Primary controller GUI module which takes camera frames and passes
them off to the overlay function to be modified. Those images
are then displayed
'''

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
    '''
    This class deals with making data, camera, and controller message
    subscribers. The running time is also calculated within the methods
    of this class. FPS is calculated and the controller GUI is visualized
    here
    '''
    def __init__(self):
        #Track the start of the program
        self.prog_start = datetime.now()
        super().__init__('controller_gui')

        #Make an object that holds subscription info for the cam (UDP)
        self.cam_sub = self.create_subscription(Image, 'camera/image',
                                                self.cam_callback, qos.qos_profile_sensor_data)

        #Data subscription will go here (UDP)
        self.data_sub = self.create_subscription(DataInfo, 'data_info',
                                                 self.data_callback, qos.qos_profile_sensor_data)

        #Command subscription goes here (for screenshots) UDP
        self.screenshot_sub = self.create_subscription(ComInfo, 'com_info',
                                                       self.screenshot_callback,
                                                       qos.qos_profile_sensor_data)

        #Previous time (for FPS calc)
        self.previous_time = 0

        #Make the object that will convert a ROS2 image message to
        #an OpenCV image format
        self.bridge = CvBridge()

        #Tkinter (GUI)
        self.root = tk.Tk()
        self.root.title("Submarine View")

        #Make window fullscreen
        self.root.attributes('-fullscreen', True)

        #Have window fit the screen
        #self.root.geometry("%dx%d" % (self.screen_width, self.screen_height))

        self.panel_a = None

        #Variables to hold data information
        self.battery_voltage = 12
        self.ballast_left = 0
        self.ballast_right = 0
        self.err_mess = " "
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
        '''
        This method is responsible for capturing the images which flow in from 
        the camera subscription and passing them off to be modified by the overlay
        method
        '''
        #Toggle pin on
        gpio.output(24, 1)

        #Using monotonic time because I don't really care about real timezones. The
        #monotonic clock naively ticks up - perfect for what I want
        current_time = time.monotonic()

        #Convert from ROS2 message to OpenCV image format
        convert_image = self.bridge.imgmsg_to_cv2(cam_sub)

        #OpenCV stores colors in BGR. PIL uses RGB - need to convert
        rgb_img = cv.cvtColor(convert_image, cv.COLOR_BGR2RGB)

        #Although not exactly current at the time of display, close enough for our purposes
        prog_current = datetime.now()
        prog_time = prog_current - self.prog_start

        #Overlay data onto the image
        rgb_img = overlay(rgb_img, self.speed, self.battery_voltage, 255-self.ballast_left,
                          255-self.ballast_right, self.depth, self.heading, prog_time)

        #Make frame per second count. Isn't perfect, but it is a degree of inaccuracy I am
        #willing to absorb since it is not in our favor
        difference = current_time - self.previous_time
        fps = 1/difference

        self.previous_time = current_time
        cv.putText(rgb_img, str(int(fps)), (2, 15), cv.FONT_HERSHEY_SIMPLEX, .5,
                  (255, 255, 255), 1, cv.LINE_AA)


        #Current Program time
        prog_time = datetime.now() - self.prog_start

        #Remove microseconds from time delta
        prog_time = str(prog_time).split('.', maxsplit = 1)[0]

        #Draw uptime counter
        cv.rectangle(rgb_img, (920, 560), (1024, 600), (255, 255, 255), -1)
        cv.rectangle(rgb_img, (920, 560), (1024, 600), (0, 0, 0), 2)
        cv.putText(rgb_img, str(prog_time), (930, 585), cv.FONT_HERSHEY_SIMPLEX,
                   .65, (0, 0, 0), 1, cv.LINE_AA)

        #Draw Error Messages
        cv.rectangle(rgb_img, (0, 560), (900, 600), (255, 255, 255), -1)
        cv.rectangle(rgb_img, (0, 560), (900, 600), (0, 0, 0), 2)
        cv.putText(rgb_img, str(self.err_mess), (25, 590), cv.FONT_HERSHEY_SIMPLEX,
                   1, (0, 0, 0), 1, cv.LINE_AA)

        #Convert images to PIL format
        pil_img = PIL.Image.fromarray(rgb_img)

        #Rescale image to fit the screen
        pil_img = pil_img.resize((720, 578))

        #Convert from PIL to ImageTk format (what actually gets displayed)
        tk_img = PIL.ImageTk.PhotoImage(pil_img)

        #Save image?
        if self.screenshot:
            path = "/mnt/usb/images/" +  str(prog_time).replace(":", "-") + ".jpg"
            pil_img.save(path, 'JPEG')


        #Work to display will be shown here
        #Init panels at start
        if self.panel_a is None:
            self.panel_a = tk.Label(image=tk_img)
            self.panel_a.image = tk_img
            #Tell Tkinter to fill both x and y axes. Additionally, if the
            #window should change size, the frame will too

        #If already intitialized, update
        else:
            self.panel_a.configure(image=tk_img)
            self.panel_a.image = tk_img
            self.panel_a.pack(fill=tk.BOTH, expand = True)

        self.root.update()

        #Toggle pin off
        gpio.output(24, 0)

    def data_callback(self, data):
        '''
        Take message data in from the appropriate subscriber
        and set it to class variables so that the values
        may be accessed for the GUI overlay
        '''
        self.heading = data.degrees_north
        self.battery_voltage = data.voltage_battery
        self.depth = data.upward
        self.err_mess = data.error
        self.ballast_left = data.ballast_left
        self.ballast_right = data.ballast_right
        self.speed = data.forward

    def screenshot_callback(self, com):
        '''
        This is the controller subscriber. The only
        value that we care about is the screenshot as this
        is the only command the controller takes action on
        '''
        self.screenshot = com.screenshot



def main(args=None):
    '''
    This method actually spins up the ROS node so that it may function
    '''
    rclpy.init(args=args)
    gui_obj = GUI()
    rclpy.spin(gui_obj)
    rclpy.shutdown()
    #Shut down pin grab
    gpio.cleanup()

if __name__ == '__main__':
    main()
