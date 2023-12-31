'''
This file is made strictly as a testbench for the controller GUI.
Built by Jacob Wildes
'''
#This node will not be used in the end product. All this will do is publish dummy data so
#that the GUI can be tested
import rclpy
from rclpy.node import Node

#Import the custom message type for testing
from com_interfaces.msg import DataInfo

class DummyPublisher(Node):
    '''
    This data publisher sends dummy data which is a placeholder for what the 
    submarine would send when it is ready
    '''
    def __init__(self):
        super().__init__('dummy')
        self.publisher = self.create_publisher(DataInfo, '/dummy_data', 10)
        timer_period = .066 #In seconds (15 FPS)
        #Create the callback that will go into the "timer_callback" method (also publishes data)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.north = 0
        self.speed = 0
        self.depth = 0
        self.batt = 14.8 #Charged voltage of the battery bank sub-side
        self.i = 0 #Simple message counter for dummy message line

    def timer_callback(self):
        '''
        This method incremements the data to show a full range of the GUI
        '''
        if self.north == 359:
            self.north = 0
        if self.speed > 2.5:
            self.speed = 0
        if self.depth == 10 :
            self.depth = 0
        if self.batt < 12: #Each ICR cell has a dead voltage of 3V. 3*4 = 12
            self.batt = 14.8

        #Set the message data fields to dummy data
        #Cast to particular variables to prevent assertion problems
        msg = DataInfo()
        msg.degrees_north = int(self.north)
        msg.speed_scalar = float(self.speed)
        msg.depth_approx = int(self.depth)
        msg.voltage_battery = float(self.batt)
        msg.message = "Message #: " + str(self.i)

        self.publisher.publish(msg)
        self.get_logger().info("Publishing data")

        #Increment
        self.north += 1
        self.speed += .01
        self.depth += 1
        self.batt -= .01
        self.i += 1

def main(args=None):
    '''
    Spin up the ROS node to publish
    '''
    rclpy.init(args=args)
    pub = DummyPublisher()
    rclpy.spin(pub)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
