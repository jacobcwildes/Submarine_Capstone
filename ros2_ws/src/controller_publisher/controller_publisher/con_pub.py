'''
This file deals with making a controller data publishing node.
Once the node is created this file will continue to run so that the 
data can be published down to the sub
'''
#ROS2 Necessities
import rclpy
from rclpy.node import Node
from rclpy import qos

#Import custom message type
from com_interfaces.msg import ComInfo

import serial

class ControllerOutput(Node):
    '''
    This class makes the controller publishing node, which publishes data at 
    20Hz.
    '''
    def __init__(self):
        #Command publication
        super().__init__('controller_publisher')
        #Make a publisher - first arg is custom defined message, second
        #is topic name, third is how many things to queue to publish (TCP)
        self.com_pub = self.create_publisher(ComInfo, 'com_info', qos.qos_profile_services_default)

        #Serial read setup
        self.serial_port = None
        self.serial_line = None

        #Publish data
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        '''
        This callback is responsible for actually querying and publishing data from
        the controller STM board
        '''
        #Keep trying to initiate contact with board, once established proceed
        try:
            if self.serial_port is None:
                self.serial_port = serial.Serial("/dev/ttyACM0", 115200, timeout=0.5)
        except:
            print("Unable to connect to STM board!")
            #Set message to custom datatype
            msg = ComInfo()
            msg.left_toggle_ud = 0
            msg.left_toggle_lr = 1
            msg.right_toggle_ud = 2
            msg.right_toggle_lr = 3
            msg.sub_up = 0
            msg.sub_down = 1
            msg.screenshot = 0
            self.com_pub.publish(msg)

        if self.serial_port is not None:
            #Read serial data and save to variables
            self.serial_line = self.serial_port.readline()
            self.serial_line = self.serial_line.decode('ascii')
            local_dat = self.serial_line.strip().strip('\x00')
            local_dat = local_dat.split(',')
            #print(local_dat)

            try:
                if len(local_dat) == 7:
                    #Set message to custom datatype
                    msg = ComInfo()

                    #Normalize & put data in each field of datatype
                    msg.left_toggle_ud = normalization(int(local_dat[0]))
                    msg.left_toggle_lr = normalization(int(local_dat[1]))
                    msg.right_toggle_ud = (255 -  normalization(int(local_dat[2])))
                    msg.right_toggle_lr =(255 -  normalization(int(local_dat[3])))
                    msg.sub_up = int(local_dat[4])
                    msg.sub_down = int(local_dat[5])
                    msg.screenshot = int(local_dat[6])

                    self.com_pub.publish(msg)
                    self.get_logger().info(f"Publishing Left Toggle UD: {msg.left_toggle_ud}")
                    self.get_logger().info(f"Publishing Left Toggle LR: {msg.left_toggle_lr}")
                    self.get_logger().info(f"Publishing Right Toggle UD: {msg.right_toggle_ud}")
                    self.get_logger().info(f"Publishing Right Toggle LR: {msg.right_toggle_lr}")
                    self.get_logger().info(f"Publishing SubUp: {msg.sub_up}")
                    self.get_logger().info(f"Publishing SubDown: {msg.sub_down}")
                    self.get_logger().info(f"Publishing Screenshot: {msg.screenshot}")

            #Sometimes the first data line is bad - no need to crash the entire program
            #because of it
            except ValueError:
                pass
            except UnicodeDecodeError:
                pass

def normalization(data):
    '''
    This normalization method helps keep the toggle drift to a minimum. When "at rest",
    the values drift slightly. This prevents the sub from acting erratically
    '''
    ##normal = abs(int((((4.64 * (10 ** -3) * (data ** 2)) - (.182 * data)))))
    #return normal
    if data > 100 and data < 150:
        data = 128
        return data
    else:
        return data

    #for i in range(len(local_dat)):
    #    normalized_data.append((((4.64 * (10**-3)) *
    #    (int(str(local_dat[int(i)])) ** 2)) - (.182 * int(str(local_dat[int(i)])))))
    #return normalized_data

def main(args=None):
    '''
    The method spins up the ROS node so that the controller publisher actually
    runs
    '''
    rclpy.init(args=args)
    con_obj = ControllerOutput()
    rclpy.spin(con_obj)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
