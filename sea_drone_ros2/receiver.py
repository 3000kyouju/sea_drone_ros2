#!/usr/bin/env python3

#あらかじめ実行すること
#(1) ~/RaspberryPiMouse/utils/build_install.bash
#(2) ros2 launch raspimouse raspimouse.launch.py

from rclpy.node import Node
import rclpy
from std_msgs.msg import String
import datetime
import math

class Receiver(Node):
    def __init__(self):
        super().__init__('receiver')

        self.previous_control_time = datetime.datetime.now()
        self.linear_R = 0.0
        self.linear_L = 0.0
        self.subscription = self.create_subscription(String, 'controller', self.set_joy, 1)
        self.pub = self.create_publisher(String, '/verchal_joy', 1)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run)

    def set_joy(self, data):
        rev = data.data.split(',')
        speed = float(rev[0])
        angle = float(rev[1])
        self.convert_LR(speed, angle)
        self.previous_control_time = datetime.datetime.now() 

    def convert_LR(self, sd, ag):

        if ag > 0:
            self.linear_R = -0.3
            self.linear_L = 0
        elif ag < 0:
            self.linear_R = 0
            self.linear_L = -0.3
        elif ag == 0 and sd > 0:
            self.linear_R = -0.3
            self.linear_L = -0.3
        else:
            self.linear_R = 0
            self.linear_L = 0

        self.get_logger().info("R:{},L:{}".format(self.linear_R, self.linear_L))

    def run(self, reset=False):
        msg = String()
        if datetime.datetime.now() > self.previous_control_time + datetime.timedelta(seconds=0.3):
            self.linear_R = 0.0
            self.linear_L = 0.0
        msg.data = "{},{}".format(self.linear_R, self.linear_L) 
        self.get_logger().info(msg.data)
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    receiver = Receiver()
    rclpy.spin(receiver)

if __name__ == '__main__':
    main()