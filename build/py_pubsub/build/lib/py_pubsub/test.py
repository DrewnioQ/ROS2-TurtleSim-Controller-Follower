#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # ROS2 package to convert between ROS and OpenCV Images
import cv2 # Python OpenCV library
import numpy as np
from geometry_msgs.msg import Twist

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.window_name = "Controller"
        self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.get_logger().info("Publishing cmd_vel to turtlesim")
        self.point = None
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.listener_callback)

    def listener_callback(self):
        cv_image = np.zeros((512,700,3), np.uint8)
        if(self.point is not None):
            cv2.rectangle(cv_image,self.point,(self.point[0]+200,self.point[1]+200),(0,255,0),3)
        cv2.imshow(self.window_name, cv_image)
        cv2.waitKey(25)
        cv2.setMouseCallback(self.window_name)
        

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()