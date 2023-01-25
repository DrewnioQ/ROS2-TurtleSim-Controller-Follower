#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt


class TurtleSimFollower(Node):
    def __init__(self):
        super().__init__("turtlesim_follower")
        self.publisher_ = self.create_publisher(Twist, "/turtle2/cmd_vel", 10)
        self.turtle2_subscriber = self.create_subscription(Pose, "/turtle2/pose", self.update_turtle2_pose, 10) 
        self.turtle1_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.update_turtle1_pose, 10)
        self.turtle1_pose = Pose()
        self.turtle2_pose = Pose()
        self.get_logger().info("Turtle1 follower initiated")
        

    def update_turtle1_pose(self, data):
        self.turtle1_pose = data
        self.turtle1_pose.x = round(self.turtle1_pose.x, 4)
        self.turtle1_pose.y = round(self.turtle1_pose.y, 4)

    def update_turtle2_pose(self, data):
        self.turtle2_pose = data
        self.turtle2_pose.x = round(self.turtle2_pose.x, 4)
        self.turtle2_pose.y = round(self.turtle2_pose.y, 4)
        self.move2goal()

    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.turtle2_pose.x), 2) +
                    pow((goal_pose.y - self.turtle2_pose.y), 2))

    def linear_vel(self, goal_pose):
        constant = 1
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.turtle2_pose.y, goal_pose.x - self.turtle2_pose.x)

    def angular_vel(self, goal_pose):
        constant = 6
        return constant * (self.steering_angle(goal_pose) - self.turtle2_pose.theta)

    def move2goal(self):
        
        goal_pose = Pose()
        goal_pose.x = self.turtle1_pose.x
        goal_pose.y = self.turtle1_pose.y
        distance_tolerance = 0.01

        vel_msg = Twist()
        
        if self.euclidean_distance(goal_pose) >= distance_tolerance:
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0

            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            self.publisher_.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    follower = TurtleSimFollower()
    rclpy.spin(follower)
    rclpy.shutdown()


if __name__ == '__main__':
    main()