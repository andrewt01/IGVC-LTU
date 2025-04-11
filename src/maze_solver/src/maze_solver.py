#!/usr/bin/env python3

# imported libaries for code 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class mazeSolver(Node):

    def __init__(self):
        super().__init__('maze_solver')


        # publisher and subscribers for twist and laser scan 
        # vel is velocity 
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.laser_scan_subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.vel = Twist()

    def laser_callback(self, msg):
        #represents the degress since the sensor is 360 
        left_distance = msg.ranges[60:120]  
        right_distance = msg.ranges[240:300]
        forward_distance = min(msg.ranges[0:10]+ msg.ranges[-10:])

        if forward_distance < 1:
            # stop moving towards the wall in the maze
            self.vel.linear.x = 0.0 # set velocity to 0 if object is 0.5 meters away in front of robot
            self.vel.angular.z = 0.0 # set angular velocity to 0 as well

            # now need to rotate to the left or right to allow robot to move in another direction
            self.vel.angular.z = -0.5 # set at 0.5 to allow for smooth rotation, can increase later 
            self.vel_publisher.publish(self.vel)

            self.get_logger().info('Wall in front of Waffle, rotating to avoid.')

        elif left_distance < 1:
             # stop moving towards the wall in the maze
            self.vel.linear.x = 0.0 # set velocity to 0 if object is 0.5 meters away in front of robot
            self.vel.angular.z = 0.0 # set angular velocity to 0 as well

            self.vel.angular.z = -0.5 # set at 0.5 to allow for smooth rotation, can increase later 
            self.vel_publisher.publish(self.vel)

        
        else:
            # set linear velocity to 0.5 to move forward and angular to 0 since we do not need to rotate yet
            self.vel.linear.x = 0.5
            self.vel.angular.z = 0.0
            self.vel_publisher.publish(self.vel)

            self.get_logger().info('No wall detected, keep moving forward Waffle.')



def main(args=None):
    rclpy.init(args=args)
    node = mazeSolver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
