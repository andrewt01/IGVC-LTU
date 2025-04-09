#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist

class LaneFollowingNode(Node):
    def __init__(self):
        super().__init__('lane_following_node')
        self.bridge = CvBridge()

        # Subscribe to the camera feed on the turtlebot3 waffle
        self.subscription = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',  # TurtleBot3 camera topic
            self.image_callback,
            10
        )

        # control to twist publisher
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.initial_forward_vel()
        self.get_logger().info("Lane following node initialized.")
        
    def initial_forward_vel(self):
    	twist = Twist()
    	twist.linear.x = 1.0
    	twist.angular.z = 0.0
    	self.publisher.publish(twist)
    	self.get_logger().info("Moving Forward")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return

        # convert to grayscale and blur
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # canny to detect edges 
        edges = cv2.Canny(blurred, 50, 150)

        # roi definition 
        height, width = edges.shape
        mask = np.zeros_like(edges)
        polygon = np.array([[
            (0, height),
            (width / 2, height / 2),
            (width, height)
        ]], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        roi = cv2.bitwise_and(edges, mask)

        # Hough transform to detect straight lines and shapes if any 
        lines = cv2.HoughLinesP(roi, 1, np.pi / 180, 50, minLineLength=50, maxLineGap=200)

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 3)

            # Assuming a simple lane-following control strategy
            # Calculate the center of the detected lines
            line_midpoints = [((x1 + x2) / 2, (y1 + y2) / 2) for x1, y1, x2, y2 in lines[:, 0]]
            avg_x = np.mean([p[0] for p in line_midpoints])
            avg_y = np.mean([p[1] for p in line_midpoints])

            # Control the robot based on line position
            # Simple proportional control (steering)
            error = avg_x - width / 2  # Difference from the center of the image
            steering_speed = error / (width / 2)  # Normalize

            # Define Twist message for velocity control
            twist = Twist()
            twist.linear.x = 1.0  # Move forward at constant speed
            twist.angular.z = -steering_speed  # Adjust steering based on error

            # Publish control command
            self.publisher.publish(twist)
            self.get_logger().info("Moving along the Course")

        # Optionally, display the processed image
        cv2.imshow("Lane Following", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowingNode()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
