#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import math

from vs_msgs.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped


class ParkingController(Node):
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """

    def __init__(self):
        super().__init__("parking_controller")

        self.declare_parameter("drive_topic")
        DRIVE_TOPIC = self.get_parameter("drive_topic").value  # set in launch file; different for simulator vs racecar

        self.drive_pub = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 10)
        self.error_pub = self.create_publisher(ParkingError, "/parking_error", 10)

        self.create_subscription(ConeLocation, "/relative_cone", self.relative_cone_callback, 1)

        self.parking_distance = 1.5  # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0

        self.get_logger().info("Parking Controller Initialized")

        self.distance = 0.0
        self.speed = 0.0
        self.steering_angle = 0.0

    # def relative_cone_callback(self, msg):
    #     self.relative_x = msg.x_pos
    #     self.relative_y = msg.y_pos
    #     drive_cmd = AckermannDriveStamped()

    #     #################################

    #     # YOUR CODE HERE
    #     # Use relative position and your control law to set drive_cmd

    #     self.distance = math.hypot(self.relative_x, self.relative_y)
    #     angle = math.atan2(self.relative_y, self.relative_x)

    #     # slow_threshold = 2.0

    #     stop_distance = (self.distance + angle) - self.parking_distance
    #     self.speed = np.clip(stop_distance, -1.0, 1.0)

    #     if self.speed < 0:
    #         steering_angle = -angle
    #     else:
    #         steering_angle = angle
            
    #     self.steering_angle = np.clip(steering_angle, -4.0, 4.0)
            

    #     drive_cmd.drive.steering_angle = self.steering_angle
    #     drive_cmd.drive.speed = self.speed
    #     drive_cmd.drive.steering_angle_velocity = 0.0 

    #     #################################

    #     self.drive_pub.publish(drive_cmd)
    #     self.error_publisher()

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()

        #################################

        # YOUR CODE HERE
        # Use relative position and your control law to set drive_cmd

        self.distance = math.hypot(self.relative_x, self.relative_y)
        angle = math.atan2(self.relative_y, self.relative_x)

        # slow_threshold = 2.0

        if angle > 0.05 or angle < - 0.05:
            self.speed = -1.0
            steering_angle = -angle
        else: 
            stop_distance = (self.distance + angle) - self.parking_distance
            self.speed = np.clip(stop_distance, -1.0, 1.0)
            steering_angle = angle
            
        self.steering_angle = np.clip(steering_angle, -4.0, 4.0)
            

        drive_cmd.drive.steering_angle = self.steering_angle
        drive_cmd.drive.speed = self.speed
        drive_cmd.drive.steering_angle_velocity = 0.0 

        #################################

        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        #################################

        # YOUR CODE HERE
        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)

        error_msg.x_error = self.relative_x
        error_msg.y_error = self.relative_y

        error_msg.distance_error = self.distance
        #################################

        self.error_pub.publish(error_msg)


def main(args=None):
    rclpy.init(args=args)
    pc = ParkingController()
    rclpy.spin(pc)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
