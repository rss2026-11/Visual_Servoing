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
        self.last_cone_msg_time = self.get_clock().now()
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)
        self.parking_distance = 0.7  # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0
        self.distance = 0.0
        self.speed = 0.0
        self.steering_angle = 0.0

        self.line_follow = False  # use to set line follow vs cone parking

        self.get_logger().info("Parking Controller Initialized")


    def relative_cone_callback(self, msg):
        self.last_cone_msg_time = self.get_clock().now()
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()
        # self.get_logger().info(f"X: {self.relative_x}, Y: {self.relative_y}")
        self.distance = math.hypot(self.relative_x, self.relative_y)
        angle = math.atan2(self.relative_y, self.relative_x)
        if self.line_follow:
            angle_error = 1.5
            distance_offset = 2
        else:
            angle_error = 0.15
            distance_offset = 0

        distance_error = self.distance - self.parking_distance + distance_offset



        # if cone is at a large angle, turn toward it first
        if abs(angle) > angle_error and self.distance < 0.5:
            speed = -0.5  # drive forward slowly while turning
            steering_angle = -angle
        # close enough to target — stop
        elif abs(distance_error) < 0.1:
            speed = 0.0
            steering_angle = 0.0
        # cone is roughly ahead — drive toward/away based on distance
        else:
            speed = np.clip(distance_error, -1.0, 1.0)
            if distance_error > 0:
                # gain x2 for cone, x.75 for line
                steering_angle = 2 * angle
            else:
                steering_angle = -angle * 2

        steering_angle = np.clip(steering_angle, -0.4, 0.4)

        drive_cmd.drive.speed = float(speed)
        drive_cmd.drive.steering_angle = float(steering_angle)
        drive_cmd.drive.steering_angle_velocity = 0.0

        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

    def watchdog_callback(self):

        current_time = self.get_clock().now()
        time_since_last_cone = (current_time - self.last_cone_msg_time).nanoseconds / 1e9

        if time_since_last_cone > 1:
            drive_cmd = AckermannDriveStamped()
            # If we were already very close, losing the cone probably means it went under the camera's view!
            if self.distance > 0.0 and self.distance < 0.6:
                drive_cmd.drive.speed = 0.0
                drive_cmd.drive.steering_angle = 0.0
            else:
                drive_cmd.drive.speed = -0.3
                drive_cmd.drive.steering_angle = -0.3
            self.drive_pub.publish(drive_cmd)


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

    #     if angle > 0.05 or angle < - 0.05:
    #         self.speed = -1.0
    #         steering_angle = -angle
    #     else:
    #         stop_distance = (self.distance + angle) - self.parking_distance
    #         self.speed = np.clip(stop_distance, -1.0, 1.0)
    #         steering_angle = angle

    #     self.steering_angle = np.clip(steering_angle, -4.0, 4.0)


    #     drive_cmd.drive.steering_angle = self.steering_angle
    #     drive_cmd.drive.speed = self.speed
    #     drive_cmd.drive.steering_angle_velocity = 0.0

    #     #################################

    #     self.drive_pub.publish(drive_cmd)
    #     self.error_publisher()

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

        error_msg.distance_error = math.sqrt(self.relative_x**2 + self.relative_y**2)
        #################################

        self.error_pub.publish(error_msg)


def main(args=None):
    rclpy.init(args=args)
    pc = ParkingController()
    rclpy.spin(pc)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
