#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from vs_msgs.msg import ConeLocation

class HomographyEvaluator(Node):
    def __init__(self):
        super().__init__("homography_evaluator")

        # Subscribe to the estimated cone location
        self.cone_sub = self.create_subscription(
            ConeLocation,
            "/relative_cone",
            self.cone_callback,
            10
        )
        METERS_PER_INCH = 0.0254
        # Ground truth: specify the actual (x, y) location of the cone in meters here
        self.ground_truth_x = 48.0 * METERS_PER_INCH
        self.ground_truth_y = -24 * METERS_PER_INCH

        self.errors = []
        self.get_logger().info("Homography Evaluator Initialized. Play your bag file to start collecting data!")

    def cone_callback(self, msg):
        # Calculate Euclidean distance error
        error = np.sqrt((msg.x_pos - self.ground_truth_x)**2 + (msg.y_pos - self.ground_truth_y)**2)
        self.errors.append(error)

        avg_error = np.mean(self.errors)
        self.get_logger().info(f"Received cone estimation. Error: {error:.4f} m | Average Error: {avg_error:.4f} m")

def main(args=None):
    rclpy.init(args=args)
    evaluator = HomographyEvaluator()
    try:
        rclpy.spin(evaluator)
    except KeyboardInterrupt:
        # Upon exiting, print final metrics
        if evaluator.errors:
            avg_error = np.mean(evaluator.errors)
            std_dev = np.std(evaluator.errors)
            evaluator.get_logger().info(f"--- Final Evaluation ---")
            evaluator.get_logger().info(f"Total points evaluated: {len(evaluator.errors)}")
            evaluator.get_logger().info(f"Average Error: {avg_error:.4f} meters")
            evaluator.get_logger().info(f"Standard Deviation: {std_dev:.4f} meters")
        else:
            evaluator.get_logger().info("No data received.")
    finally:
        evaluator.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
