#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import numpy as np
import math

class RotationCalibrationTest(Node):
    def __init__(self):
        super().__init__('rotation_calibration_test')
        
        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.calibration_trigger_pub = self.create_publisher(Bool, '/calibration_trigger', 10)
        self.rotation_results_sub = self.create_subscription(
            Bool, 
            '/rotation_calibration_results', 
            self.rotation_results_callback, 
            10
        )
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('rotation_angle', 90.0),     # Degrees per rotation
                ('rotation_speed', 0.5),      # radians/s
                ('num_rotations', 4),         # Number of rotations to perform
                ('settle_time', 1.0),         # Settle time between rotations
                ('error_threshold', 0.1)      # Acceptable rotation error
            ]
        )
        
        # Get parameter values
        self.rotation_angle = self.get_parameter('rotation_angle').value
        self.rotation_speed = self.get_parameter('rotation_speed').value
        self.num_rotations = self.get_parameter('num_rotations').value
        self.settle_time = self.get_parameter('settle_time').value
        self.error_threshold = self.get_parameter('error_threshold').value
        
        # State tracking
        self.rotations_completed = 0
        self.rotation_results = []
        
        self.get_logger().info("Rotation Calibration Test Node Initialized")

    def start_rotation_test(self):
        """Initiate the rotation calibration test sequence."""
        self.get_logger().info("Starting Rotation Calibration Test Sequence")
        self.rotations_completed = 0
        self.rotation_results = []
        self.perform_next_rotation()

    def perform_next_rotation(self):
        if hasattr(self, 'timer'):
            self.timer.cancel() # Cancel the timer after first execution

        """Perform a single controlled rotation."""
        if self.rotations_completed >= self.num_rotations:
            self.finalize_test()
            return

        # Trigger calibration data collection
        trigger_msg = Bool()
        trigger_msg.data = True
        self.calibration_trigger_pub.publish(trigger_msg)
        
        # Prepare rotation command
        twist = Twist()
        twist.angular.z = self.rotation_speed  # Positive for counterclockwise
        
        # Publish rotation command
        self.get_logger().info(f"Publishing rotational_speed {twist.angular.z} to cmd_vel")
        self.cmd_vel_pub.publish(twist)
        
        # Schedule stopping the rotation
        # Seconds to stop
        seconds = abs(self.rotation_angle/180.0*3.1415 / self.rotation_speed);  
        self.timer = self.create_timer(seconds, self.stop_rotation)
        self.get_logger().info(f"Created timer for {seconds}")

    def stop_rotation(self):
        """Stop the robot's rotation and prepare for next step."""
        # Stop the robot

        if hasattr(self, 'timer'):
            self.timer.cancel() # Cancel the timer after first execution

        twist = Twist()
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Published stop to cmd_vel")
        
        # Increment rotation counter
        self.rotations_completed += 1
        
        # Wait a moment for settling
        self.timer = self.create_timer(1.0, self.perform_next_rotation)

    def finalize_test(self):
        """Complete the rotation test."""
        self.get_logger().info("Rotation Calibration Test Completed")
        # Optionally publish a final calibration trigger or log results

    def rotation_results_callback(self, result_msg):
        self.get_logger().info("Rotation Results: {result_msg}")


def main(args=None):
    rclpy.init(args=args)
    node = RotationCalibrationTest()
    
    try:
        node.start_rotation_test()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
