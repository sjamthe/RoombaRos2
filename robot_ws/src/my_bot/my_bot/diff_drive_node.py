#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from example_interfaces.srv import SetBool
from std_msgs.msg import Int32  #Import for robot_state aiMode message

import requests
from urllib.parse import urljoin
import math
import numpy as np

class AccelerationProfile:
    def __init__(self, max_accel, max_decel, max_jerk):
        self.max_accel = max_accel
        self.max_decel = max_decel
        self.max_jerk = max_jerk
        self.current_accel = 0.0

    def calculate_next_speed(self, current_speed, target_speed, dt):
        # Determine desired acceleration
        speed_diff = target_speed - current_speed
        desired_accel = speed_diff / dt if dt > 0 else 0
        
        # Limit jerk (rate of acceleration change)
        max_accel_change = self.max_jerk * dt
        target_accel = np.clip(
            desired_accel,
            self.current_accel - max_accel_change,
            self.current_accel + max_accel_change
        )
        
        # Apply acceleration limits
        if target_speed > current_speed:
            target_accel = min(target_accel, self.max_accel)
        else:
            target_accel = max(target_accel, -self.max_decel)
        
        # Update current acceleration
        self.current_accel = target_accel
        
        # Calculate new speed
        new_speed = current_speed + target_accel * dt
        return new_speed

class DifferentialDriveNode(Node):
    def __init__(self):
        super().__init__('diff_drive_node')
        
        # Robot state init
        self.is_powered = False 

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_url', 'http://192.168.86.28'),
                ('wheel_separation', 0.232),
                ('wheel_diameter', 0.072),
                ('max_wheel_speed', 200),
                ('min_wheel_speed', 50),
                ('max_acceleration', 200),
                ('max_deceleration', 300),
                ('max_jerk', 500),
                ('command_timeout', 0.5),
                ('control_frequency', 1.0)  # Hz
            ]
        )
        
        # Get parameters
        self.base_url = self.get_parameter('robot_url').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.max_wheel_speed = self.get_parameter('max_wheel_speed').value
        self.min_wheel_speed = self.get_parameter('min_wheel_speed').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.control_period = 1.0 / self.control_frequency  # seconds
        
        # Initialize acceleration profiles
        self.left_accel_profile = AccelerationProfile(
            self.get_parameter('max_acceleration').value,
            self.get_parameter('max_deceleration').value,
            self.get_parameter('max_jerk').value
        )
        self.right_accel_profile = AccelerationProfile(
            self.get_parameter('max_acceleration').value,
            self.get_parameter('max_deceleration').value,
            self.get_parameter('max_jerk').value
        )
        
        # Current state
        self.prev_left_speed = 0
        self.prev_right_speed = 0
        self.current_left_speed = 0
        self.current_right_speed = 0
        self.target_left_speed = 0
        self.target_right_speed = 0
        self.last_command_time = self.get_clock().now()
        
        # Subscribe to robot_mode
        self.state_sub = self.create_subscription(
            Int32, 'robot_mode', self.robot_mode_callback, 10)
        
        # Subscribe to cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # Services
        self.srv_power = self.create_service(
            SetBool, 'robot/power', self.handle_power)
        self.srv_dock = self.create_service(
            Trigger, 'robot/dock', self.handle_dock)
        
        self.create_timer(self.control_period, self.control_timer)
        
        self.get_logger().info(
            f'Differential drive node initialized\n'
            f'Robot URL: {self.base_url}\n'
            f'Wheel separation: {self.wheel_separation}m\n'
            f'Wheel diameter: {self.wheel_diameter}m\n'
            f'Control frequency: {self.control_frequency}Hz'
        )

    def make_request(self, endpoint, params=None):
        url = urljoin(self.base_url, endpoint)
        try:
            response = requests.get(url, params=params, timeout=1.0)
            response.raise_for_status()
            return True, "Command executed successfully"
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'HTTP request failed: {str(e)}')
            return False, str(e)

    def set_wheel_speeds(self, left_speed, right_speed):
        """Send wheel speeds to robot"""
        return self.make_request('/diffDrive', {
            'left': int(left_speed),
            'right': int(right_speed)
        })

    def apply_deadband(self, speed):
        """Apply deadband to wheel speeds"""
        if abs(speed) < self.min_wheel_speed:
            return 0
        return speed

    def robot_mode_callback(self, msg):
        """Handle robot_mode messages"""
        self.robot_mode = msg.data
        if self.robot_mode <= 1:
            # Reset speeds for PowerOff and Passive mode
            self.current_left_speed = 0
            self.current_right_speed = 0
            self.target_left_speed = 0
            self.target_right_speed = 0
            self.is_powered = False
        else:
            self.is_powered = True

    def cmd_vel_callback(self, msg):
        """Convert cmd_vel to wheel speeds"""
        self.last_command_time = self.get_clock().now()
        
        # Differential drive kinematics for linear wheel velocities (m/s)
        left_speed = msg.linear.x - msg.angular.z * self.wheel_separation / 2
        right_speed = msg.linear.x + msg.angular.z * self.wheel_separation / 2
        
        # Scale to robot's speed range
        max_speed = max(abs(left_speed), abs(right_speed))
        if max_speed > self.max_wheel_speed:
            scale = self.max_wheel_speed / max_speed
            left_speed *= scale
            right_speed *= scale
        
        # Apply deadband and set targets
        self.target_left_speed = self.apply_deadband(left_speed)
        self.target_right_speed = self.apply_deadband(right_speed)
        self.get_logger().debug(f'In cmd_vel_callback: sett speeds {self.target_left_speed} {self.target_right_speed}')


    def control_timer(self):
        """Main control loop"""
        # Only send commands if robot is powered on
        if not self.is_powered or self.robot_mode <= 1:
            return
    
        # Check command timeout
        dt = self.control_period
        if (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9 > self.get_parameter('command_timeout').value:
            self.target_left_speed = 0
            self.target_right_speed = 0
        
        # Apply acceleration profiles
        self.current_left_speed = self.left_accel_profile.calculate_next_speed(
            self.current_left_speed, self.target_left_speed, dt)
        self.current_right_speed = self.right_accel_profile.calculate_next_speed(
            self.current_right_speed, self.target_right_speed, dt)
        
        # Send commands to robot - don;t repeat 0 0 as it happens too ma ny times when idle.
        if self.current_left_speed != 0 or self.current_right_speed != 0 or self.prev_left_speed != 0 or self.prev_right_speed != 0:
            self.set_wheel_speeds(self.current_left_speed, self.current_right_speed)

        self.prev_left_speed = self.current_left_speed
        self.prev_right_speed = self.current_right_speed

    def handle_power(self, request, response):
        """Handle power on/off requests"""
        endpoint = '/safeMode' if request.data else '/powerOff'
        success, message = self.make_request(endpoint)
        if success:
            # Update power state immediately rather than waiting for state update
            self.is_powered = request.data
            if not self.is_powered:
                # Reset speeds when powering off
                self.current_left_speed = 0
                self.current_right_speed = 0
                self.target_left_speed = 0
                self.target_right_speed = 0

        response.success = success
        response.message = message
        return response

    def handle_dock(self, request, response):
        """Handle dock requests"""
        success, message = self.make_request('/dock')
        response.success = success
        response.message = message
        return response

def main():
    rclpy.init()
    node = DifferentialDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
