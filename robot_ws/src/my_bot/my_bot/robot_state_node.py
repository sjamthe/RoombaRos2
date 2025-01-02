#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import BatteryState
from tf2_ros import TransformBroadcaster
import math
import json
import asyncio
import aiohttp
from threading import Thread

class OdometryProcessor:
    def __init__(self, wheel_radius, wheel_separation):
        self.wheel_radius = wheel_radius
        self.wheel_separation = wheel_separation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_left_count = None
        self.last_right_count = None
        
    def process_motion_state(self, motion_state):
        current_left_count = motion_state.get('leftEncoderCount', 0)
        current_right_count = motion_state.get('rightEncoderCount', 0)
        
        if self.last_left_count is None:
            self.last_left_count = current_left_count
            self.last_right_count = current_right_count
            return None
            
        # Calculate deltas
        delta_left = current_left_count - self.last_left_count
        delta_right = current_right_count - self.last_right_count
        
        # Update last counts
        self.last_left_count = current_left_count
        self.last_right_count = current_right_count
        
        # Calculate distances
        left_distance = delta_left * self.wheel_radius
        right_distance = delta_right * self.wheel_radius
        
        # Update pose
        center_distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_separation
        
        self.theta += delta_theta
        self.x += center_distance * math.cos(self.theta)
        self.y += center_distance * math.sin(self.theta)
        
        return {
            'x': self.x,
            'y': self.y,
            'theta': self.theta
        }

class PowerStateProcessor:
    @staticmethod
    def process_power_state(power_state):
        battery_msg = BatteryState()
        battery_msg.voltage = float(power_state['voltage']) / 1000.0  # Convert mV to V
        battery_msg.current = float(power_state['current']) / 1000.0  # Convert mA to A
        battery_msg.charge = float(power_state['batteryCharge']) / 1000.0  # Convert mAh to Ah
        battery_msg.capacity = float(power_state['batteryCapacity']) / 1000.0  # Convert mAh to Ah
        battery_msg.temperature = float(power_state['temperature'])
        
        charging_state = power_state['chargingState']
        if charging_state == 4:
            battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
        elif charging_state > 0:
            battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        else:
            battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            
        return battery_msg

class RobotStateNode(Node):
    def __init__(self):
        super().__init__('robot_state_node')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('server_url', 'http://192.168.86.28/events'),
                ('wheel_radius', 0.033),
                ('wheel_separation', 0.17)
            ]
        )
        
        # Get parameter values
        self.server_url = self.get_parameter('server_url').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        
        # Log parameters
        self.get_logger().info(f'Server URL: {self.server_url}')
        self.get_logger().info(f'Wheel radius: {self.wheel_radius}m')
        self.get_logger().info(f'Wheel separation: {self.wheel_separation}m')
        
        # Initialize processors
        self.odometry_processor = OdometryProcessor(
            wheel_radius=self.wheel_radius,
            wheel_separation=self.wheel_separation
        )
        self.power_processor = PowerStateProcessor()
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.battery_pub = self.create_publisher(BatteryState, 'battery_state', 10)
        
        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Start event loop in separate thread
        self.event_thread = Thread(target=self._run_event_loop)
        self.event_thread.daemon = True
        self.event_thread.start()

    def _run_event_loop(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self._listen_events())

    async def _listen_events(self):
        async with aiohttp.ClientSession() as session:
            while rclpy.ok():
                try:
                    async with session.get(self.server_url) as response:
                        async for line in response.content:
                            line = line.decode('utf-8').strip()
                            if line.startswith('data: '): 
                                if line.startswith('data: Connection'):
                                    continue  # Skip connection established event

                                data = json.loads(line[6:])
                                self.process_state_data(data)
                except Exception as e:
                    self.get_logger().error(f'Error in event connection: {e} : {line}')
                    await asyncio.sleep(5)  # Wait before retrying

    def process_state_data(self, state_data):
        # Process motion state for odometry
        if 'motionState' in state_data:
            odom_data = self.odometry_processor.process_motion_state(state_data['motionState'])
            if odom_data:
                self.publish_odometry(odom_data)
        
        # Process power state
        if 'powerState' in state_data:
            battery_msg = self.power_processor.process_power_state(state_data['powerState'])
            battery_msg.header.stamp = self.get_clock().now().to_msg()
            self.battery_pub.publish(battery_msg)

    def publish_odometry(self, odom_data):
        current_time = self.get_clock().now().to_msg()
        
        # Publish transform
        transform = TransformStamped()
        transform.header.stamp = current_time
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        
        transform.transform.translation.x = odom_data['x']
        transform.transform.translation.y = odom_data['y']
        transform.transform.translation.z = 0.0
        
        # Convert theta to quaternion
        theta = odom_data['theta']
        transform.transform.rotation.z = math.sin(theta / 2.0)
        transform.transform.rotation.w = math.cos(theta / 2.0)
        
        self.tf_broadcaster.sendTransform(transform)
        
        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = odom_data['x']
        odom.pose.pose.position.y = odom_data['y']
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = transform.transform.rotation
        
        self.odom_pub.publish(odom)

def main():
    rclpy.init()
    node = RobotStateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()