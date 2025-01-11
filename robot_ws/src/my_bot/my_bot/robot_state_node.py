#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import BatteryState
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Int32  #Import for robot_state aiMode message

import math
import json
import asyncio
import aiohttp
from threading import Thread
import numpy as np

class OdometryProcessor:
    def __init__(self, wheel_diameter, wheel_separation, ticks_per_revolution, correction_factor, logger):
        self.wheel_diameter = wheel_diameter
        self.wheel_separation = wheel_separation
        self.ticks_per_revolution = ticks_per_revolution
        self.correction_factor = correction_factor
        self.logger = logger
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_left_count = None
        self.last_right_count = None

    def calculate_encoder_delta(self, current_count, previous_count):
        """
        Calculate encoder count delta, handling 16-bit unsigned integer rollover
        Assumes encoder counts are 16-bit unsigned integers (0-65535)
        """
        # Calculate raw difference
        delta = current_count - previous_count
        
        # Handle rollover
        if delta > 32767:  # If difference is more than half the max value, it means rollover occurred
            delta -= 65536
        elif delta < -32768:  # If difference is less than negative half the max value, rollover occurred in opposite direction
            delta += 65536
        
        return delta
       
    def process_motion_state(self, motion_state):
        """ Commenting encoder code, will read distance and theta from motion state
        current_left_count = motion_state.get('leftEncoderCount', 0)
        current_right_count = motion_state.get('rightEncoderCount', 0)
        
        if self.last_left_count is None:
            self.last_left_count = current_left_count
            self.last_right_count = current_right_count
            return None
        
        # Calculate deltas
        left_count_change = current_left_count - self.last_left_count
        right_count_change = current_right_count - self.last_right_count

        # Update last counts
        self.last_left_count = current_left_count
        self.last_right_count = current_right_count

        
        # Convert encoder counts to distance
        left_distance = (left_count_change / self.ticks_per_revolution) * (np.pi * self.wheel_diameter) * self.correction_factor
        right_distance = (right_count_change / self.ticks_per_revolution) * (np.pi * self.wheel_diameter) * self.correction_factor
        
        # Calculate center distance and angle change
        center_distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_separation
        """
        center_distance = motion_state.get('distance', 0)/1000.0  # Convert mm to m
        delta_theta = motion_state.get('angle', 0) * math.pi / 180.0  # Convert to radians
        
        self.theta += delta_theta
        self.x += center_distance * math.cos(self.theta)
        self.y += center_distance * math.sin(self.theta)

        if(center_distance != 0 or delta_theta != 0):
            self.logger.info(f'dist: {center_distance:.3f}, delta_theta: {delta_theta:.3f}, x: {self.x:.3f}, y: {self.y:.3f}, theta: {self.theta:.3f}')
        
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
        if charging_state == 0:
            battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        elif charging_state > 0 & charging_state < 4:
            battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif charging_state == 4:
            battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        else:
            battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
            
        return battery_msg

class RobotStateNode(Node):
    def __init__(self):
        super().__init__('robot_state_node')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_url', 'http://roombapi/events'),
                ('wheel_diameter', 0.072),
                ('wheel_separation', 0.232),
                ('ticks_per_revolution', 508.8),
                ('correction_factor', 0.7928 )
            ]
        )
        
        # Get parameter values
        self.robot_url = self.get_parameter('robot_url').value
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.ticks_per_revolution = self.get_parameter('ticks_per_revolution').value
        self.correction_factor = self.get_parameter('correction_factor').value
        
        # Log parameters
        self.get_logger().info(f'Robot URL: {self.robot_url}')
        self.get_logger().info(f'Wheel diameter: {self.wheel_diameter}m')
        self.get_logger().info(f'Wheel separation: {self.wheel_separation}m')
        self.get_logger().info(f'Ticks per sec: {self.ticks_per_revolution}m')
        self.get_logger().info(f'Correction factor: {self.correction_factor}')
        
        # Initialize processors
        self.odometry_processor = OdometryProcessor(
            wheel_diameter=self.wheel_diameter,
            wheel_separation=self.wheel_separation,
            ticks_per_revolution=self.ticks_per_revolution,
            correction_factor=self.correction_factor,
            logger=self.get_logger()
        )
        self.power_processor = PowerStateProcessor()
        
        # Publishers
        self.mode_publisher = self.create_publisher(Int32, 'robot_mode', 10)
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
        while rclpy.ok():
            try:
                # Create connector with shorter timeouts
                connector = aiohttp.TCPConnector(
                    force_close=True,
                    enable_cleanup_closed=True,
                    limit=1
                )                
                
                timeout = aiohttp.ClientTimeout(
                    total=10,
                    connect=5,
                    sock_read=5
                )
                
                async with aiohttp.ClientSession(
                    connector=connector,
                    timeout=timeout,
                    connector_owner=True
                ) as session:
                    self.get_logger().info(f'New connection to {self.robot_url}')
                    while rclpy.ok():
                        try:
                            async with session.get(self.robot_url) as response:
                                if response.status != 200:
                                    self.get_logger().error(f'Bad response: {response.status}')
                                    await asyncio.sleep(2)  # Shorter sleep for HTTP errors
                                    continue
                                    
                                async for line in response.content:
                                    line = line.decode('utf-8').strip()
                                    if line.startswith('data: '): 
                                        if line.startswith('data: Connection'):
                                            continue
                                        data = json.loads(line[6:])
                                        self.process_state_data(data)
                                        
                        except (aiohttp.ClientError, aiohttp.ServerDisconnectedError) as e:
                            self.get_logger().error(f'Connection error: {e}')
                            await asyncio.sleep(2)  # Shorter sleep for reconnection
                            break  # Break inner loop to create new session
                            
                        except json.JSONDecodeError as e:
                            self.get_logger().error(f'JSON decode error: {e}')
                            continue
                            
            except Exception as e:
                self.get_logger().error(f'Session error: {e}')
                await asyncio.sleep(2)  # Shorter sleep before retry

    def process_state_data(self, state_data):
        # Process aiMode        
        if 'oiMode' in state_data:
            self.msg = Int32()
            self.msg.data = state_data['oiMode'] 
            self.mode_publisher.publish(self.msg)
        
        # Process power state
        if 'powerState' in state_data:
            battery_msg = self.power_processor.process_power_state(state_data['powerState'])
            battery_msg.header.stamp = self.get_clock().now().to_msg()
            self.battery_pub.publish(battery_msg)

        # Process motion state for odometry◊
        if 'motionState' in state_data:
            odom_data = self.odometry_processor.process_motion_state(state_data['motionState'])
            if odom_data:
                self.publish_tf_and_odometry(odom_data, state_data['time'])

    def publish_tf_and_odometry(self, odom_data, time):
        # current_time = self.get_clock().now().to_msg()
        # replace time with ESP32Roomba time to be more accurate with incoming messages
        current_time = rclpy.time.Time(seconds=int(time["seconds"]), 
                              nanoseconds=int(time["microseconds"]) * 1000).to_msg()
        
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
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        
        self.tf_broadcaster.sendTransform(transform)
        
        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = transform.header.stamp 
        odom.header.frame_id = transform.header.frame_id 
        odom.child_frame_id = transform.child_frame_id 
        
        odom.pose.pose.position.x = transform.transform.translation.x
        odom.pose.pose.position.y = transform.transform.translation.y
        odom.pose.pose.position.z = transform.transform.translation.z
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
