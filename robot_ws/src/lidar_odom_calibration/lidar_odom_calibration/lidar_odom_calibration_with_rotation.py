#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster
from sklearn.neighbors import NearestNeighbors
import math

class LidarOdomCalibration(Node):
    def __init__(self):
        super().__init__('lidar_odom_calibration')
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('min_point_distance', 0.1),
                ('max_point_distance', 12.0),
                ('num_samples', 100),
                ('num_runs', 5),
                ('current_transform_x', -0.05),
                ('current_transform_y', 0.10),
                ('current_transform_z', 0.10),
                ('current_transform_roll', 0.0),
                ('current_transform_pitch', 0.0),
                ('current_transform_yaw', 0.0)
            ]
        )
        
        # Calibration mode configurations
        self.min_point_distance = self.get_parameter('min_point_distance').value
        self.max_point_distance = self.get_parameter('max_point_distance').value
        self.num_samples = self.get_parameter('num_samples').value
        
        # Store previous data
        self.prev_scan = None
        self.prev_time = None
        self.prev_odom = None
        self.curr_odom = None
        
        # Calibration state management
        self.runs_completed = 0
        self.num_runs = self.get_parameter('num_runs').value
        self.is_collecting = False
        
        # Rotation test specific tracking
        self.rotation_samples = {
            'lidar_rotation': [],
            'odom_rotation': [],
            'rotation_error': []
        }
        
        # QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # TF2 Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            sensor_qos
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            sensor_qos
        )
        
        # Calibration trigger subscriber
        self.trigger_sub = self.create_subscription(
            Bool,
            '/calibration_trigger',
            self.calibration_trigger_callback,
            10
        )
        
        # Rotation test results publisher
        self.rotation_results_pub = self.create_publisher(
            Bool,  # Simple bool for now, can expand later
            '/rotation_calibration_results',
            10
        )

    def calibration_trigger_callback(self, msg):
        """Reset collection state when triggered."""
        if msg.data:
            self.get_logger().info("Calibration trigger received. Resetting collection.")
            # Reset collection state
            self.is_collecting = True
            self.rotation_samples = {
                'lidar_rotation': [],
                'odom_rotation': [],
                'rotation_error': []
            }

    def polar_to_cartesian(self, scan_msg):
        """Convert LaserScan message to cartesian coordinates."""
        ranges = np.array(scan_msg.ranges)
        angles = np.arange(
            scan_msg.angle_min,
            scan_msg.angle_max + scan_msg.angle_increment,
            scan_msg.angle_increment
        )
        
        # Filter out invalid measurements
        valid = (ranges >= self.min_point_distance) & (ranges <= self.max_point_distance)
        ranges = ranges[valid]
        angles = angles[valid]
        
        # Convert to cartesian
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        
        return np.column_stack((x, y))

    def estimate_rotation(self, points1, points2):
        """
        Estimate rotation between two point clouds.
        Returns rotation angle in radians.
        """
        # Center both point clouds
        centroid1 = np.mean(points1, axis=0)
        centroid2 = np.mean(points2, axis=0)
        
        centered1 = points1 - centroid1
        centered2 = points2 - centroid2
        
        # Find corresponding points
        nbrs = NearestNeighbors(n_neighbors=1).fit(centered2)
        distances, indices = nbrs.kneighbors(centered1)
        
        # Filter out poor matches
        good_matches = distances.flatten() < np.mean(distances) + np.std(distances)
        p1_matched = centered1[good_matches]
        p2_matched = centered2[indices.flatten()[good_matches]]
        
        # Calculate rotation matrix using SVD
        H = p1_matched.T @ p2_matched
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        
        # Extract rotation angle
        return np.arctan2(R[1, 0], R[0, 0])

    def get_odom_rotation(self):
        """Extract rotation from odometry."""
        if self.prev_odom is None or self.curr_odom is None:
            return None
        
        # Extract quaternion components
        prev_qz = self.prev_odom.pose.pose.orientation.z
        prev_qw = self.prev_odom.pose.pose.orientation.w
        curr_qz = self.curr_odom.pose.pose.orientation.z
        curr_qw = self.curr_odom.pose.pose.orientation.w
        
        # Convert to yaw angles
        prev_yaw = 2 * np.arctan2(prev_qz, prev_qw)
        curr_yaw = 2 * np.arctan2(curr_qz, curr_qw)
        
        # Calculate rotation difference
        rotation = curr_yaw - prev_yaw
        
        # Normalize to [-pi, pi]
        rotation = math.atan2(math.sin(rotation), math.cos(rotation))
        
        return rotation

    def scan_callback(self, scan_msg):
        """Process incoming laser scan messages."""
        # Convert scan to cartesian coordinates
        current_points = self.polar_to_cartesian(scan_msg)
        
        if self.prev_scan is not None and self.is_collecting:
            # Estimate rotation using LiDAR data
            try:
                lidar_rotation = self.estimate_rotation(self.prev_scan, current_points)
                odom_rotation = self.get_odom_rotation()
                
                if odom_rotation is not None:
                    # Calculate rotation error
                    rotation_error = lidar_rotation - odom_rotation
                    
                    # Store samples
                    self.rotation_samples['lidar_rotation'].append(lidar_rotation)
                    self.rotation_samples['odom_rotation'].append(odom_rotation)
                    self.rotation_samples['rotation_error'].append(rotation_error)
                    
                    # Log intermediate results
                    self.get_logger().info(
                        f"Rotation - LiDAR: {lidar_rotation:.4f}, "
                        f"Odom: {odom_rotation:.4f}, "
                        f"Error: {rotation_error:.4f}"
                    )
                    
                    # Check if we've collected enough samples
                    if len(self.rotation_samples['rotation_error']) >= self.num_samples:
                        self.finalize_rotation_calibration()
            
            except Exception as e:
                self.get_logger().error(f"Error in rotation estimation: {e}")
        
        # Update previous scan
        self.prev_scan = current_points

    def finalize_rotation_calibration(self):
        """Analyze rotation calibration results."""
        self.is_collecting = False
        
        # Calculate statistics
        error_samples = np.array(self.rotation_samples['rotation_error'])
        lidar_samples = np.array(self.rotation_samples['lidar_rotation'])
        odom_samples = np.array(self.rotation_samples['odom_rotation'])
        
        # Compute statistics
        rotation_error_mean = np.mean(error_samples)
        rotation_error_std = np.std(error_samples)
        
        # Detailed logging
        self.get_logger().info("\n=== Rotation Calibration Results ===")
        self.get_logger().info(f"Rotation Error Statistics:")
        self.get_logger().info(f"  Mean Error: {rotation_error_mean:.6f} rad")
        self.get_logger().info(f"  Std Deviation: {rotation_error_std:.6f} rad")
        
        # LiDAR vs Odometry comparisons
        self.get_logger().info("\nLiDAR Rotation:")
        self.get_logger().info(f"  Mean: {np.mean(lidar_samples):.6f} rad")
        self.get_logger().info(f"  Std Deviation: {np.std(lidar_samples):.6f} rad")
        
        self.get_logger().info("\nOdometry Rotation:")
        self.get_logger().info(f"  Mean: {np.mean(odom_samples):.6f} rad")
        self.get_logger().info(f"  Std Deviation: {np.std(odom_samples):.6f} rad")
        
        # Publish results (simple boolean for now)
        result_msg = Bool()
        result_msg.data = True
        if(abs(rotation_error_mean) < 0.1): # Threshold can be adjusted
            result_msg.data = False
        self.rotation_results_pub.publish(result_msg)
        
        # Optional: Suggest calibration adjustments
        if abs(rotation_error_mean) > 0.1:
            self.get_logger().warn(
                "Significant rotation discrepancy detected. "
                "Consider adjusting wheel base or encoder resolution."
            )

    def odom_callback(self, odom_msg):
        """Store odometry data for comparison."""
        self.prev_odom = self.curr_odom
        self.curr_odom = odom_msg

def main(args=None):
    rclpy.init(args=args)
    node = LidarOdomCalibration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
