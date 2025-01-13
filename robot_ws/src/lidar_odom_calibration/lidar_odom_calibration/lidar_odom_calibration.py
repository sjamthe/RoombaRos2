#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from sklearn.neighbors import NearestNeighbors

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
                ('num_runs', 5),  # Number of calibration runs to perform
                ('current_transform_x', -0.05),
                ('current_transform_y', 0.10),
                ('current_transform_z', 0.10),
                ('current_transform_roll', 0.0),
                ('current_transform_pitch', 0.0),
                ('current_transform_yaw', 0.0)
            ]
        )
        
        self.min_point_distance = self.get_parameter('min_point_distance').value
        self.max_point_distance = self.get_parameter('max_point_distance').value
        self.num_samples = self.get_parameter('num_samples').value
        
        # Store previous data
        self.prev_scan = None
        self.prev_time = None
        self.prev_odom = None
        self.curr_odom = None
        
        # Multi-run statistics
        self.runs_completed = 0
        self.num_runs = 5 #self.get_parameter('num_runs').value
        self.run_results = {
            'x': [],
            'y': [],
            'yaw': []
        }
        
        # Calibration data collection
        self.rotation_samples = []
        self.translation_x_samples = []
        self.translation_y_samples = []
        self.is_collecting = True
        
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

    def estimate_transform(self, points1, points2):
        """
        Estimate transformation between two point clouds.
        Returns rotation angle (rad) and translation vector [x, y].
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
        rotation = np.arctan2(R[1, 0], R[0, 0])
        
        # Calculate translation
        translation = centroid2 - (R @ centroid1.T).T
        
        return rotation, translation

    def publish_transform(self, rotation, translation, timestamp, is_error=False):
        """Publish transformation as a TransformStamped message."""
        msg = TransformStamped()
        msg.header.stamp = timestamp
        msg.header.frame_id = "base_link"
        msg.child_frame_id = "lidar_motion" if not is_error else "calibration_error"
        
        # Translation
        msg.transform.translation.x = float(translation[0])
        msg.transform.translation.y = float(translation[1])
        msg.transform.translation.z = 0.0
        
        # Rotation (in quaternion)
        msg.transform.rotation.x = 0.0
        msg.transform.rotation.y = 0.0
        msg.transform.rotation.z = float(np.sin(rotation/2))
        msg.transform.rotation.w = float(np.cos(rotation/2))
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(msg)

    def get_odom_transform(self):
        """Calculate transformation from odometry data."""
        if self.prev_odom is None or self.curr_odom is None:
            return None, None
            
        # Calculate rotation difference
        prev_yaw = 2 * np.arctan2(self.prev_odom.pose.pose.orientation.z,
                                 self.prev_odom.pose.pose.orientation.w)
        curr_yaw = 2 * np.arctan2(self.curr_odom.pose.pose.orientation.z,
                                 self.curr_odom.pose.pose.orientation.w)
        rotation = curr_yaw - prev_yaw
        
        # Calculate translation difference
        translation = np.array([
            self.curr_odom.pose.pose.position.x - self.prev_odom.pose.pose.position.x,
            self.curr_odom.pose.pose.position.y - self.prev_odom.pose.pose.position.y
        ])
        
        return rotation, translation

    def calculate_calibration_statistics(self):
        """Calculate and print statistics from collected samples."""
        # Calculate means for this run
        mean_rotation = np.mean(self.rotation_samples)
        mean_translation_x = np.mean(self.translation_x_samples)
        mean_translation_y = np.mean(self.translation_y_samples)
        
        # Calculate standard deviations for this run
        std_rotation = np.std(self.rotation_samples)
        std_translation_x = np.std(self.translation_x_samples)
        std_translation_y = np.std(self.translation_y_samples)
        
        # Get current static transform values
        current_x = self.get_parameter('current_transform_x').value
        current_y = self.get_parameter('current_transform_y').value
        current_z = self.get_parameter('current_transform_z').value
        current_roll = self.get_parameter('current_transform_roll').value
        current_pitch = self.get_parameter('current_transform_pitch').value
        current_yaw = self.get_parameter('current_transform_yaw').value
        
        # Calculate corrected transform values for this run
        new_x = current_x - mean_translation_x
        new_y = current_y - mean_translation_y
        new_yaw = current_yaw - mean_rotation
        
        # Store results for this run
        self.run_results['x'].append(new_x)
        self.run_results['y'].append(new_y)
        self.run_results['yaw'].append(new_yaw)
        
        # Print results for this run
        self.get_logger().info(f"\n=== Calibration Run {self.runs_completed + 1}/{self.num_runs} ===")
        self.get_logger().info(f"Sample Statistics for this run:")
        self.get_logger().info(f"  Rotation - Mean: {mean_rotation:.6f} rad, Std Dev: {std_rotation:.6f}")
        self.get_logger().info(f"  X Translation - Mean: {mean_translation_x:.6f} m, Std Dev: {std_translation_x:.6f}")
        self.get_logger().info(f"  Y Translation - Mean: {mean_translation_y:.6f} m, Std Dev: {std_translation_y:.6f}")
        
        self.get_logger().info(f"\nSuggested transform for this run:")
        self.get_logger().info(f"  X: {new_x:.6f} m")
        self.get_logger().info(f"  Y: {new_y:.6f} m")
        self.get_logger().info(f"  Yaw: {new_yaw:.6f} rad")
        
        # Increment run counter
        self.runs_completed += 1
        
        # If all runs completed, print final statistics
        if self.runs_completed >= self.num_runs:
            self.print_final_statistics()
        else:
            # Reset for next run
            self.rotation_samples = []
            self.translation_x_samples = []
            self.translation_y_samples = []
            self.is_collecting = True
            self.get_logger().info("\nStarting next run... Keep robot stationary.")

    def print_final_statistics(self):
        """Print statistical analysis of all runs."""
        x_values = np.array(self.run_results['x'])
        y_values = np.array(self.run_results['y'])
        yaw_values = np.array(self.run_results['yaw'])
        
        self.get_logger().info("\n========= Final Calibration Statistics =========")
        self.get_logger().info(f"Statistics across {self.num_runs} runs:")
        
        self.get_logger().info("\nX Translation (meters):")
        self.get_logger().info(f"  Mean: {np.mean(x_values):.6f}")
        self.get_logger().info(f"  Std Dev: {np.std(x_values):.6f}")
        self.get_logger().info(f"  Min: {np.min(x_values):.6f}")
        self.get_logger().info(f"  Max: {np.max(x_values):.6f}")
        self.get_logger().info(f"  Max Difference: {np.max(x_values) - np.min(x_values):.6f}")
        
        self.get_logger().info("\nY Translation (meters):")
        self.get_logger().info(f"  Mean: {np.mean(y_values):.6f}")
        self.get_logger().info(f"  Std Dev: {np.std(y_values):.6f}")
        self.get_logger().info(f"  Min: {np.min(y_values):.6f}")
        self.get_logger().info(f"  Max: {np.max(y_values):.6f}")
        self.get_logger().info(f"  Max Difference: {np.max(y_values) - np.min(y_values):.6f}")
        
        self.get_logger().info("\nYaw Rotation (radians):")
        self.get_logger().info(f"  Mean: {np.mean(yaw_values):.6f}")
        self.get_logger().info(f"  Std Dev: {np.std(yaw_values):.6f}")
        self.get_logger().info(f"  Min: {np.min(yaw_values):.6f}")
        self.get_logger().info(f"  Max: {np.max(yaw_values):.6f}")
        self.get_logger().info(f"  Max Difference: {np.max(yaw_values) - np.min(yaw_values):.6f}")
        
        # Recommended final transform (using mean values)
        self.get_logger().info("\n=== Recommended Final Static Transform ===")
        self.get_logger().info("laser_tf_node = Node(")
        self.get_logger().info("    package='tf2_ros',")
        self.get_logger().info("    executable='static_transform_publisher',")
        self.get_logger().info(f"    arguments=['{np.mean(x_values):.6f}', '{np.mean(y_values):.6f}', '{self.get_parameter('current_transform_z').value}', "
                             f"'{self.get_parameter('current_transform_roll').value}', '{self.get_parameter('current_transform_pitch').value}', "
                             f"'{np.mean(yaw_values):.6f}', 'base_link', 'laser'],")
        self.get_logger().info(")")
        
        self.get_logger().info("\nCalibration complete. You can now proceed with movement tests.")

    def scan_callback(self, scan_msg):
        """Process incoming laser scan messages."""
        # Convert scan to cartesian coordinates
        current_points = self.polar_to_cartesian(scan_msg)
        
        if self.prev_scan is not None and self.is_collecting:
            # Estimate transformation using lidar data
            lidar_rotation, lidar_translation = self.estimate_transform(
                self.prev_scan, current_points)
            
            # Get transformation from odometry
            odom_rotation, odom_translation = self.get_odom_transform()
            
            if odom_rotation is not None:
                # Calculate and publish error between lidar and odometry
                rotation_error = lidar_rotation - odom_rotation
                translation_error = lidar_translation - odom_translation
                
                # Publish results
                self.publish_transform(lidar_rotation, lidar_translation, 
                                    scan_msg.header.stamp, is_error=False)
                self.publish_transform(rotation_error, translation_error, 
                                    scan_msg.header.stamp, is_error=True)
                
                # Store samples
                self.rotation_samples.append(rotation_error)
                self.translation_x_samples.append(translation_error[0])
                self.translation_y_samples.append(translation_error[1])
                
                # Check if we've collected enough samples for this run
                if len(self.rotation_samples) >= self.num_samples:
                    self.is_collecting = False
                    self.calculate_calibration_statistics()
        
        # Update previous scan
        self.prev_scan = current_points
        self.prev_time = scan_msg.header.stamp

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
