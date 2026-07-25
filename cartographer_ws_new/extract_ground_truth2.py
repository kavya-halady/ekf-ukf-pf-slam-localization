#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
import numpy as np
import csv
import time
from rclpy.duration import Duration

class GroundTruthExtractor(Node):
    def __init__(self):
        super().__init__('ground_truth_extractor')
        
        # TF listener with longer buffer
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Storage for poses
        self.poses = []
        self.last_successful_time = None
        
        # Timer to extract poses (lower frequency to avoid issues)
        self.timer = self.create_timer(0.2, self.extract_pose)  # 5 Hz
        
        # Counter for debugging
        self.attempt_count = 0
        self.success_count = 0
        
        self.get_logger().info("Ground truth extractor started - waiting for transforms...")
        
        # Wait a bit for transforms to be available
        time.sleep(2.0)
    
    def extract_pose(self):
        self.attempt_count += 1
        
        try:
            # Try multiple time options
            time_options = [
                rclpy.time.Time(),  # Latest available
                rclpy.time.Time(seconds=0),  # Zero time (latest)
            ]
            
            transform = None
            for time_opt in time_options:
                try:
                    if self.tf_buffer.can_transform('map', 'base_link', time_opt, Duration(seconds=0.1)):
                        transform = self.tf_buffer.lookup_transform('map', 'base_link', time_opt)
                        break
                except Exception as e:
                    continue
            
            if transform is None:
                if self.attempt_count % 50 == 0:  # Log every 10 seconds
                    self.get_logger().warn(f"No transform available (attempt {self.attempt_count})")
                return
            
            # Extract position and orientation
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            
            # Convert to euler angles
            yaw = np.arctan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
            
            # Use current time if transform time is invalid
            current_time = self.get_clock().now()
            timestamp = current_time.nanoseconds * 1e-9
            
            # Store data
            pose_data = [timestamp, x, y, z, qx, qy, qz, qw, yaw]
            self.poses.append(pose_data)
            self.success_count += 1
            
            if self.success_count % 10 == 0:
                self.get_logger().info(
                    f"Extracted {self.success_count} poses. Latest: x={x:.3f}, y={y:.3f}, yaw={np.degrees(yaw):.1f}°"
                )
            
        except Exception as e:
            if self.attempt_count % 50 == 0:
                self.get_logger().debug(f"Transform lookup failed: {str(e)}")
    
    def save_ground_truth(self, filename='cartographer_ground_truth_new.csv'):
        """Save ground truth to CSV file"""
        if len(self.poses) == 0:
            self.get_logger().error("No poses to save! Check if Cartographer is running and map frame exists.")
            return
            
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'yaw'])
            writer.writerows(self.poses)
        
        self.get_logger().info(f"Saved {len(self.poses)} poses to {filename}")
        
        # Print some statistics
        if len(self.poses) > 1:
            poses_array = np.array(self.poses)
            duration = poses_array[-1, 0] - poses_array[0, 0]
            self.get_logger().info(f"Trajectory duration: {duration:.2f} seconds")
            self.get_logger().info(f"Average sampling rate: {len(self.poses)/duration:.2f} Hz")

def main():
    rclpy.init()
    extractor = GroundTruthExtractor()
    
    try:
        rclpy.spin(extractor)
    except KeyboardInterrupt:
        extractor.get_logger().info("Shutting down...")
    finally:
        extractor.save_ground_truth()
        extractor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()