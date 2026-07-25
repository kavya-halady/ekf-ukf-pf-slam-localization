#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped, PoseStamped
import numpy as np
import csv
import time

class GroundTruthExtractor(Node):
    def __init__(self):
        super().__init__('ground_truth_extractor')
        
        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Storage for poses
        self.poses = []
        self.timestamps = []
        
        # Timer to extract poses
        self.timer = self.create_timer(0.1, self.extract_pose)  # 10 Hz
        
        self.get_logger().info("Ground truth extractor started")
    
    def extract_pose(self):
        try:
            # Get transform from map to base_link
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"Transform lookup failed: {e}")

            
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
            
            # Store data
            timestamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
            self.poses.append([timestamp, x, y, z, qx, qy, qz, qw, yaw])
            
            self.get_logger().info(f"Pose: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")
            
        except Exception as e:
            self.get_logger().debug(f"Transform lookup failed: {e}")
    
    def save_ground_truth(self, filename):
        """Save ground truth to CSV file"""
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'yaw'])
            writer.writerows(self.poses)
        
        self.get_logger().info(f"Saved {len(self.poses)} poses to {filename}")

def main():
    rclpy.init()
    extractor = GroundTruthExtractor()
    
    try:
        rclpy.spin(extractor)
    except KeyboardInterrupt:
        extractor.save_ground_truth('cartographer_ground_truth_new.csv')
    
    extractor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()