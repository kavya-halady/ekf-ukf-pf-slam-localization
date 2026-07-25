#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
import csv
import numpy as np
from rclpy.duration import Duration

class SimpleTFExtractor(Node):
    def __init__(self):
        super().__init__('simple_tf_extractor')
        
        # TF setup
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.poses = []
        self.timer = self.create_timer(0.1, self.extract_transform)  # 10Hz
        
        self.get_logger().info("Simple TF extractor started. Play your slam_trajectory.bag file now!")
        
    def extract_transform(self):
        try:
            # Look for map -> base_link transform
            if self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time()):
                transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                
                # Extract data
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                z = transform.transform.translation.z
                
                qx = transform.transform.rotation.x
                qy = transform.transform.rotation.y
                qz = transform.transform.rotation.z
                qw = transform.transform.rotation.w
                
                # Convert to yaw
                yaw = np.arctan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
                
                # Use transform timestamp
                timestamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
                
                # Store
                pose_data = [timestamp, x, y, z, qx, qy, qz, qw, yaw]
                self.poses.append(pose_data)
                
                if len(self.poses) % 50 == 0:
                    self.get_logger().info(f"Extracted {len(self.poses)} poses")
                    
        except Exception as e:
            pass  # Ignore errors, just keep trying
    
    def save_poses(self, filename='ground_truth_from_tf.csv'):
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'yaw'])
            writer.writerows(self.poses)
        
        self.get_logger().info(f"Saved {len(self.poses)} poses to {filename}")

def main():
    rclpy.init()
    extractor = SimpleTFExtractor()
    
    try:
        rclpy.spin(extractor)
    except KeyboardInterrupt:
        extractor.save_poses()
    
    extractor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()