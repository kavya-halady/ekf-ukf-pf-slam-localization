#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node 
import sqlite3 
import struct 
from rclpy.serialization import deserialize_message 
from rosidl_runtime_py.utilities import get_message 
import numpy as np 
import csv 
import tf_transformations 
from geometry_msgs.msg import TransformStamped 
from tf2_msgs.msg import TFMessage

class TFGroundTruthExtractor: 
    def __init__(self, bag_path): 
        self.bag_path = bag_path 
        self.poses = []

    def extract_poses_from_bag(self):
        """Extract map->base_link transforms from the bag file"""
        
        # Connect to the SQLite database
        conn = sqlite3.connect(self.bag_path)
        cursor = conn.cursor()
        
        # Get all TF messages
        cursor.execute("""
            SELECT messages.timestamp, messages.data 
            FROM messages 
            JOIN topics ON messages.topic_id = topics.id 
            WHERE topics.name = '/tf'
            ORDER BY messages.timestamp
        """)
        
        tf_msg_type = get_message('tf2_msgs/msg/TFMessage')
        
        for timestamp_ns, data in cursor.fetchall():
            # Deserialize the message
            tf_msg = deserialize_message(data, tf_msg_type)
            
            # Look for map->base_link transform
            for transform in tf_msg.transforms:
                print(f"Found transform: {transform.header.frame_id} -> {transform.child_frame_id}")
                if (transform.header.frame_id == 'map' and 
                    transform.child_frame_id == 'base_link'):
                    
                    # Convert timestamp from nanoseconds to seconds
                    timestamp = timestamp_ns / 1e9
                    
                    # Extract position
                    x = transform.transform.translation.x
                    y = transform.transform.translation.y
                    z = transform.transform.translation.z
                    
                    # Extract quaternion
                    qx = transform.transform.rotation.x
                    qy = transform.transform.rotation.y
                    qz = transform.transform.rotation.z
                    qw = transform.transform.rotation.w
                    
                    # Convert quaternion to euler angles
                    euler = tf_transformations.euler_from_quaternion([qx, qy, qz, qw])
                    roll, pitch, yaw = euler
                    
                    # Store the pose
                    pose_data = [timestamp, x, y, z, qx, qy, qz, qw, roll, pitch, yaw]
                    self.poses.append(pose_data)
                    
                    print(f"Extracted pose: t={timestamp:.3f}, x={x:.3f}, y={y:.3f}, yaw={np.degrees(yaw):.1f}°")
        
        conn.close()
        print(f"\nExtracted {len(self.poses)} poses from bag file")
        
    def save_ground_truth(self, output_file='cartographer_ground_truth.csv'):
        """Save extracted poses to CSV file"""
        
        if not self.poses:
            print("No poses to save! Run extract_poses_from_bag() first.")
            return
            
        with open(output_file, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            # Write header
            writer.writerow(['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'roll', 'pitch', 'yaw'])
            # Write poses
            writer.writerows(self.poses)
        
        print(f"Saved {len(self.poses)} poses to {output_file}")
        
    def validate_extraction(self):
        """Basic validation of extracted data"""
        
        if not self.poses:
            print("No poses to validate!")
            return
            
        poses_array = np.array(self.poses)
        
        print(f"\n=== Ground Truth Validation ===")
        print(f"Total poses: {len(self.poses)}")
        print(f"Time span: {poses_array[-1, 0] - poses_array[0, 0]:.2f} seconds")
        print(f"X range: {poses_array[:, 1].min():.3f} to {poses_array[:, 1].max():.3f} m")
        print(f"Y range: {poses_array[:, 2].min():.3f} to {poses_array[:, 2].max():.3f} m")
        print(f"Total distance traveled: {self._calculate_total_distance():.3f} m")
        
        # Check for reasonable sampling rate
        timestamps = poses_array[:, 0]
        dt = np.diff(timestamps)
        avg_dt = np.mean(dt)
        sampling_rate = 1.0 / avg_dt if avg_dt > 0 else 0
        
        print(f"Average sampling rate: {sampling_rate:.1f} Hz")
        
        if sampling_rate < 1.0:
            print("⚠️  Warning: Low sampling rate. Consider interpolation for smoother trajectory.")
        elif sampling_rate > 100.0:
            print("⚠️  Warning: Very high sampling rate. Consider downsampling for efficiency.")
        else:
            print("✓ Good sampling rate for ground truth.")
            
    def _calculate_total_distance(self):
        """Calculate total distance traveled"""
        if len(self.poses) < 2:
            return 0.0
            
        poses_array = np.array(self.poses)
        x = poses_array[:, 1]
        y = poses_array[:, 2]
        
        dx = np.diff(x)
        dy = np.diff(y)
        distances = np.sqrt(dx**2 + dy**2)
        
        return np.sum(distances)
def main(): # Initialize ROS 2 (needed for message deserialization) 
    rclpy.init()

    try:
        # Path to your bag file directory
        bag_path = "/home/sneha/cartographer_ws/slam_trajectoryyyyy.bag/slam_trajectoryyyyy.bag_0.db3"  # Adjust this path as needed
        
        print("Extracting ground truth from TF bag file...")
        extractor = TFGroundTruthExtractor(bag_path)
        
        # Extract poses from the bag
        extractor.extract_poses_from_bag()
        
        # Validate the extraction
        extractor.validate_extraction()
        
        # Save to CSV
        extractor.save_ground_truth('cartographer_ground_truth_new_new.csv')
        
        print("\n✓ Ground truth extraction completed successfully!")
        print("Next steps:")
        print("1. Run the validation script to visualize your trajectory")
        print("2. Compare with your PF/EKF/UKF results using the RMSE script")
        
    except Exception as e:
        print(f"Error: {e}")
        print("\nTroubleshooting tips:")
        print("1. Make sure the bag path is correct")
        print("2. Ensure you have the required ROS 2 packages installed")
        print("3. Check that the bag file contains map->base_link transforms")
        
    finally:
        rclpy.shutdown()
if __name__ == '__main__': 
    main()