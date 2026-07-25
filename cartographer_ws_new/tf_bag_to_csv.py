#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import numpy as np
import csv
from tf2_msgs.msg import TFMessage

class TFBagProcessor(Node):
    def __init__(self):
        super().__init__('tf_bag_processor')
        self.poses = []
        
    def process_bag(self, bag_path, output_csv='ground_truth_from_tf.csv'):
        """Extract ground truth from recorded TF bag file"""
        
        # Open bag file
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
        
        # Get topic types
        topic_types = reader.get_all_topics_and_types()
        type_map = {topic.name: topic.type for topic in topic_types}
        
        self.get_logger().info(f"Processing bag file: {bag_path}")
        self.get_logger().info(f"Available topics: {list(type_map.keys())}")
        
        # Process messages
        while reader.has_next():
            (topic, data, timestamp) = reader.read_next()
            
            if topic == '/tf':
                # Deserialize TF message
                msg_type = get_message(type_map[topic])
                tf_msg = deserialize_message(data, msg_type)
                
                # Process each transform in the message
                for transform in tf_msg.transforms:
                    if (transform.header.frame_id == 'map' and 
                        transform.child_frame_id == 'base_link'):
                        
                        # Extract pose data
                        x = transform.transform.translation.x
                        y = transform.transform.translation.y
                        z = transform.transform.translation.z
                        
                        qx = transform.transform.rotation.x
                        qy = transform.transform.rotation.y
                        qz = transform.transform.rotation.z
                        qw = transform.transform.rotation.w
                        
                        # Convert to yaw
                        yaw = np.arctan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
                        
                        # Use message timestamp
                        msg_time = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
                        
                        # Store pose
                        pose_data = [msg_time, x, y, z, qx, qy, qz, qw, yaw]
                        self.poses.append(pose_data)
        
        # Sort by timestamp
        self.poses.sort(key=lambda x: x[0])
        
        # Save to CSV
        self.save_csv(output_csv)
        
        self.get_logger().info(f"Extracted {len(self.poses)} poses from TF data")
        
    def save_csv(self, filename):
        """Save poses to CSV file"""
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'yaw'])
            writer.writerows(self.poses)
        
        if len(self.poses) > 0:
            duration = self.poses[-1][0] - self.poses[0][0]
            avg_rate = len(self.poses) / duration if duration > 0 else 0
            self.get_logger().info(f"Saved to {filename}")
            self.get_logger().info(f"Duration: {duration:.2f} seconds, Avg rate: {avg_rate:.2f} Hz")
        else:
            self.get_logger().warn("No poses found in TF data!")

def main():
    rclpy.init()
    processor = TFBagProcessor()
    
    # Process the recorded TF bag file
    bag_path = input("Enter path to slam_trajectory.bag: ").strip()
    if not bag_path:
        bag_path = "/home/sneha/slam_trajectory.bag"
    
    try:
        processor.process_bag(bag_path)
    except Exception as e:
        processor.get_logger().error(f"Error processing bag: {e}")
    
    processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()