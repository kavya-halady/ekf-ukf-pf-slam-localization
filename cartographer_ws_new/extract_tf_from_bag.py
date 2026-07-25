#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, TopicFilter
from tf2_msgs.msg import TFMessage
import tf_transformations
import csv

class TFExtractor(Node):
    def __init__(self):
        super().__init__('tf_extractor')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.poses = []

    def extract_from_bag(self, bag_path):
        reader = SequentialReader()
        storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
        converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
        reader.open(storage_options, converter_options)
        reader.set_filter(TopicFilter(topics=['/tf', '/tf_static']))

        while reader.has_next():
            topic, data, t = reader.read_next()
            msg = TFMessage()
            msg.deserialize(data)
            for transform in msg.transforms:
                if transform.child_frame_id == 'base_link' and transform.header.frame_id in ['map', 'odom']:
                    ts = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
                    x = transform.transform.translation.x
                    y = transform.transform.translation.y
                    z = transform.transform.translation.z
                    qx = transform.transform.rotation.x
                    qy = transform.transform.rotation.y
                    qz = transform.transform.rotation.z
                    qw = transform.transform.rotation.w
                    yaw = tf_transformations.euler_from_quaternion([qx, qy, qz, qw])[2]
                    self.poses.append([ts, x, y, z, qx, qy, qz, qw, yaw])

        self.save_to_csv()

    def save_to_csv(self):
        with open('extracted_poses.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'yaw'])
            writer.writerows(self.poses)
        self.get_logger().info(f"Saved {len(self.poses)} poses to extracted_poses.csv")

def main():
    rclpy.init()
    node = TFExtractor()
    node.extract_from_bag('/home/sneha/localization_ws/data/tb4_data_with_tf_static/tb4_data_with_tf_static_new/tb4_data_with_tf_static_new_0.db3')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
