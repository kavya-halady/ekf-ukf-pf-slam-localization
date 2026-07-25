import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
import csv
import time

class TFLogger(Node):
    def __init__(self):
        super().__init__('tf_logger')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.log_tf)  # 10 Hz
        self.file = open('speed_linear_1.6.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.file)
        self.csv_writer.writerow(['time', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])

    def log_tf(self):
        try:
            now = rclpy.time.Time()
            trans: TransformStamped = self.tf_buffer.lookup_transform('map', 'base_link', now)
            t = trans.transform.translation
            r = trans.transform.rotation
            self.csv_writer.writerow([
                trans.header.stamp.sec + trans.header.stamp.nanosec * 1e-9,
                t.x, t.y, t.z,
                r.x, r.y, r.z, r.w
            ])
        except Exception as e:
            self.get_logger().warn(f"Could not transform: {e}")

    def destroy_node(self):
        self.file.close()
        super().destroy_node()

def main():
    rclpy.init()
    node = TFLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
