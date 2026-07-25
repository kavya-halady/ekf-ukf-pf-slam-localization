#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.executors import MultiThreadedExecutor
import subprocess
import signal
import os
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class TurtleBot4WaypointRecorder(Node):
    def __init__(self):
        super().__init__('turtlebot4_waypoint_recorder')
        
        # Navigation action client
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Rosbag process
        self.rosbag_process = None
        
        # Define waypoints using full pose format
        # Modify these coordinates based on your environment
        self.waypoints = [
            {
                'position': {'x': -7.60896, 'y': -2.58193, 'z': -2.51596e-05},
                'orientation': {'x': 0.00171059, 'y': -0.0047323, 'z': -0.0237868, 'w': 0.999704}
            },
            {
                'position': {'x': -5.0, 'y': -2.0, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.707, 'w': 0.707}
            },
            {
                'position': {'x': -5.0, 'y': 0.0, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 1.0, 'w': 0.0}
            },
            {
                'position': {'x': -7.0, 'y': 0.0, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': -0.707, 'w': 0.707}
            }
        ]
        
        self.current_waypoint = 0
        self.mission_complete = False
        
        # Topics to record in rosbag
        self.topics_to_record = [
            '/imu',
            '/tf',
            '/tf_static',
            '/odom',
            '/scan'
        ]
        
        self.get_logger().info('TurtleBot4 Waypoint Recorder initialized')
        
    def start_rosbag_recording(self):
        """Start rosbag recording"""
        try:
            # Create rosbag directory if it doesn't exist
            os.makedirs('rosbags', exist_ok=True)
            
            # Generate timestamp for unique filename
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            bag_name = f'rosbags/turtlebot4_mission_{timestamp}'
            
            # Build rosbag command
            cmd = ['ros2', 'bag', 'record', '-o', bag_name] + self.topics_to_record
            
            self.get_logger().info(f'Starting rosbag recording: {bag_name}')
            self.get_logger().info(f'Recording topics: {", ".join(self.topics_to_record)}')
            
            # Start rosbag process
            self.rosbag_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            
            # Wait a moment for rosbag to initialize
            time.sleep(2.0)
            self.get_logger().info('Rosbag recording started successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to start rosbag recording: {str(e)}')
            return False
            
        return True
    
    def stop_rosbag_recording(self):
        """Stop rosbag recording"""
        if self.rosbag_process:
            try:
                self.get_logger().info('Stopping rosbag recording...')
                
                # Send SIGINT to the process group
                os.killpg(os.getpgid(self.rosbag_process.pid), signal.SIGINT)
                
                # Wait for process to terminate
                self.rosbag_process.wait(timeout=10)
                self.get_logger().info('Rosbag recording stopped successfully')
                
            except subprocess.TimeoutExpired:
                self.get_logger().warning('Rosbag process did not terminate gracefully, forcing kill')
                os.killpg(os.getpgid(self.rosbag_process.pid), signal.SIGKILL)
            except Exception as e:
                self.get_logger().error(f'Error stopping rosbag: {str(e)}')
            finally:
                self.rosbag_process = None
    
    def create_pose_stamped(self, pose_dict):
        """Create a PoseStamped message from pose dictionary"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        pose.pose.position.x = pose_dict['position']['x']
        pose.pose.position.y = pose_dict['position']['y']
        pose.pose.position.z = pose_dict['position']['z']
        
        # Set orientation
        pose.pose.orientation.x = pose_dict['orientation']['x']
        pose.pose.orientation.y = pose_dict['orientation']['y']
        pose.pose.orientation.z = pose_dict['orientation']['z']
        pose.pose.orientation.w = pose_dict['orientation']['w']
        
        return pose
    
    def send_goal(self, pose):
        """Send navigation goal"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        self.get_logger().info(f'Navigating to waypoint {self.current_waypoint + 1}: '
                              f'x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}, z={pose.pose.position.z:.6f}')
        
        # Wait for action server
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Navigation action server not available!')
            return False
        
        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        return True
    
    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
        
        self.get_logger().info('Goal accepted, navigating...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining
        if distance > 0.1:  # Only log if distance is significant
            self.get_logger().info(f'Distance remaining: {distance:.2f}m')
    
    def get_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        
        if result:
            self.get_logger().info(f'Successfully reached waypoint {self.current_waypoint + 1}!')
            self.current_waypoint += 1
            
            # Check if mission is complete
            if self.current_waypoint >= len(self.waypoints):
                self.get_logger().info('Mission complete! All waypoints reached.')
                self.mission_complete = True
                self.stop_rosbag_recording()
            else:
                # Navigate to next waypoint
                self.navigate_to_next_waypoint()
        else:
            self.get_logger().error(f'Failed to reach waypoint {self.current_waypoint + 1}')
            # Optionally retry or abort mission
            self.stop_rosbag_recording()
    
    def navigate_to_next_waypoint(self):
        """Navigate to the next waypoint"""
        if self.current_waypoint < len(self.waypoints):
            waypoint = self.waypoints[self.current_waypoint]
            pose = self.create_pose_stamped(waypoint)
            self.send_goal(pose)
    
    def start_mission(self):
        """Start the waypoint navigation mission"""
        self.get_logger().info('Starting TurtleBot4 mission...')
        
        # Start rosbag recording
        if not self.start_rosbag_recording():
            self.get_logger().error('Failed to start rosbag recording. Aborting mission.')
            return False
        
        # Start navigation to first waypoint
        self.navigate_to_next_waypoint()
        return True

def main(args=None):
    rclpy.init(args=args)
    
    # Create node
    waypoint_recorder = TurtleBot4WaypointRecorder()
    
    # Use MultiThreadedExecutor for better performance
    executor = MultiThreadedExecutor()
    executor.add_node(waypoint_recorder)
    
    try:
        # Start the mission
        if waypoint_recorder.start_mission():
            waypoint_recorder.get_logger().info('Mission started. Press Ctrl+C to stop.')
            
            # Spin until mission is complete or interrupted
            while rclpy.ok() and not waypoint_recorder.mission_complete:
                executor.spin_once(timeout_sec=0.1)
        
    except KeyboardInterrupt:
        waypoint_recorder.get_logger().info('Mission interrupted by user')
    except Exception as e:
        waypoint_recorder.get_logger().error(f'Mission failed: {str(e)}')
    finally:
        # Cleanup
        waypoint_recorder.stop_rosbag_recording()
        waypoint_recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()