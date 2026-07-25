#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.executors import MultiThreadedExecutor
import subprocess
import signal
import os
import time
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class TurtleBot4WaypointRecorder(Node):
    def __init__(self):
        super().__init__('turtlebot4_waypoint_recorder')
        
        # Navigation action client
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Odometry subscriber to monitor robot position
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Current robot pose
        self.current_pose = None
        
        # Rosbag process
        self.rosbag_process = None
        
        # Define waypoints using full pose format
        # Modify these coordinates based on your environment
        # Note: These waypoints are relative to the robot's current position
        self.waypoints = [
            # {
            #     #initial
            #     'position': {'x': -0.03712274879217148, 'y':  0.10597396641969681, 'z':  -0.003421567613258958},
            #     'orientation': {'x': 0.00233858497813344, 'y':  -0.0022999995853751898, 'z':  -0.08764772117137909, 'w':  0.9961460828781128}
            # },
            {
                'position': {'x':  1.863441824913025, 'y':  -0.3247385323047638, 'z':  0.013041221536695957},
                'orientation': {'x': 0.004019924905151129, 'y':  -0.0029443297535181046, 'z':  -0.20495891571044922, 'w':  0.9787577986717224}
            },
            {
                'position': {'x': 1.794468641281128, 'y':  -0.610884964466095, 'z': 0.014712615869939327},
                'orientation': {'x':  -0.008062904700636864, 'y':  0.00021613109856843948, 'z': -0.7896281480789185, 'w':0.6135326027870178}
            },
            {
                'position': {'x': 1.8273277282714844, 'y':  -0.6264134049415588, 'z':  0.01438692957162857},
                'orientation': {'x':  0.005382736213505268, 'y':  -0.002958216704428196, 'z':0.9956531524658203, 'w':0.09293625503778458}
            },
            {
                'position': {'x':  -0.1364070326089859, 'y':  -0.28793975710868835, 'z':  0.03784285858273506},
                'orientation': {'x':  0.00462363101541996, 'y': -0.009557750076055527, 'z':  0.9974824786186218, 'w': 0.07011337578296661}
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
        
    def odom_callback(self, msg):
        """Callback to store current robot pose from odometry"""
        self.current_pose = msg.pose.pose
        
    def get_current_position_info(self):
        """Get formatted string of current robot position"""
        if self.current_pose:
            pos = self.current_pose.position
            ori = self.current_pose.orientation
            return (f"Current position: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.6f}, "
                   f"orientation: x={ori.x:.3f}, y={ori.y:.3f}, z={ori.z:.3f}, w={ori.w:.3f}")
        return "Current position: Unknown"
        
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
        self.get_logger().info(self.get_current_position_info())
        
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
        
        # Wait for initial odometry data
        self.get_logger().info('Waiting for odometry data...')
        while self.current_pose is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.current_pose is None:
            self.get_logger().error('Could not get initial robot position from odometry')
            return False
            
        self.get_logger().info(self.get_current_position_info())
        
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