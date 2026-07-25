import math
import matplotlib.pyplot as plt
import numpy as np
import scipy.linalg
import argparse
import yaml
import os
import transformations as tf_transformations
from PIL import Image

from scipy.spatial.transform import Rotation as Rot
import csv
import os

# Initialize writer globally (once)

import numpy as np
from scipy.spatial.distance import euclidean

def calculate_rmse(ground_truth_data, ukf_estimates, times):
    """
    Calculate RMSE between ground truth and UKF estimates
    
    Args:
        ground_truth_data: List of ground truth dictionaries with 'time', 'position', 'yaw'
        ukf_estimates: numpy array of UKF state estimates [x, y, yaw, v]
        times: List of timestamps corresponding to UKF estimates
    
    Returns:
        dict: Dictionary containing RMSE values for position, x, y, and yaw
    """
    if not ground_truth_data or len(times) == 0:
        print("No data available for RMSE calculation")
        return None
    
    # Interpolate ground truth to match UKF timestamps
    gt_positions_interp = []
    gt_yaw_interp = []
    
    # Convert ground truth to arrays for easier processing
    gt_times = np.array([gt['time'] for gt in ground_truth_data])
    gt_x = np.array([gt['position'][0] for gt in ground_truth_data])
    gt_y = np.array([gt['position'][1] for gt in ground_truth_data])
    gt_yaw = np.array([gt['yaw'] for gt in ground_truth_data])
    
    # Interpolate ground truth to UKF timestamps
    for ukf_time in times:
        # Find the time range in ground truth
        if ukf_time < gt_times[0] or ukf_time > gt_times[-1]:
            continue  # Skip if outside ground truth time range
        
        # Linear interpolation
        interp_x = np.interp(ukf_time, gt_times, gt_x)
        interp_y = np.interp(ukf_time, gt_times, gt_y)
        # interp_yaw = np.interp(ukf_time, gt_times, gt_yaw)
        interp_yaw = np.arctan2(
            np.interp(ukf_time, gt_times, np.sin(gt_yaw)),
            np.interp(ukf_time, gt_times, np.cos(gt_yaw))
        )
    

        
        gt_positions_interp.append([interp_x, interp_y])
        gt_yaw_interp.append(interp_yaw)
    
    if len(gt_positions_interp) == 0:
        print("No overlapping timestamps between ground truth and UKF estimates")
        return None
    
    # Convert to numpy arrays
    gt_positions_interp = np.array(gt_positions_interp)
    gt_yaw_interp = np.array(gt_yaw_interp)
    
    # Match UKF estimates to the same time range
    min_length = min(len(gt_positions_interp), ukf_estimates.shape[1])
    gt_positions_interp = gt_positions_interp[:min_length]
    gt_yaw_interp = gt_yaw_interp[:min_length]
    ukf_positions = ukf_estimates[:2, :min_length].T  # [x, y] positions
    ukf_yaw = ukf_estimates[2, :min_length]  # yaw angles
    
    # Calculate position RMSE (Euclidean distance)
    position_errors = []
    x_errors = []
    y_errors = []
    
    for i in range(min_length):
        pos_error = euclidean(gt_positions_interp[i], ukf_positions[i])
        position_errors.append(pos_error)
        x_errors.append(gt_positions_interp[i][0] - ukf_positions[i][0])
        y_errors.append(gt_positions_interp[i][1] - ukf_positions[i][1])
    
    # Calculate yaw RMSE (handle angle wraparound)
    yaw_errors = []
    for i in range(min_length):
        yaw_error = gt_yaw_interp[i] - ukf_yaw[i]
        # Normalize angle difference to [-pi, pi]
        yaw_error = np.arctan2(np.sin(yaw_error), np.cos(yaw_error))
        yaw_errors.append(yaw_error)
    
    # Calculate RMSE values
    rmse_position = np.sqrt(np.mean(np.array(position_errors)**2))
    rmse_x = np.sqrt(np.mean(np.array(x_errors)**2))
    rmse_y = np.sqrt(np.mean(np.array(y_errors)**2))
    rmse_yaw = np.sqrt(np.mean(np.array(yaw_errors)**2))
    
    # Calculate additional statistics
    mae_position = np.mean(np.abs(position_errors))  # Mean Absolute Error
    max_position_error = np.max(np.abs(position_errors))  # Maximum error
    
    results = {
        'rmse_position': rmse_position,
        'rmse_x': rmse_x,
        'rmse_y': rmse_y,
        'rmse_yaw': rmse_yaw,
        'mae_position': mae_position,
        'max_position_error': max_position_error,
        'num_compared_points': min_length,
        'mean_x_error': np.mean(x_errors),
        'mean_y_error': np.mean(y_errors),
        'std_x_error': np.std(x_errors),
        'std_y_error': np.std(y_errors)
    }
    
    return results

def print_rmse_results(rmse_results):
    """Print RMSE results in a formatted way"""
    if rmse_results is None:
        print("No RMSE results to display")
        return
    
    print("\n" + "="*50)
    print("           RMSE ANALYSIS RESULTS")
    print("="*50)
    print(f"Number of compared points: {rmse_results['num_compared_points']}")
    print(f"Position RMSE:     {rmse_results['rmse_position']:.4f} m")
    print(f"X RMSE:           {rmse_results['rmse_x']:.4f} m")
    print(f"Y RMSE:           {rmse_results['rmse_y']:.4f} m")
    print(f"Yaw RMSE:         {rmse_results['rmse_yaw']:.4f} rad ({np.rad2deg(rmse_results['rmse_yaw']):.2f} deg)")
    print(f"Position MAE:      {rmse_results['mae_position']:.4f} m")
    print(f"Max Position Error: {rmse_results['max_position_error']:.4f} m")
    print(f"Mean X Error:      {rmse_results['mean_x_error']:.4f} ± {rmse_results['std_x_error']:.4f} m")
    print(f"Mean Y Error:      {rmse_results['mean_y_error']:.4f} ± {rmse_results['std_y_error']:.4f} m")
    print("="*50)

def save_rmse_results(rmse_results, output_file="rmse_results.txt"):
    """Save RMSE results to a text file"""
    if rmse_results is None:
        print("No RMSE results to save")
        return
    
    with open(output_file, 'w') as f:
        f.write("UKF vs Ground Truth RMSE Analysis Results\n")
        f.write("="*50 + "\n")
        f.write(f"Number of compared points: {rmse_results['num_compared_points']}\n")
        f.write(f"Position RMSE: {rmse_results['rmse_position']:.6f} m\n")
        f.write(f"X RMSE: {rmse_results['rmse_x']:.6f} m\n")
        f.write(f"Y RMSE: {rmse_results['rmse_y']:.6f} m\n")
        f.write(f"Yaw RMSE: {rmse_results['rmse_yaw']:.6f} rad ({np.rad2deg(rmse_results['rmse_yaw']):.4f} deg)\n")
        f.write(f"Position MAE: {rmse_results['mae_position']:.6f} m\n")
        f.write(f"Max Position Error: {rmse_results['max_position_error']:.6f} m\n")
        f.write(f"Mean X Error: {rmse_results['mean_x_error']:.6f} ± {rmse_results['std_x_error']:.6f} m\n")
        f.write(f"Mean Y Error: {rmse_results['mean_y_error']:.6f} ± {rmse_results['std_y_error']:.6f} m\n")
    
    print(f"RMSE results saved to {output_file}")


# Covariance for UKF simulation
Q = np.diag([
    0.1,  # variance of location on x-axis
    0.1,  # variance of location on y-axis
    np.deg2rad(1.0),  # variance of yaw angle
    1.0  # variance of velocity
]) ** 2  # predict state covariance
R = np.diag([1.0, 1.0]) ** 2  # Observation x,y position covariance

# UKF parameters
ALPHA = 0.001
BETA = 2
KAPPA = 0

# Simulation parameter
DT = 0.1  # time tick [s]
show_animation = True

def motion_model(x, u, dt=DT):
    """
    Motion Model
    x{t+1} = xt + vdtcos(yaw)
    y{t+1} = yt + vdtsin(yaw)
    yaw{t+1} = yawt + omega*dt
    v{t+1} = v{t}
    """
    F = np.array([[1.0, 0, 0, 0],
                  [0, 1.0, 0, 0],
                  [0, 0, 1.0, 0],
                  [0, 0, 0, 0]])

    B = np.array([[dt * math.cos(x[2, 0]), 0],
                  [dt * math.sin(x[2, 0]), 0],
                  [0.0, dt],
                  [1.0, 0.0]])

    x = F @ x + B @ u

    return x

def observation_model(x):
    """
    Observation Model
    """
    H = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    z = H @ x

    return z

def generate_sigma_points(xEst, PEst, gamma):
    """
    Generate sigma points
    """
    sigma = xEst
    Psqrt = scipy.linalg.sqrtm(PEst)
    n = len(xEst[:, 0])
    # Positive direction
    for i in range(n):
        sigma = np.hstack((sigma, xEst + gamma * Psqrt[:, i:i + 1]))

    # Negative direction
    for i in range(n):
        sigma = np.hstack((sigma, xEst - gamma * Psqrt[:, i:i + 1]))

    return sigma

def predict_sigma_motion(sigma, u, dt=DT):
    """
    Sigma Points prediction with motion model
    """
    for i in range(sigma.shape[1]):
        sigma[:, i:i + 1] = motion_model(sigma[:, i:i + 1], u, dt)

    return sigma

def predict_sigma_observation(sigma):
    """
    Sigma Points prediction with observation model
    """
    z_sigma = np.zeros((2, sigma.shape[1]))

    for i in range(sigma.shape[1]):
        z = observation_model(sigma[:, i:i + 1])
        z_sigma[:, i] = z.flatten()

    return z_sigma

def calc_sigma_covariance(x, sigma, wc, Pi):
    """
    Calculate covariance matrix using sigma points and weights
    """
    nSigma = sigma.shape[1]
    d = sigma - x[0:sigma.shape[0]]
    P = Pi
    for i in range(nSigma):
        P = P + wc[0, i] * d[:, i:i + 1] @ d[:, i:i + 1].T
    return P

def calc_pxz(sigma, x, z_sigma, zb, wc):
    """
    Calculate cross-covariance matrix
    """
    nSigma = sigma.shape[1]
    dx = sigma - x
    dz = z_sigma - zb[0:2].reshape(2, 1)
    P = np.zeros((dx.shape[0], dz.shape[0]))

    for i in range(nSigma):
        P = P + wc[0, i] * dx[:, i:i + 1] @ dz[:, i:i + 1].T

    return P

def setup_ukf(nx):
    """
    Set up UKF parameters
    """
    lamb = ALPHA ** 2 * (nx + KAPPA) - nx
    # calculate weights
    wm = [lamb / (lamb + nx)]
    wc = [(lamb / (lamb + nx)) + (1 - ALPHA ** 2 + BETA)]
    for i in range(2 * nx):
        wm.append(1.0 / (2 * (nx + lamb)))
        wc.append(1.0 / (2 * (nx + lamb)))
    gamma = math.sqrt(nx + lamb)

    wm = np.array([wm])
    wc = np.array([wc])

    return wm, wc, gamma

def ukf_estimation(xEst, PEst, z, u, wm, wc, gamma, dt=DT):
    """
    UKF Estimation
    """
    #  Predict
    sigma = generate_sigma_points(xEst, PEst, gamma)
    sigma = predict_sigma_motion(sigma, u, dt)
    xPred = (wm @ sigma.T).T
    PPred = calc_sigma_covariance(xPred, sigma, wc, Q)

    #  Update
    sigma = generate_sigma_points(xPred, PPred, gamma)
    z_sigma = predict_sigma_observation(sigma)
    zPred = (wm @ z_sigma.T).T

    # Ensure z and zPred have the same shape
    z_reshape = z.reshape(2, 1)
    zPred_reshape = zPred.reshape(2, 1)
    y = z_reshape - zPred_reshape

    st = calc_sigma_covariance(zPred_reshape, z_sigma, wc, R)
    Pxz = calc_pxz(sigma, xPred, z_sigma, zPred_reshape, wc)
    K = Pxz @ np.linalg.inv(st)
    xEst = xPred + K @ y
    PEst = PPred - K @ st @ K.T

    return xEst, PEst

def plot_covariance_ellipse(xEst, PEst):
    """
    Plot the covariance ellipse
    """
    Pxy = PEst[0:2, 0:2]
    eigval, eigvec = np.linalg.eig(Pxy)

    if eigval[0] >= eigval[1]:
        bigind = 0
        smallind = 1
    else:
        bigind = 1
        smallind = 0

    t = np.arange(0, 2 * math.pi + 0.1, 0.1)
    a = math.sqrt(eigval[bigind])
    b = math.sqrt(eigval[smallind])
    x = [a * math.cos(it) for it in t]
    y = [b * math.sin(it) for it in t]
    angle = math.atan2(eigvec[1, bigind], eigvec[0, bigind])
    rot = Rot.from_euler('z', angle).as_matrix()[0:2, 0:2]
    fx = rot @ (np.array([x, y]))
    px = np.array(fx[0, :] + xEst[0, 0]).flatten()
    py = np.array(fx[1, :] + xEst[1, 0]).flatten()
    # plt.plot(px, py, "--r")

class OccupancyGridMap:
    """Simple occupancy grid map class for displaying with UKF"""
    def __init__(self,yaml_path = None, width=2.7, height=2.0, resolution=0.05):
    # def __init__(self,yaml_path = None, width=27.0, height=20.0, resolution=0.05):
    #     self.width = width  # meters
    #     self.height = height  # meters
    #     self.resolution = resolution  # meters per cell
    #     self.grid_width = int(width / resolution)
    #     self.grid_height = int(height / resolution)
    #     self.grid = np.zeros((self.grid_height, self.grid_width))
    #     self.origin_x = -1.5  # center the map
    #     self.origin_y = -0.2
        # yaml_path = 
        if yaml_path:
            self.load_from_yaml(yaml_path)
            print("aaaaaaaaaaaaaaaaaaaaaaaa")
        else:
    
            self.width = width # meters
            self.height = height # meters
            self.resolution = resolution # meters per cell
            self.grid_width = int(width / resolution)
            self.grid_height = int(height / resolution)
            self.grid = np.zeros((self.grid_height, self.grid_width))
            # self.origin_x = -1.897173919528723 # updated origin x
            # self.origin_y = -0.751378056406975 # updated origin y
            self.origin_x = 0.0 # updated origin x
            self.origin_y = 0.0 # updated origin y




    


    def load_from_image(self, image_path, threshold=128):
        try:
            print(f"Loading image from {image_path}")
            img = Image.open(image_path).convert('L')
            print(f"Image size: {img.size}")
            img = img.resize((self.grid_width, self.grid_height))
            print(f"Resized image size: {img.size}")
            img_array = np.array(img)
            print(f"Image array shape: {img_array.shape}")
            self.grid = np.flipud((img_array < threshold).astype(np.int8))
            print(f"Loaded map from {image_path}")
            return True
        except Exception as e:
            print(f"Failed to load map from image: {e}")
            return False



    def load_from_yaml(self, yaml_path):
        """Load map parameters from YAML file"""
        try:
            print(f"Loading YAML from {yaml_path}")
            with open(yaml_path, 'r') as f:
                map_data = yaml.safe_load(f)

            # Get map parameters
            self.resolution = map_data.get('resolution', self.resolution)
            self.origin_x = map_data.get('origin', [0, 0, 0])[0]
            self.origin_y = map_data.get('origin', [0, 0, 0])[1]

            # If yaml contains path to image file
            image_path = map_data.get('image', None)
            if image_path:
                # If the path is relative, make it relative to the yaml file
                if not os.path.isabs(image_path):
                    yaml_dir = os.path.dirname(yaml_path)
                    image_path = os.path.join(yaml_dir, image_path)

                # Update width and height based on image
                img = Image.open(image_path)
                self.width = img.width * self.resolution
                self.height = img.height * self.resolution
                self.grid_width = img.width
                self.grid_height = img.height

                return self.load_from_image(image_path)
            return False
        except Exception as e:
            print(f"Failed to load map from YAML: {e}")
            return False

    def add_walls_from_rectangle(self, x_min, y_min, x_max, y_max):
        """Add walls in a rectangular shape"""
        # Convert to grid coordinates
        gx_min = int((x_min - self.origin_x) / self.resolution)
        gy_min = int((y_min - self.origin_y) / self.resolution)
        gx_max = int((x_max - self.origin_x) / self.resolution)
        gy_max = int((y_max - self.origin_y) / self.resolution)

        # Ensure within bounds
        gx_min = max(0, min(gx_min, self.grid_width - 1))
        gy_min = max(0, min(gy_min, self.grid_height - 1))
        gx_max = max(0, min(gx_max, self.grid_width - 1))
        gy_max = max(0, min(gy_max, self.grid_height - 1))

        # Add walls (just the perimeter)
        # Bottom and top walls
        self.grid[gy_min, gx_min:gx_max+1] = 1
        self.grid[gy_max, gx_min:gx_max+1] = 1
        # Left and right walls
        self.grid[gy_min:gy_max+1, gx_min] = 1
        self.grid[gy_min:gy_max+1, gx_max] = 1

    def create_simple_map(self):
        """Create a simple map for demo purposes"""
        # Create rectangular room with walls
        self.add_walls_from_rectangle(-1.5, -0.2, 0.2, 1.3)
        # Add internal walls/obstacles
        # self.add_walls_from_rectangle(-1.0, 0.3, -0.5, 0.8)

    def build_from_scan_data(self, scan_data, robot_positions, orientation_data):
        """Build a map from LiDAR scan data and robot positions"""
        if not scan_data or not robot_positions:
            print("No scan data or position data available for map building")
            return False

        # Clear the map
        self.grid.fill(0)

        # Process each scan
        for i, scan in enumerate(scan_data):
            if i >= len(robot_positions):
                break

            # Get robot position and orientation
            robot_x, robot_y = robot_positions[i][1]
            # Find closest orientation to this scan time
            closest_yaw = 0
            min_time_diff = float('inf')
            for time_stamp, yaw in orientation_data:
                time_diff = abs(time_stamp - scan['time'])
                if time_diff < min_time_diff:
                    min_time_diff = time_diff
                    closest_yaw = yaw

            # Process each range reading in the scan
            ranges = scan['ranges']
            angle_min = scan['angle_min']
            angle_increment = scan['angle_increment']
            range_min = scan['range_min']
            range_max = scan['range_max']

            for j, r in enumerate(ranges):
                # Skip invalid readings
                if r < range_min or r > range_max or math.isnan(r) or math.isinf(r):
                    continue

                # Calculate angle for this reading
                angle = angle_min + j * angle_increment + closest_yaw

                # Calculate endpoint in world coordinates
                endpoint_x = robot_x + r * math.cos(angle)
                endpoint_y = robot_y + r * math.sin(angle)

                # Convert to grid coordinates
                grid_x = int((endpoint_x - self.origin_x) / self.resolution)
                grid_y = int((endpoint_y - self.origin_y) / self.resolution)

                # Check if within grid bounds
                if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
                    self.grid[grid_y, grid_x] = 1

        print("Built map from scan data")
        return True


    
    def plot(self, ax=None):
        if ax is None:
            ax = plt.gca()
            # Create coordinate meshgrid
        x_grid = np.arange(self.grid_width) * self.resolution + self.origin_x
        y_grid = np.arange(self.grid_height) * self.resolution + self.origin_y
        X, Y = np.meshgrid(x_grid, y_grid)
    # Plot occupied cells
        occupied = np.where(self.grid == 1)
        if len(occupied[0]) > 0:
            y_flipped = (self.grid_height - 1 - occupied[0]) * self.resolution + self.origin_y
            xcoords = occupied[1] * self.resolution + self.origin_x
            ax.scatter(X[occupied], Y[occupied], s=1, c='k', marker='s')
        return ax


class ROS2BagDataProvider:
    def __init__(self, bag_path, yaml_path=None):
        """Initialize the data provider with a ROS2 bag file"""
        self.bag_path = bag_path
        self.yaml_path = yaml_path
        self.scan_data = []
        self.odom_data = []
        self.imu_data = []
        self.tf_data = []
        self.current_time = None

        # Load YAML config if provided
        self.config = None
        if yaml_path:
            self._load_yaml_config()

        # Import ROS2 bag reading modules
        try:
            import rclpy
            from rclpy.serialization import deserialize_message
            from rosidl_runtime_py.utilities import get_message
            from rosbag2_py import StorageOptions, ConverterOptions, SequentialReader

            # Initialize ROS2 (not starting the node)
            if not rclpy.ok():
                rclpy.init()
        except ImportError:
            print("ROS2 modules not found. Cannot read bag files directly.")
            print("Attempting to use SQLite directly to read bag data...")
            self._process_bag_with_sqlite()
            return

        # Process the bag file
        self._process_bag()

    def _load_yaml_config(self):
        """Load configuration from YAML file"""
        try:
            with open(self.yaml_path, 'r') as f:
                self.config = yaml.safe_load(f)
            print(f"Loaded configuration from {self.yaml_path}")
        except Exception as e:
            print(f"Error loading YAML config: {e}")

    def _process_bag_with_sqlite(self):
        """Process bag file using direct SQLite access as fallback"""
        import sqlite3

        try:
            # Open the database
            conn = sqlite3.connect(self.bag_path)
            cursor = conn.cursor()

            # Get topics table
            cursor.execute("SELECT id, name, type FROM topics")
            topics = {row[0]: (row[1], row[2]) for row in cursor.fetchall()}

            # Print available topics
            print("Available topics:")
            for topic_id, (topic_name, topic_type) in topics.items():
                print(f"  {topic_name} ({topic_type})")

            # Get messages (simplified - this won't deserialize messages properly)
            cursor.execute("SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp")

            for topic_id, timestamp, data in cursor.fetchall():
                if topic_id in topics:
                    topic_name, topic_type = topics[topic_id]
                    time_sec = timestamp * 1e-9  # Convert nanoseconds to seconds

                    # Simple detection of message types by topic name
                    if '/scan' in topic_name:
                        # Can't deserialize data properly without ROS, so add placeholder
                        self.scan_data.append({
                            'time': time_sec,
                            'ranges': [],  # Empty ranges
                            'angle_min': 0,
                            'angle_increment': 0.01,
                            'range_min': 0.1,
                            'range_max': 10.0
                        })

                    elif '/odom' in topic_name:
                        # Add placeholder odometry data
                        self.odom_data.append({
                            'time': time_sec,
                            'position': [0, 0],  # Placeholder
                            'yaw': 0,            # Placeholder
                            'velocity': 0,       # Placeholder
                            'yaw_rate': 0        # Placeholder
                        })

            conn.close()
            print("Data partially processed from SQLite database. Note: actual message data not parsed.")
            print(f"Found {len(self.scan_data)} scan msgs, {len(self.odom_data)} odom msgs (placeholders)")

        except Exception as e:
            print(f"Error processing bag with SQLite: {e}")

    def _process_bag(self):
        """Extract all necessary data from the ROS2 bag file"""
        print(f"Processing ROS2 bag file: {self.bag_path}")

        try:
            from rosbag2_py import StorageOptions, ConverterOptions, SequentialReader
            from rclpy.serialization import deserialize_message
            from rosidl_runtime_py.utilities import get_message

            # Configure storage and converter options
            storage_options = StorageOptions(uri=self.bag_path, storage_id='sqlite3')
            converter_options = ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr')

            # Create and initialize reader
            reader = SequentialReader()
            reader.open(storage_options, converter_options)

            # Get topic types
            topic_types = reader.get_all_topics_and_types()
            type_map = {topic_info.name: topic_info.type for topic_info in topic_types}
            print(f"Available topics: {list(type_map.keys())}")

            # Create deserializer map
            deserializers = {}
            for topic_name, topic_type in type_map.items():
                try:
                    deserializers[topic_name] = get_message(topic_type)
                except (AttributeError, ModuleNotFoundError, ValueError) as e:
                    print(f"Could not import message type for {topic_name}: {e}")

            # Read all messages
            while reader.has_next():
                topic_name, data, timestamp = reader.read_next()

                # Skip if the topic isn't in our list
                if topic_name not in deserializers:
                    continue

                # Deserialize message
                msg_type = deserializers[topic_name]
                msg = deserialize_message(data, msg_type)

                # Convert nanosecond timestamp to seconds
                time_sec = timestamp * 1e-9

                # Process scan messages
                if '/scan' in topic_name:
                    self.scan_data.append({
                        'time': time_sec,
                        'ranges': np.array(msg.ranges),
                        'angle_min': msg.angle_min,
                        'angle_increment': msg.angle_increment,
                        'range_min': msg.range_min,
                        'range_max': msg.range_max
                    })

                # Process odometry messages
                elif '/odom' in topic_name:
                    # Extract position and orientation
                    position = [msg.pose.pose.position.x, msg.pose.pose.position.y]
                    quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
                    roll, pitch, yaw = tf_transformations.euler_from_quaternion(quat)

                    # Extract velocity
                    vel_linear = msg.twist.twist.linear.x
                    vel_angular = msg.twist.twist.angular.z

                    self.odom_data.append({
                        'time': time_sec,
                        'position': position,
                        'yaw': yaw,
                        'velocity': vel_linear,
                        'yaw_rate': vel_angular
                    })

            print(f"Processed {len(self.scan_data)} scan msgs, {len(self.odom_data)} odom msgs")

        except Exception as e:
            print(f"Error processing bag: {e}")

    def get_scan_data(self):
        """Return the scan data"""
        return self.scan_data

    def get_odom_data(self):
        """Return the odometry data"""
        return self.odom_data

    def get_trajectory(self):
        """Extract trajectory from odometry data"""
        trajectory = []
        times = []
        for odom in self.odom_data:
            times.append(odom['time'])
            trajectory.append((odom['time'], odom['position']))
        return times, trajectory
    
    def get_orientation_data(self):
        """Extract orientation data from odometry"""
        orientation_data = []
        for odom in self.odom_data:
            orientation_data.append((odom['time'], odom['yaw']))
        return orientation_data

    def get_control_inputs(self):
        """Extract control inputs from odometry data"""
        control_inputs = []
        times = []
        for odom in self.odom_data:
            times.append(odom['time'])
            control_inputs.append(np.array([[odom['velocity']], [odom['yaw_rate']]]))
        return times, control_inputs




def main():
    parser = argparse.ArgumentParser(description='UKF Localization with ROS2 bag data')
    parser.add_argument('--bag', type=str, required=True, help='Path to ROS2 bag file')
    parser.add_argument('--map_yaml', type=str, help='Path to map YAML file')

    parser.add_argument('--sim', action='store_true', help='Run with simulated data (instead of bag data)')
    parser.add_argument('--ground_truth', type=str, help='Path to ground truth CSV file')
    args = parser.parse_args()
    print("UKF Localization")

    # Set up figure
    plt.figure(figsize=(10, 8))

    # Create a map
    grid_map = OccupancyGridMap()

    # Try to load map from YAML if provided
    if args.map_yaml:
        success = grid_map.load_from_yaml(args.map_yaml)
        if not success:
            print("Failed to load map from YAML file.")
            return
    else:
        # Create a simple map
        grid_map.create_simple_map()

    # Debug prints to verify map loading
    print(f"Map origin: ({grid_map.origin_x}, {grid_map.origin_y})")
    print(f"Map resolution: {grid_map.resolution}")
    print(f"Map dimensions: {grid_map.grid_width} x {grid_map.grid_height}")

    # Plot the map first (this is the background)
    ax = grid_map.plot()

    # Set plot limits to match the map and trajectory
    plt.xlim(grid_map.origin_x - 0.5, grid_map.origin_x + grid_map.width + 0.5)
    plt.ylim(grid_map.origin_y - 0.5, grid_map.origin_y + grid_map.height + 0.5)

    # Set axis labels and ticks
    plt.xlabel('x in meters')
    plt.ylabel('y in meters')
    plt.xticks(np.arange(grid_map.origin_x - 0.5, grid_map.origin_x + grid_map.width + 0.5, 0.5))
    plt.yticks(np.arange(grid_map.origin_y - 0.5, grid_map.origin_y + grid_map.height + 0.5, 0.5))

    # Make sure axes are equal so circles look like circles
    plt.axis('equal')

    # Load data provider
    data_provider = ROS2BagDataProvider(args.bag, args.map_yaml)
    ground_truth_data = []
    if args.ground_truth:
        ground_truth_data = load_ground_truth_data(args.ground_truth)

    # Get trajectory and control inputs
    times, trajectory = data_provider.get_trajectory()
    _, control_inputs = data_provider.get_control_inputs()
    # Offset trajectory so it starts at (0, 0)
    # x_offset = trajectory[0][1][0]
    # y_offset = trajectory[0][1][1]

    # for i in range(len(trajectory)):
    #     t, pos = trajectory[i]
    #     trajectory[i] = (t, [pos[0] - x_offset, pos[1] - y_offset])
    # Offset trajectory so it starts at (0, 0)
    x_offset = trajectory[0][1][0]
    y_offset = trajectory[0][1][1]
    
    for i in range(len(trajectory)):
        t, pos = trajectory[i]
        trajectory[i] = (t, [pos[0] - x_offset, pos[1] - y_offset])
    
    # Also offset ground truth data to match the same coordinate system
    if ground_truth_data:
        # Find the closest ground truth point to the first trajectory point in time
        first_traj_time = trajectory[0][0]
        min_time_diff = float('inf')
        closest_gt_idx = 0
        
        for i, gt in enumerate(ground_truth_data):
            time_diff = abs(gt['time'] - first_traj_time)
            if time_diff < min_time_diff:
                min_time_diff = time_diff
                closest_gt_idx = i
        
        # Offset ground truth to match trajectory coordinate system
        if closest_gt_idx < len(ground_truth_data):
            gt_x_offset = ground_truth_data[closest_gt_idx]['position'][0]
            gt_y_offset = ground_truth_data[closest_gt_idx]['position'][1]
            
            for gt in ground_truth_data:
                gt['position'][0] = gt['position'][0] - gt_x_offset + trajectory[0][1][0]
                gt['position'][1] = gt['position'][1] - gt_y_offset + trajectory[0][1][1]
    


    # Get scan data (can be used to build map)
    scan_data = data_provider.get_scan_data()
    orientation_data = data_provider.get_orientation_data()

    # Build map from scan data if needed
    if len(scan_data) > 0 and not args.map_yaml:
        grid_map.build_from_scan_data(scan_data, trajectory, orientation_data)

    # Extract positions for observations
    z_obs = np.zeros((2, len(trajectory)))
    for i, (t, pos) in enumerate(trajectory):
        z_obs[0, i] = pos[0]  # x position
        z_obs[1, i] = pos[1]  # y position

    # Add noise to create simulated observations from real trajectory
    z_noised = z_obs + np.random.randn(*z_obs.shape) * 0.1

    # Initialize state with first position and orientation
    if len(trajectory) > 0:
        x0 = trajectory[0][1][0]
        y0 = trajectory[0][1][1]
        yaw0 = 0.0
        if len(orientation_data) > 0:
            yaw0 = orientation_data[0][1]  # Use first orientation if available

    # Initial state estimate
    xEst = np.array([[x0], [y0], [yaw0], [0.0]])
    PEst = np.eye(4)

    # UKF setup
    nx = xEst.shape[0]
    wm, wc, gamma = setup_ukf(nx)

    # Store results
    hxEst = xEst
    hxTrue = np.array([[x0], [y0], [yaw0], [0.0]])
    hz = np.zeros((2, 1))

    # Main UKF loop for real data
    for i in range(len(trajectory) - 1):
        # Get time difference between measurements
        if i < len(times) - 1:
            dt = times[i+1] - times[i]
        else:
            dt = DT  # Use default if no time difference available

        # Get control input and observation
        u = control_inputs[i]
        # z = z_noised[:, i].reshape(2, 1)
        z = z_obs[:, i].reshape(2, 1)

        # Update true state from real trajectory
        _, next_pos = trajectory[i]
        if isinstance(next_pos, list) or isinstance(next_pos, tuple):
            hxTrue = np.hstack((hxTrue, np.array([[next_pos[0]], [next_pos[1]], [0.0], [0.0]])))
        else:
            # Handle the case where next_pos is not a list/tuple
            print(f"Warning: unexpected position format at index {i}")
            # Use previous position or default
            hxTrue = np.hstack((hxTrue, hxTrue[:, -1:]))

        # Update UKF with proper time delta
        xEst, PEst = ukf_estimation(xEst, PEst, z, u, wm, wc, gamma, dt)

        # Store estimation
        hxEst = np.hstack((hxEst, xEst))
        hz = np.hstack((hz, z))

        # Plot for animation
        if show_animation and i % 10 == 0:
            plt.cla()
            # Re-plot the map
            grid_map.plot()
            # Plot true trajectory
            x_traj = []
            y_traj = []
            for _, pos in trajectory[:i+1]:
                if isinstance(pos, list) or isinstance(pos, tuple):
                    x_traj.append(pos[0])
                    y_traj.append(pos[1])
            # plt.plot(x_traj, y_traj, "-b",linewidth=5, label="DeadReckoning")
            # Plot estimated trajectory
            plt.plot(hxEst[0, :], hxEst[1, :], "-r", linewidth=4,label="UKF")
            if ground_truth_data:
                gt_x = [gt['position'][0] for gt in ground_truth_data[:i+1]]
                gt_y = [gt['position'][1] for gt in ground_truth_data[:i+1]]
                plt.plot(gt_x,gt_y,"y",linewidth=3,label="ground Truth")
            # Plot observations
            plt.plot(hxEst[0, 0], hxEst[1, 0], "o", color='green', markersize=8, label='Start')
            plt.plot(hxEst[0, -1], hxEst[1, -1], "s", color='green', markersize=8, label='End')
            # plt.plot(hz[0, :], hz[1, :], ".-g",linewidth=0.5 ,label="Observations")
            # Plot covariance ellipse
            plot_covariance_ellipse(xEst, PEst)
            plt.axis('equal')
            plt.grid(True)
            plt.title(f"UKF Localization (t={times[i]:.1f}s)")
            plt.legend()
            plt.pause(0.001)
      # Calculate RMSE if ground truth data is available
    if ground_truth_data:
        print("Calculating RMSE between UKF estimates and ground truth...")
        rmse_results = calculate_rmse(ground_truth_data, hxEst, times)
        
        if rmse_results:
            print_rmse_results(rmse_results)
            save_rmse_results(rmse_results, "ukf_rmse_results.txt")
            
            # Create error plots
            create_error_plots(ground_truth_data, hxEst, times, rmse_results)
    # Final plot
    plt.figure(figsize=(10, 8))
    grid_map.plot()
    # Extract x,y coordinates for plotting
    x_traj = []
    y_traj = []
    for _, pos in trajectory:
        if isinstance(pos, list) or isinstance(pos, tuple):
            x_traj.append(pos[0])
            y_traj.append(pos[1])
    # plt.plot(x_traj, y_traj, "-b",linewidth=5, label="Dead Reckoning")
    plt.plot(hxEst[0, :], hxEst[1, :], "-r",linewidth=4, label="UKF Estimate")
    if ground_truth_data:
        gt_x = [gt['position'][0] for gt in ground_truth_data]
        gt_y = [gt['position'][1] for gt in ground_truth_data]
        plt.plot(gt_x, gt_y,"-y",linewidth=3,label="ground Truth")
    plt.plot(hxEst[0, 0], hxEst[1, 0], "o", color='green', markersize=8, label='Start')
    plt.plot(hxEst[0, -1], hxEst[1, -1], "s", color='green', markersize=8, label='End')
    # plt.plot(hz[0, :], hz[1, :], ".-g",linewidth=0.5, label="Observations")
    # Draw final position covariance
    plot_covariance_ellipse(xEst, PEst)
    # Set axis labels and ticks for final plot
    plt.xlabel('x in meters')
    plt.ylabel('y in meters')
    plt.xticks(np.arange(grid_map.origin_x - 0.5, grid_map.origin_x + grid_map.width + 0.5, 0.5))
    plt.yticks(np.arange(grid_map.origin_y - 0.5, grid_map.origin_y + grid_map.height + 0.5, 0.5))
    plt.axis('equal')
    plt.grid(True)
    plt.title("UKF Localization with ROS2 Bag Data")
    plt.legend()
    plt.savefig("ukf_new.png", dpi=300)
    plt.show()



if __name__ == "__main__":
    def load_ground_truth_data(file_path):
        """Load ground truth data from CSV file"""
        ground_truth = []
        try:
            with open(file_path, 'r') as f:
                lines = f.readlines()
                # Skip header line
                for line in lines[1:]:
                    parts = line.strip().split(',')
                    if len(parts) >= 8:
                        time = float(parts[0])
                        x = float(parts[1])
                        y = float(parts[2])
                        z = float(parts[3])
                        qx = float(parts[4])
                        qy = float(parts[5])
                        qz = float(parts[6])
                        qw = float(parts[7])
                        
                        # Convert quaternion to yaw
                        yaw = tf_transformations.euler_from_quaternion([qx, qy, qz, qw])[2]
                        
                        ground_truth.append({
                            'time': time,
                            'position': [x, y],
                            'yaw': yaw
                        })
            print(f"Loaded {len(ground_truth)} ground truth points")
            return ground_truth
        except Exception as e:
            print(f"Error loading ground truth data: {e}")
            return []
    def create_error_plots(ground_truth_data, ukf_estimates, times, rmse_results):
        """Create plots showing the errors over time"""
        # Interpolate ground truth to match UKF timestamps
        gt_times = np.array([gt['time'] for gt in ground_truth_data])
        gt_x = np.array([gt['position'][0] for gt in ground_truth_data])
        gt_y = np.array([gt['position'][1] for gt in ground_truth_data])
        gt_yaw = np.array([gt['yaw'] for gt in ground_truth_data])
        
        # Find overlapping time range
        valid_times = []
        gt_positions_interp = []
        ukf_positions_matched = []
        position_errors = []
        
        for i, ukf_time in enumerate(times):
            if ukf_time < gt_times[0] or ukf_time > gt_times[-1]:
                continue
            
            # Interpolate ground truth
            interp_x = np.interp(ukf_time, gt_times, gt_x)
            interp_y = np.interp(ukf_time, gt_times, gt_y)
            
            # Calculate error
            ukf_x = ukf_estimates[0, i]
            ukf_y = ukf_estimates[1, i]
            error = np.sqrt((interp_x - ukf_x)**2 + (interp_y - ukf_y)**2)
            
            valid_times.append(ukf_time)
            gt_positions_interp.append([interp_x, interp_y])
            ukf_positions_matched.append([ukf_x, ukf_y])
            position_errors.append(error)
        
        if len(valid_times) == 0:
            print("No overlapping times for error plotting")
            return
        
        # Convert to relative time (start from 0)
        valid_times = np.array(valid_times)
        relative_times = valid_times - valid_times[0]
        
        # Create error plot
        plt.figure(figsize=(12, 8))
        
        plt.subplot(2, 2, 1)
        plt.plot(relative_times, position_errors, 'r-', linewidth=2)
        plt.axhline(y=rmse_results['rmse_position'], color='b', linestyle='--', 
                    label=f'RMSE: {rmse_results["rmse_position"]:.4f}m')
        plt.xlabel('Time (s)')
        plt.ylabel('Position Error (m)')
        plt.title('Position Error Over Time')
        plt.grid(True)
        plt.legend()
        
        # X and Y error plots
        gt_positions_interp = np.array(gt_positions_interp)
        ukf_positions_matched = np.array(ukf_positions_matched)
        x_errors = gt_positions_interp[:, 0] - ukf_positions_matched[:, 0]
        y_errors = gt_positions_interp[:, 1] - ukf_positions_matched[:, 1]
        
        plt.subplot(2, 2, 2)
        plt.plot(relative_times, x_errors, 'g-', linewidth=2, label='X Error')
        plt.plot(relative_times, y_errors, 'm-', linewidth=2, label='Y Error')
        plt.axhline(y=0, color='k', linestyle='-', alpha=0.3)
        plt.xlabel('Time (s)')
        plt.ylabel('Error (m)')
        plt.title('X and Y Errors Over Time')
        plt.grid(True)
        plt.legend()
        
        # Error histogram
        plt.subplot(2, 2, 3)
        plt.hist(position_errors, bins=30, alpha=0.7, color='red', edgecolor='black')
        plt.axvline(x=rmse_results['rmse_position'], color='b', linestyle='--', 
                    label=f'RMSE: {rmse_results["rmse_position"]:.4f}m')
        plt.axvline(x=rmse_results['mae_position'], color='g', linestyle='--', 
                    label=f'MAE: {rmse_results["mae_position"]:.4f}m')
        plt.xlabel('Position Error (m)')
        plt.ylabel('Frequency')
        plt.title('Position Error Distribution')
        plt.grid(True)
        plt.legend()
        
        # Trajectory comparison
        plt.subplot(2, 2, 4)
        plt.plot(gt_positions_interp[:, 0], gt_positions_interp[:, 1], 'y-', 
                linewidth=3, label='Ground Truth')
        plt.plot(ukf_positions_matched[:, 0], ukf_positions_matched[:, 1], 'r-', 
                linewidth=2, label='UKF Estimate')
        plt.plot(gt_positions_interp[0, 0], gt_positions_interp[0, 1], 'go', 
                markersize=8, label='Start')
        plt.plot(gt_positions_interp[-1, 0], gt_positions_interp[-1, 1], 'ro', 
                markersize=8, label='End')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title('Trajectory Comparison')
        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        
        plt.tight_layout()
        plt.savefig('ukf_error_analysis.png', dpi=300, bbox_inches='tight')
        print("Error analysis plot saved as 'ukf_error_analysis.png'")
        plt.show()
    main()
