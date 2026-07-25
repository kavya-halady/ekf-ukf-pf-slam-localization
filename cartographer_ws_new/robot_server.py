import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.executors import SingleThreadedExecutor
import config
import requests
import time
import threading
import subprocess
from flask import Flask, request, jsonify,Response
from threading import Thread, Lock
from threading import Event
import json
from std_msgs.msg import String
import glob
import os
import logging
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
from flask_cors import CORS
import ros_helper
import signal
import atexit
import sys
from irobot_create_msgs.msg import DockStatus

def load_data_from_file(file_path):
    try:
        with open(file_path, 'r') as file:
            data = json.load(file)  # Reads JSON data from the file
        return data
    except FileNotFoundError:
        print(f"Error: File not found - {file_path}")
        return None
app = Flask(__name__)
CORS(app)

# Logger configuration
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("RobotServer")
lock=Lock()
event=None 
retry=False
task_monitor = None
notification_event = Event()
obj_found=False
task_id = None
last_litter_task_id = None
last_shelf_task_id = None
battery_percentage = None  
message_received = False  
ros_initialized = False   
zone=1
litter_process_id=None
inventory_process_id=None

@app.route('/send_data', methods=['POST'])
def receive_data():
    global last_shelf_task_id
    global task_monitor
    global zone
    data = request.get_json()
    print("inventory task data:-----------------",data)
    zone=data['zone']
 
    # creating task of inventory
    url = f"http://{config.HOST_IP}:{config.HOST_PORT}/tasks"
    payload={
        "name":"Inventory check"
    }
    try:
        response = requests.post(url, json=payload)
        response.raise_for_status()
        print(f"POST API Response: {response.json()}")
        last_shelf_task_id = response.json().get("id")  # Correct way to retrieve `id`
        if not last_shelf_task_id:
            raise ValueError("Task ID not found in the response")
        print(f"POST API Response: {response.json()}")
    except requests.exceptions.RequestException as e:
        print(f"Error during POST API call: {e}")
        return jsonify({"message": "Inventory check failed", "error": str(e)}), 500

    try:
        dock_status = ros_helper.get_dock_status()
        if dock_status is not None:
            print(f"Dock status: {'Docked' if dock_status else 'Not docked'}")
            if dock_status:
                undock()
        else:
            print("Failed to retrieve dock status.")
    except Exception as e:
        print(f"Error retrieving dock status: {e}")

    task_monitor.run_shelf_scan()
    print("Data received from ROS 2:", data)
    return jsonify({"message": "Process completed successfully", "received_data": data}), 200
 
@app.route('/dock', methods=['POST'])
def perform_dock():
    global task_id
    data = request.get_json()
    print("is_create_task",data.get("is_create_task"))
    if data.get("is_create_task")==1:
    # creating docking task
        url = f"http://{config.HOST_IP}:{config.HOST_PORT}/tasks"
        payload={
            "name":"Dock to Station"
        }
        try:
            print("creating dock task")
            response = requests.post(url, json=payload)
            response.raise_for_status()
            task_id = response.json().get("id")  # Correct way to retrieve `id`
            if not task_id:
                raise ValueError("Task ID not found in the response")
        except requests.exceptions.RequestException as e:
            print(f"Error during POST API call: {e}")
            return jsonify({"message": "Docking failed", "error": str(e)}), 500
    
    status = send_goal_and_dock()
    if data.get("is_create_task")==1:
        url = f"http://{config.HOST_IP}:{config.HOST_PORT}/tasks/{task_id}"
        print("updating dock status")
        if status=="success":
            # updating docking task to completed
            payload={
                "status":"Completed"
            }
        else:
            payload={
                "status":"Failed"
            }
        try:
            response = requests.post(url, json=payload)
            response.raise_for_status()  
            print(f"POST API Response: {response.json()}")
        except requests.exceptions.RequestException as e:
            print(f"Error during POST API call: {e}")
            return jsonify({"message": "docking failed", "error": str(e)}), 500
    if status =="success":
        return jsonify({"message": "Dock operation completed successfully", "received_data": "data"}), 200
    else:
        return jsonify({"message": "Dock operation Failed", "received_data": "data"}), 200

@app.route('/undock', methods=['GET'])
def perform_undock():
    global task_id
 
    # creating undocking task
    url = f"http://{config.HOST_IP}:{config.HOST_PORT}/tasks"
    payload={
        "name":"Undock from station"
    }
    try:
        response = requests.post(url, json=payload)
        response.raise_for_status()
        print(f"POST API Response: {response.json()}")
        task_id = response.json().get("id")  # Correct way to retrieve `id`
        if not task_id:
            raise ValueError("Task ID not found in the response")
    except requests.exceptions.RequestException as e:
        print(f"Error during POST API call: {e}")
        return jsonify({"message": "Undocking failed", "error": str(e)}), 500
    except ValueError as e:
        print(f"Error parsing response: {e}")
        return jsonify({"message": "Undocking failed", "error": str(e)}), 500    
    status = undock()
    url = f"http://{config.HOST_IP}:{config.HOST_PORT}/tasks/{task_id}"
    payload = {
            "status":"Completed"
        }
    try:
        print("payload for undock ", payload)
        response = requests.post(url, json=payload)
        response.raise_for_status()  
        print(f"POST API Response: {response.json()}")
    except requests.exceptions.RequestException as e:
        print(f"Error during POST API call: {e}")
        return jsonify({"message": "Undocking failed", "error": str(e)}), 500
    if status =="success":
        return jsonify({"message": "Undock operation completed successfully ", "received_data": "data"}), 200
    else:
        return jsonify({"message": "Undock operation Failed ", "received_data": "data"}), 200
 
@app.route('/set_initial_pose', methods=['GET'])
def reset_initial_pose():
    """Sets the initial pose of the robot after docking."""
    try:
        time.sleep(5)  # Wait for 5 seconds after the first two commands
        subprocess.run([
            "ros2", "topic", "pub", "-1", "/initialpose", "geometry_msgs/msg/PoseWithCovarianceStamped",
            "{header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {pose: {position: {x: -7.29089, y: -3.38726, z: 0}, orientation: {x: 0.0, y: 0.0, z: 0.00727615, w: 1}}}}"
            ])
        print("Initial pose set successfully.")            
    except CalledProcessError as e:
        print(f"Error while setting initial pose: {e}")
        return jsonify({"message": "Reset failed", "error": str(e)}), 500
    return jsonify({"message": " Reset Successful", "status": "success"}), 200

@app.route('/goto_restaurant', methods=['POST'])
def goto_restaurant():
    global task_id
    # creating the goto restaurant task
    url = f"http://{config.HOST_IP}:{config.HOST_PORT}/tasks"
    payload = {
        "name": "Navigate to Restaurant"
    }
    try:
        response = requests.post(url, json=payload)
        response.raise_for_status()
        print(f"POST API Response: {response.json()}")
        task_id = response.json().get("id")  # Correct way to retrieve `id`
        if not task_id:
            raise ValueError("Task ID not found in the response")
        print(f"POST API Response: {response.json()}")
    except requests.exceptions.RequestException as e:
        print(f"Error during POST API call: {e}")
        return jsonify({"message": "Goto Restaurant failed", "error": str(e)}), 500
    try:
        dock_status =  ros_helper.get_dock_status()
        if dock_status is not None:
            print(f"Dock status: {'Docked' if dock_status else 'Not docked'}")
            if dock_status:
                undock()
        else:
            print("Failed to retrieve dock status.")
    except Exception as e:
        print(f"Error retrieving dock status: {e}")

    command = (
        "ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
        "\"pose: {header: {frame_id: 'map'}, pose: {position: {x: -1.21697., y: 0.583336, z: -0.0691882}, "
        "orientation: {x: -0.00834876, y: 0.000120744, z: 0.999471, w: -0.0314426}}}\""
    )
    result = run_command(command)
 
    # After navigation task completion, update the task status
    url = f"http://{config.HOST_IP}:{config.HOST_PORT}/tasks/{task_id}"
    payload = {"status": "Completed"}
    try:
        response = requests.post(url, json=payload)
        response.raise_for_status()
        print(f"POST API Response: {response.json()}")
    except requests.exceptions.RequestException as e:
        print(f"Error during POST API call: {e}")
        return jsonify({"message": "Goto Restaurant task completion failed", "error": str(e)}), 500
    return jsonify({"message": " Restaurant Navigation task completed", "status": "success"}), 200

@app.route('/fruits_Vegetables', methods=['POST'])
def goto_fruits_Vegetables():
    global veg_task_id
    # creating the goto restaurant task
    url = f"http://{config.HOST_IP}:{config.HOST_PORT}/tasks"
    payload = {
        "name": "Navigate to fruits_Vegetables shelf area"
    }
    try:
        response = requests.post(url, json=payload)
        response.raise_for_status()
        print(f"POST API Response: {response.json()}")
        veg_task_id = response.json().get("id")  # Correct way to retrieve `id`
        if not veg_task_id:
            raise ValueError("Task ID not found in the response")
        print(f"POST API Response: {response.json()}")
    except requests.exceptions.RequestException as e:
        print(f"Error during POST API call: {e}")
        return jsonify({"message": "Goto fruits_Vegetables area failed", "error": str(e)}), 500

    try:
        dock_status =  ros_helper.get_dock_status()
        if dock_status is not None:
            print(f"Dock status: {'Docked' if dock_status else 'Not docked'}")
            if dock_status:
                undock()
        else:
            print("Failed to retrieve dock status.")
    except Exception as e:
        print(f"Error retrieving dock status: {e}")


    initial_command = (
        "ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
        "\"pose: {header: {frame_id: 'map'}, pose: {position: {x: -7.60896, y: -2.58193, z: -2.51596e-05}, "
        "orientation: {x: 0.00171059, y: -0.0047323, z: -0.0237868, w: 0.999704}}}\""
    )
    initial_result = run_command(initial_command)

    # Check if initial navigation was successful
    if not initial_result:
        return jsonify({"message": "Failed to reach intermediate position"}), 500
    command = (
        "ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
        "\"pose: {header: {frame_id: 'map'}, pose: {position: {x: -8.39192, y: -2.41935, z: 2.13028e-07}, "
        "orientation: {x: -0.000151056, y: -0.000725245, z: 0.741996, w: 0.640403}}}\""
    )
    result = run_command(command)
 
    # After navigation task completion, update the task status
    url = f"http://{config.HOST_IP}:{config.HOST_PORT}/tasks/{veg_task_id}"
    payload = {"status": "Completed"}
    try:
        response = requests.post(url, json=payload)
        response.raise_for_status()
        print(f"POST API Response: {response.json()}")
    except requests.exceptions.RequestException as e:
        print(f"Error during POST API call: {e}")
        return jsonify({"message": "Goto fruits_Vegetables shelf area task completion failed", "error": str(e)}), 500
    speak_url = f"http://{config.HOST_IP}:{config.HOST_PORT}/speak" 
    speak_payload = {"message": "I am here at fruits_vegetables section."}
    try:
        speak_response = requests.post(speak_url, json=speak_payload)
        speak_response.raise_for_status()
        print("Speech triggered successfully.")
    except requests.exceptions.RequestException as e:
        print(f"Error during /speak API call: {e}")
        return jsonify({"message": "Navigation completed, but speech trigger failed.", "error": str(e)}), 500
    time.sleep(4)
    final_command = (
        "ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
        "\"pose: {header: {frame_id: 'map'}, pose: {position: {x: -7.60896, y: -2.58193, z: -2.51596e-05}, "
        "orientation: {x: 0.00171059, y: -0.0047323, z: -0.0237868, w: 0.999704}}}\""
    )
    final_result = run_command(final_command)

    # Check if initial navigation was successful
    if not final_result:
        return jsonify({"message": "Failed to reach intermediate position"}), 500
    try:
        # Perform docking by calling the /dock endpoint
        dock_response = requests.post(f"http://{config.ROBOT_IP}:{config.ROBOT_PORT}/dock",json={"is_create_task":0})
        dock_response.raise_for_status()
        print("Docking completed successfully.")
    except requests.exceptions.RequestException as e:
        print(f"Error during docking: {e}")
        return jsonify({"message": "Docking failed", "error": str(e)}), 500
    return jsonify({"message": " fruits_Vegetables area Navigation task completed", "status": "success"}), 200
 
@app.route('/beverages', methods=['POST'])
def goto_beverages():
    global task_id
    global colddrink_task_id
    # creating the goto restaurant task
    url = f"http://{config.HOST_IP}:{config.HOST_PORT}/tasks"
    payload = {
        "name": "Navigate to beverages area"
    }
    print("aa")
    try:
        response = requests.post(url, json=payload)
        response.raise_for_status()
        print(f"POST API Response: {response.json()}")
        print("bbbb")
        colddrink_task_id = response.json().get("id")  # Correct way to retrieve `id`
        if not colddrink_task_id:
            raise ValueError("Task ID not found in the response")
        print(f"POST API Response: {response.json()}")
    except requests.exceptions.RequestException as e:
        print(f"Error during POST API call: {e}")
        return jsonify({"message": "Goto beverages failed", "error": str(e)}), 500
    try:
        dock_status =  ros_helper.get_dock_status()
        if dock_status is not None:
            print(f"Dock status: {'Docked' if dock_status else 'Not docked'}")
            if dock_status:
                undock()
        else:
            print("Failed to retrieve dock status.")
    except Exception as e:
        print(f"Error retrieving dock status: {e}")
    command = (
        "ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
        "\"pose: {header: {frame_id: 'map'}, pose: {position: {x: -8.46868, y: -3.39011, z: 2.36772e-07}, "
        "orientation: {x: -0.00089028, y: -0.00339309, z: -0.780719, w: 0.624874}}}\""
    )
    result = run_command(command)
    # After navigation task completion, update the task status
    url = f"http://{config.HOST_IP}:{config.HOST_PORT}/tasks/{colddrink_task_id}"
    payload = {"status": "Completed"}
    try:
        response = requests.post(url, json=payload)
        response.raise_for_status()
        print(f"POST API Response: {response.json()}")
    except requests.exceptions.RequestException as e:
        print(f"Error during POST API call: {e}")
        return jsonify({"message": "Navigation to beverages task completion failed", "error": str(e)}), 500
    speak_url = f"http://{config.HOST_IP}:{config.HOST_PORT}/speak"  # Update this to the actual host URL of your API
    speak_payload = {"message": "i am here at beverages section."}
    try:
        speak_response = requests.post(speak_url, json=speak_payload)
        speak_response.raise_for_status()
        print("Speech triggered successfully.")
    except requests.exceptions.RequestException as e:
        print(f"Error during /speak API call: {e}")
        return jsonify({"message": "Navigation completed, but speech trigger failed.", "error": str(e)}), 500
    time.sleep(4)
    try:
        # Perform docking by calling the /dock endpoint
        dock_response = requests.post(f"http://{config.ROBOT_IP}:{config.ROBOT_PORT}/dock",json={"is_create_task":0})
        dock_response.raise_for_status()
        print("Docking completed successfully.")
    except requests.exceptions.RequestException as e:
        print(f"Error during docking: {e}")
        return jsonify({"message": "Docking failed", "error": str(e)}), 500
    return jsonify({"message": "Navigation to beverages task completed", "status": "success"}), 200

@app.route('/meat', methods=['POST'])
def goto_meat():
    global bakery_task_id
    # creating the goto restaurant task
    url = f"http://{config.HOST_IP}:{config.HOST_PORT}/tasks"
    payload = {
        "name": "Navigate to meat area"
    }
    try:
        response = requests.post(url, json=payload)
        response.raise_for_status()
        print(f"POST API Response: {response.json()}")
        bakery_task_id = response.json().get("id")  # Correct way to retrieve `id`
        if not bakery_task_id:
            raise ValueError("Task ID not found in the response")
        print(f"POST API Response: {response.json()}")
    except requests.exceptions.RequestException as e:
        print(f"Error during POST API call: {e}")
        return jsonify({"message": "Goto meat area failed", "error": str(e)}), 500
    try:
        dock_status =  ros_helper.get_dock_status()
        if dock_status is not None:
            print(f"Dock status: {'Docked' if dock_status else 'Not docked'}")
            if dock_status:
                undock()
        else:
            print("Failed to retrieve dock status.")
    except Exception as e:
        print(f"Error retrieving dock status: {e}")

    command = (
        "ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
        "\"pose: {header: {frame_id: 'map'}, pose: {position: {x: -8.55317, y: -3.26409, z: 3.41246e-06}, "
        "orientation: {x: -0.00024219, y: -0.00089938, z: 0.710977, w: 0.703214}}}\""
    )
    result = run_command(command)
 
    # After navigation task completion, update the task status
    url = f"http://{config.HOST_IP}:{config.HOST_PORT}/tasks/{bakery_task_id}"
    payload = {"status": "Completed"}
    try:
        response = requests.post(url, json=payload)
        response.raise_for_status()
        print(f"POST API Response: {response.json()}")
    except requests.exceptions.RequestException as e:
        print(f"Error during POST API call: {e}")
        return jsonify({"message": "Goto meat area task completion failed", "error": str(e)}), 500
    speak_url = f"http://{config.HOST_IP}:{config.HOST_PORT}/speak"  # Update this to the actual host URL of your API
    speak_payload = {"message": "I am here at meat area"}
    try:
        speak_response = requests.post(speak_url, json=speak_payload)
        speak_response.raise_for_status()
        print("Speech triggered successfully.")
    except requests.exceptions.RequestException as e:
        print(f"Error during /speak API call: {e}")
        return jsonify({"message": "Navigation completed, but speech trigger failed.", "error": str(e)}), 500
    time.sleep(4)
    try:
        # Perform docking by calling the /dock endpoint
        dock_response = requests.post(f"http://{config.ROBOT_IP}:{config.ROBOT_PORT}/dock",json={"is_create_task":0})
        dock_response.raise_for_status()
        print("Docking completed successfully.")
    except requests.exceptions.RequestException as e:
        print(f"Error during docking: {e}")
        return jsonify({"message": "Docking failed", "error": str(e)}), 500
    return jsonify({"message": " meat area Navigation task completed", "status": "success"}), 200

@app.route('/crockery_area', methods=['POST'])
def goto_crockery():
    global bakery_task_id
    # creating the goto restaurant task
    url = f"http://{config.HOST_IP}:{config.HOST_PORT}/tasks"
    payload = {
        "name": "Navigate to crockery area"
    }
    try:
        response = requests.post(url, json=payload)
        response.raise_for_status()
        print(f"POST API Response: {response.json()}")
        bakery_task_id = response.json().get("id")  # Correct way to retrieve `id`
        if not bakery_task_id:
            raise ValueError("Task ID not found in the response")
        print(f"POST API Response: {response.json()}")
    except requests.exceptions.RequestException as e:
        print(f"Error during POST API call: {e}")
        return jsonify({"message": "Goto crockery area failed", "error": str(e)}), 500
    try:
        dock_status =  ros_helper.get_dock_status()
        if dock_status is not None:
            print(f"Dock status: {'Docked' if dock_status else 'Not docked'}")
            if dock_status:
                undock()
        else:
            print("Failed to retrieve dock status.")
    except Exception as e:
        print(f"Error retrieving dock status: {e}")
    initial_command = (
        "ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
        "\"pose: {header: {frame_id: 'map'}, pose: {position: {x: -7.60896, y: -2.58193, z: -2.51596e-05}, "
        "orientation: {x: 0.00171059, y: -0.0047323, z: -0.0237868, w: 0.999704}}}\""
    )
    initial_result = run_command(initial_command)

    # Check if initial navigation was successful
    if not initial_result:
        return jsonify({"message": "Failed to reach intermediate position"}), 500
    command = (
        "ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
        "\"pose: {header: {frame_id: 'map'}, pose: {position: {x: -8.61979, y: -2.46974, z: -8.40707e-07}, "
        "orientation: {x: 0.000535862, y: 0.00131295, z: -0.714739, w: 0.69939}}}\""
    )
    result = run_command(command)
    # After navigation task completion, update the task status
    url = f"http://{config.HOST_IP}:{config.HOST_PORT}/tasks/{bakery_task_id}"
    payload = {"status": "Completed"}
    try:
        response = requests.post(url, json=payload)
        response.raise_for_status()
        print(f"POST API Response: {response.json()}")
    except requests.exceptions.RequestException as e:
        print(f"Error during POST API call: {e}")
        return jsonify({"message": "Goto crockery area task completion failed", "error": str(e)}), 500
    speak_url = f"http://{config.HOST_IP}:{config.HOST_PORT}/speak"  # Update this to the actual host URL of your API
    speak_payload = {"message": "I am here at crockery area"}
    try:
        speak_response = requests.post(speak_url, json=speak_payload)
        speak_response.raise_for_status()
        print("Speech triggered successfully.")
    except requests.exceptions.RequestException as e:
        print(f"Error during /speak API call: {e}")
        return jsonify({"message": "Navigation completed, but speech trigger failed.", "error": str(e)}), 500
    
    time.sleep(4)
    final_command = (
        "ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
        "\"pose: {header: {frame_id: 'map'}, pose: {position: {x: -7.60896, y: -2.58193, z: -2.51596e-05}, "
        "orientation: {x: 0.00171059, y: -0.0047323, z: -0.0237868, w: 0.999704}}}\""
    )
    final_result = run_command(final_command)

    # Check if initial navigation was successful
    if not final_result:
        return jsonify({"message": "Failed to reach intermediate position"}), 500
    try:
        # Perform docking by calling the /dock endpoint
        dock_response = requests.post(f"http://{config.ROBOT_IP}:{config.ROBOT_PORT}/dock",json={"is_create_task":0})
        dock_response.raise_for_status()
        print("Docking completed successfully.")
    except requests.exceptions.RequestException as e:
        print(f"Error during docking: {e}")
        return jsonify({"message": "Docking failed", "error": str(e)}), 500
    return jsonify({"message": " crockery area Navigation task completed", "status": "success"}), 200

@app.route('/litter_detection', methods=['GET'])
def receive_data1():
    global last_litter_task_id
    global task_monitor
    url = f"http://{config.HOST_IP}:{config.HOST_PORT}/tasks"
    payload={
        "name":"Litter Detection"
    }
    try:
        print("in litter api call")
        response = requests.post(url, json=payload)
        response.raise_for_status()
        print(f"POST API Response: {response.json()}")
        last_litter_task_id = response.json().get("id")  # Correct way to retrieve `id`
        if not last_litter_task_id:
            raise ValueError("Task ID not found in the response")
        print(f"POST API Response: {response.json()}")
    except requests.exceptions.RequestException as e:
        print(f"Error during POST API call: {e}")
        return jsonify({"message": " litter detection failed ", "error": str(e)}), 500
    try:
        dock_status = ros_helper.get_dock_status()
        if dock_status is not None:
            print(f"Dock status: {'Docked' if dock_status else 'Not docked'}")
            if dock_status:
                undock()
        else:
            print("Failed to retrieve dock status.")
    except Exception as e:
        print(f"Error retrieving dock status: {e}")
    task_monitor.run_litter_detection()
    return jsonify({"message": "Litter detection started ", "received_data": "data"}), 200

@app.route('/goto_supermarket', methods=['POST'])
def goto_supermarket():
    global task_id
    # creating the goto supermarket task
    url = f"http://{config.HOST_IP}:{config.HOST_PORT}/tasks"
    payload = {
        "name": "Navigate to Supermarket"
    }
    try:
        response = requests.post(url, json=payload)
        response.raise_for_status()
        print(f"POST API Response: {response.json()}")
        task_id = response.json().get("id")  # Correct way to retrieve `id`
        if not task_id:
            raise ValueError("Task ID not found in the response")
        print(f"POST API Response: {response.json()}")
    except requests.exceptions.RequestException as e:
        print(f"Error during POST API call: {e}")
        return jsonify({"message": "Goto Supermarket failed", "error": str(e)}), 500
    try:
        dock_status = ros_helper.get_dock_status()
        if dock_status is not None:
            print(f"Dock status: {'Docked' if dock_status else 'Not docked'}")
            if dock_status:
                undock()
        else:
            print("Failed to retrieve dock status.")
    except Exception as e:
        print(f"Error retrieving dock status: {e}")

    command = (
        "ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
        "\"pose: {header: {frame_id: 'map'}, pose: {position: {x: -1.29047, y: -0.47117, z: -0.0680732}, "
        "orientation: {x: -0.0102092, y: 0.000309027, z: 0.98946, w: -0.144447}}}\""
    )
    result = run_command(command)
 
    # After navigation task completion, update the task status
    url = f"http://{config.HOST_IP}:{config.HOST_PORT}/tasks/{task_id}"
    payload = {"status": "Completed"}
    try:
        response = requests.post(url, json=payload)
        response.raise_for_status()
        print(f"POST API Response: {response.json()}")
    except requests.exceptions.RequestException as e:
        print(f"Error during POST API call: {e}")
        return jsonify({"message": "Goto Supermarket task completion failed", "error": str(e)}), 500
 
    return jsonify({"message": " Supermarket Navigation task completed", "status": "success"}), 200
 
def start_flask():
    app.run(debug=True, host='0.0.0.0', port=config.ROBOT_PORT, use_reloader=False)
 
def dock():
    subprocess.run(['ros2', 'action','send_goal','/dock', 'irobot_create_msgs/action/Dock','{}'])

def send_goal_and_dock():
    global retry
    if not retry: 
        navigate_command = [
            "ros2", "action", "send_goal", "/navigate_to_pose", "nav2_msgs/action/NavigateToPose",
            "pose: {header: {frame_id: 'map'}, pose: {position: {x: -7.83108, y: -3.31151, z: -3.43532e-07 }, orientation: {x: 0.00605701, y: 0.000434269, z: 0.978879, w: 0.204352}}}"
        ]
    else:     
        navigate_command = [
            "ros2", "action", "send_goal", "/navigate_to_pose", "nav2_msgs/action/NavigateToPose",
            "pose: {header: {frame_id: 'map'}, pose: {position: {x: -7.56087, y: -3.43395, z: -0.040474 }, orientation: {x: 0.00757463, y: -6.54988e-05, z: 0.127736, w: 0.991779}}}"
        ] 
    
    try:
        # Run the navigation command and capture output
        navigate_process = subprocess.run(navigate_command, capture_output=True, text=True, check=True)
        output = navigate_process.stdout
        print("Navigation Output:", output)
        # Check if "SUCCEEDED" is in the output
        if "SUCCEEDED" in output:
            print("Navigation succeeded, proceeding to dock.")
            dock_command = [
                "ros2", "action", "send_goal", "/dock", "irobot_create_msgs/action/Dock", "{}"
            ]
            dock_process = subprocess.run(dock_command, capture_output=True, text=True, check=True)
            print("Docking Output:", dock_process.stdout)
            if "SUCCEEDED" in dock_process.stdout:
                return "success"
            else:
                if not retry:
                    print("Navigation did not succeed, docking will not proceed, retrying")
                    retry=True
                    return send_goal_and_dock()
                else:
                    print("Navigation did not succeed, retry also failed, docking will not proceed")
                    return "failed"
    except subprocess.CalledProcessError as e:
        print("Error executing command:", e)
        print("Command Output:", e.output)
        return "failed"

def undock():
    print("undocking")
    try:
        subprocess.run(['ros2', 'action','send_goal','/undock', 'irobot_create_msgs/action/Undock','{}'])
    except Exception as e:
        return "error"

def run_command(command):
    try:
        result = subprocess.run(command, shell=True, text=True, capture_output=True)
        output = result.stdout + result.stderr
        if "SUCCEEDED" in output:
            return "success"
        else:
            return "failed"
    except Exception as e:
        return "error"
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=10
)

class BatterySubscriber(Node):
    def __init__(self):
        super().__init__('battery_state_subscriber')
        logger.info("Initializing BatterySubscriber node...")
        self.subscription = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            qos_profile
        )
        logger.info("Subscription to /battery_state topic created.")

    def battery_callback(self, msg):
        global battery_percentage, message_received
        logger.info("Received message on /battery_state topic.")
        battery_percentage = msg.percentage
        logger.info(f"Battery percentage received: {battery_percentage}")
        message_received = True  

class TaskMonitor(Node):
    def __init__(self):
        super().__init__('task_monitor')
        self.last_received_time = None

        self.dock_operation_done = False  # State flag
        self.dock_last_execution_time = 0  # Timestamp of last operation
        self.cooldown_period = 120   # Cooldown period in seconds (adjust as needed)

        self.litter_operation_done = False  # State flag
        self.litter_last_execution_time = 0  # Timestamp of last operation

        self.subscription = self.create_subscription(
            String,
            '/shelf_check_completed',
            self.shelf_check_completed_callback,
            10
        )
        self.subscription = self.create_subscription(
            String,
            '/litter_check_completed',
            self.litter_check_completed_callback,
            10
        )
        self.litter_start_check=self.create_publisher(String,'/litter_start_check',10)
        self.shelf_start_check=self.create_publisher(String,'/shelf_start_check',10)
    
    def shelf_check_completed_callback(self, msg):
        global event,last_received_time, last_shelf_task_id, notification_event, lock,inventory_process_id,zone
        current_time = time.time()  
        print("INSIDE Empty shelf task completed callback")
        print("last time fir shelf  and current : ", self.dock_last_execution_time, current_time)

        if (current_time - self.dock_last_execution_time) > self.cooldown_period:
            print("Empty shelf task completed")
            file_path = '/home/ubuntu/ros2_ws/src/shelf_scan/shelf_scan/item_details_2.json'
            print("file path")
            # Load data from the file
            data = load_data_from_file(file_path)
            print("file read")
            print(type(data))
            if data is None:
                print("Error loading data from file")
            if zone==1:
                data=data.get("1")
            else:
                data=data.get("2")
            categories_and_areas = {item.get('category', None): item.get('area',None) for item in data}
            print("done with reading json")
            print("api")
            
            try:
                url = f"http://{config.HOST_IP}:{config.HOST_PORT}/inventory-check-completed"
                payload = {
                    "message": "Empty Shelf check completed successfully",
                    "results": data
                }
                response = requests.post(url, json=payload)
                response.raise_for_status()
                print(f"Inventory check API response: {response.json()}")
            except requests.exceptions.RequestException as e:
                print(f"Error calling inventory check API: {e}")
            print("api called")
            url = f"http://{config.HOST_IP}:{config.HOST_PORT}/tasks/{last_shelf_task_id}"
            print("url")
            payload={
                "status":"Completed",
                "results": categories_and_areas
            }
            print("payload", payload)
            try:
                response = requests.post(url, json=payload)
                response.raise_for_status()
                print(f"POST API Response: {response.json()}")
            except requests.exceptions.RequestException as e:
                print(f"Error during POST API call: {e}")
                return jsonify({"message": "shelf scan failed", "error": str(e)}), 500
            finally:
                if inventory_process_id and inventory_process_id.poll() is None:
                    print("terminating inventory_process_id",inventory_process_id)
                    inventory_process_id.terminate()  # First, try to terminate gracefully
                    try:
                        inventory_process_id.wait(timeout=5)  # Wait for up to 5 seconds
                    except subprocess.TimeoutExpired:
                        self.get_logger().warning("Process did not terminate in time. Killing it.")
                        inventory_process_id.kill()  # Force kill the process
                        inventory_process_id.wait()  # Wait again to clean up
                    finally:
                        inventory_process_id = None  # Clear the reference

            print("dock_last_execution_time", self.dock_last_execution_time)
            self.dock_last_execution_time = current_time
        
    def litter_check_completed_callback(self, msg):
        global event,last_received_time, last_litter_task_id, notification_event, lock
        global litter_process_id
        current_time = time.time()  
        print("INSIDE litter detection completed callback")
        print("last time fir litter  and current : ", self.litter_last_execution_time, current_time)
        if (current_time - self.litter_last_execution_time) > self.cooldown_period:
            print("litter Detection completed")
            file_path = '/home/ubuntu/ros2_ws/src/litter_scan/scripts/litter_details.json'
            litter_data = load_data_from_file(file_path)
            litter_categories_and_count = {item.get('category', None): item.get('count',None) for item in litter_data}
            try:
                url = f"http://{config.HOST_IP}:{config.HOST_PORT}/litter-detection-completed"
                payload = {
                    "message": "Litter detection completed successfully",
                    "results": litter_data
                }
                print(payload)
                response = requests.post(url, json=payload)
                response.raise_for_status()
                print(f"Litter detection API response: {response.json()}")
            except requests.exceptions.RequestException as e:
                print(f"Error calling litter detection API: {e}")
            url = f"http://{config.HOST_IP}:{config.HOST_PORT}/tasks/{last_litter_task_id}"
            payload={
                "status":"Completed",
                "results": litter_categories_and_count
            }
            print("payload", payload)
            try:
                response = requests.post(url, json=payload)
                response.raise_for_status()
                print(f"POST API Response: {response.json()}")
            except requests.exceptions.RequestException as e:
                print(f"Error during POST API call: {e}")
                send_goal_and_dock()
                return jsonify({"message": "litter detection failed", "error": str(e)}), 500
        
    def run_shelf_scan(self):
        global inventory_process_id
        if inventory_process_id is not None:
            inventory_process_id.terminate()  # First, try to terminate gracefully
            try:
                inventory_process_id.wait(timeout=5)  # Wait for up to 5 seconds
            except subprocess.TimeoutExpired:
                self.get_logger().warning("Process did not terminate in time. Killing it. before starting new process")
                inventory_process_id.kill()  # Force kill the process
                inventory_process_id.wait()  # Wait again to clean up
            finally:
                inventory_process_id = None  # Clear the reference

        start_msg=String()
        start_msg.data="Shelf scan process started"
        self.shelf_start_check.publish(start_msg)

        with open("process_logs.txt", "w") as log_file:
            # Launch the subprocess
            if zone==1:
                inventory_process_id = subprocess.Popen(
                    ['python3', '/home/ubuntu/ros2_ws/src/shelf_scan/shelf_scan/modify_notify_two_turns_nav_goal.py', '--include-initial-forward'],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
            else:
                inventory_process_id = subprocess.Popen(
                    ['python3', '/home/ubuntu/ros2_ws/src/shelf_scan/shelf_scan/modify_notify_two_turns_nav_goal.py'],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
            #Communicate with the subprocess to get its stdout and stderr
            stdout, stderr = inventory_process_id.communicate()

            if stdout:
                log_file.write(f"STDOUT:\n{stdout.decode('utf-8')}\n")
                print(f"STDOUT:\n{stdout.decode('utf-8')}")
            
            if stderr:
                log_file.write(f"STDERR:\n{stderr.decode('utf-8')}\n")
                print(f"STDERR:\n{stderr.decode('utf-8')}")
        
        self.get_logger().info(f"inventory process id: {inventory_process_id}")


    def run_litter_detection(self):
        global litter_process_id
        if litter_process_id is not None:
            litter_process_id.terminate()  # First, try to terminate gracefully
            try:
                litter_process_id.wait(timeout=5)  # Wait for up to 5 seconds
            except subprocess.TimeoutExpired:
                self.get_logger().warning("Process did not terminate in time. Killing it.")
                litter_process_id.kill()  # Force kill the process
                litter_process_id.wait()  # Wait again to clean up
            finally:
                litter_process_id = None
        litter_process_id = subprocess.Popen(
            ['python3','/home/ubuntu/ros2_ws/src/litter_scan/scripts/three_nav_goal.py','restaurant'],
            stdout=subprocess.PIPE,  # Optional: Capture stdout
            stderr=subprocess.PIPE   # Optional: Capture stderr
        )
        self.get_logger().info(f"litter scan process id: {litter_process_id}")

        start_msg=String()
        start_msg.data="litter process started"
        self.litter_start_check.publish(start_msg)
    
    def shutdown(self):
        print("Shutting down task monitor...")
        self.destroy_node()
 
def stop_subprocesses():
    """Terminate all running subprocesses."""
    for process in subprocesses:
        if process.poll() is None:  # Process is still running
            process.terminate()
    subprocesses.clear()  
 
def start_ros_commands():
    def run_localization():
        subprocess.Popen([
            "ros2", "launch", "turtlebot4_navigation", "localization.launch.py", "map:=maps/hum_180.yaml"
        ])
 
    def run_navigation():
        subprocess.Popen([
            "ros2", "launch", "turtlebot4_navigation", "nav2.launch.py"
        ])
 
    def set_initial_pose():
        """Sets the initial pose of the robot after docking."""
        try:
            time.sleep(5)  # Wait for 5 seconds after the first two commands
            subprocess.run([
                "ros2", "topic", "pub", "-1", "/initialpose", "geometry_msgs/msg/PoseWithCovarianceStamped",
                "{header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {pose: {position: {x: -7.29089, y: -3.38726, z: 0}, orientation: {x: 0.0, y: 0.0, z: 0.00727615, w: 1}}}}"
              ])

            print("Initial pose set successfully.")            
        except CalledProcessError as e:
            print(f"Error while setting initial pose: {e}")

    # # # Start the third command after a delay in a separate thread
    initial_pose_thread = Thread(target=set_initial_pose)
    initial_pose_thread.start()
 
@app.route('/get_battery_percentage', methods=['GET'])
def get_battery_state():
    """
    Flask endpoint to get the battery state.
    """
    global battery_percentage, message_received, ros_initialized

    logger.info("Received request to /get_battery_percentage.")
    battery_percentage = None  # Reset battery percentage value
    message_received = False   # Reset message received flag

    logger.info("Creating BatterySubscriber node...")
    node = BatterySubscriber()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        logger.info("Spinning ROS 2 node to process messages...")
        start_time = time.time()
        timeout = 5.0  # Timeout after 5 seconds

        # Wait for the message to be received or timeout
        while not message_received and time.time() - start_time < timeout:
            executor.spin_once(timeout_sec=0.1)

        if not message_received:
            logger.error("Timeout waiting for message on /battery_state.")
            return jsonify({'error': 'Timeout waiting for battery state'}), 504

    except Exception as e:
        logger.error(f"Error while spinning the node: {str(e)}")
        return jsonify({'error': str(e)}), 500
    finally:
        executor.shutdown()
        node.destroy_node()
        logger.info("BatterySubscriber node destroyed.")

    # Return the battery percentage to the API caller
    logger.info(f"Returning battery percentage: {battery_percentage}")
    if battery_percentage is not None:
        battery_percentage = int(battery_percentage*100)
        return jsonify({'battery_percentage': battery_percentage})
    return jsonify({'battery_percentage': 0})

@app.route('/shutdown', methods=['POST']) 
def shutdown():
    """Shutdown the server and ROS 2."""
    stop_subprocesses()
    rclpy.shutdown()
    threading.Thread(target=os._exit, args=(0,)).start()
    return jsonify({'status': 'success', 'message': 'Server shutting down'})

def terminate_process():
    global litter_process_id,inventory_process_id
    try:
        print("Terminating subprocess...")
        if litter_process_id is not None:
            litter_process_id.terminate()
            litter_process_id.wait()  # Ensure it stops
            litter_process_id = None
        if inventory_process_id is not None:
            inventory_process_id.terminate()
            inventory_process_id.wait()  # Ensure it stops
            inventory_process_id = None
        print("Subprocess terminated.")
    except Exception as e:
        print(f"Error terminating subprocess: {e}")
    sys.exit(0)

# Ensure subprocess termination on clean exit
atexit.register(terminate_process)

# # Handle termination signals like SIGTERM and SIGINT
def handle_signal(signum, frame):
    print(f"Signal {signum} received. Cleaning up...")
    terminate_process()

signal.signal(signal.SIGTERM, handle_signal)
signal.signal(signal.SIGINT, handle_signal)
signal.signal(signal.SIGSEGV, handle_signal)
 
def main(args=None):
    global task_monitor
 
    rclpy.init(args=args)
    task_monitor = TaskMonitor()
 
    try:    
        # Run the ROS 2 node
        rclpy.spin(task_monitor)
    except KeyboardInterrupt:
        print("Keyboard interrupt received. Exiting.")
    finally:
        stop_subprocesses()
        if task_monitor:
            task_monitor.shutdown()
        rclpy.shutdown()

if __name__ == "__main__":
    retry=False
     # Start ROS commands in background threads
    ros_thread = Thread(target=start_ros_commands)
    ros_thread.daemon = True
    ros_thread.start()
 
    # Start Flask server
    flask_thread = Thread(target=start_flask)
    flask_thread.daemon = True
    flask_thread.start()
    main()
