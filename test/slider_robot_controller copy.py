import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import json
import gzip
import paho.mqtt.client as mqtt
import os
from epoll_gas import GasSensor
from epoll_motor import MotorDriver
from slider_robot import SliderRobot
from orbbeccamera.orbbeccamera import OrbbecCamera
from datetime import datetime, timedelta

class RailRobotController(Node):
    def __init__(self):
        super().__init__('rail_robot_controller')

        # Initialize MQTT client
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect("mqtt_broker_address", 1883, 60)

        # Initialize sensors and actuators
        self.initialize_sensors()
        self.initialize_slider_robot()
        self.initialize_camera()

        # Define positions and movement control
        self.position_index = 0
        self.positions = ['A1', 'A2', 'B1', 'B2']  # Position management
        self.is_moving = False

        # Initialize sensor subscribers
        self.bridge = CvBridge()
        self.initialize_subscribers()

        # File saving path
        self.data_file = "/home/pi/sensor_data.json"

        # Sensor data storage
        self.sensor_data = {
            'gas': {},
            'temp_humidity': None,
            'depth_image_path': None,
            'infrared_image_path': None,
            'color_image_path': None,
        }

        # Load task schedule
        self.load_task_schedule()

        # Set up timers for scheduled tasks
        self.setup_task_timers()

        self.get_logger().info('Rail Robot Controller Initialized')

    def load_task_schedule(self):
        with open('task_schedule.json', 'r') as f:
            self.task_schedule = json.load(f)

    def setup_task_timers(self):
        current_time = datetime.now()
        for task in self.task_schedule['daily_tasks']:
            task_time = datetime.strptime(task['time'], "%H:%M").time()
            task_datetime = datetime.combine(current_time.date(), task_time)
            
            if task_datetime <= current_time:
                task_datetime += timedelta(days=1)
            
            delay = (task_datetime - current_time).total_seconds()
            self.create_timer(delay, lambda t=task: self.execute_scheduled_task(t))

    def execute_scheduled_task(self, task):
        self.get_logger().info(f"Executing scheduled task: {task['description']}")
        task_type = task['type']
        actions = self.task_schedule['task_types'][task_type]

        for action in actions:
            if action == 'move':
                self.execute_mission()
            elif action == 'gas':
                self.read_gas_sensors()
            elif action == 'camera':
                self.capture_camera_data()
            elif action == 'upload':
                self.compress_and_upload_data()

        # Reschedule the task for the next day
        self.create_timer(24*60*60, lambda t=task: self.execute_scheduled_task(t))

    # ... (rest of the methods remain the same)

    def execute_mission(self):
        # Execute inspection mission
        for position in self.positions:
            self.move_to_position(position)
            time.sleep(2)  # Wait for stability
            self.read_gas_sensors()
            self.capture_camera_data()
            time.sleep(3)  # Wait for sensor data collection

def main(args=None):
    rclpy.init(args=args)
    rail_robot_controller = RailRobotController()
    
    try:
        rclpy.spin(rail_robot_controller)
    finally:
        rail_robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()