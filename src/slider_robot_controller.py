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

        # Load configuration
        self.load_configuration()

        # Initialize MQTT client
        self.initialize_mqtt()

        # Initialize sensors and actuators
        self.initialize_sensors()
        self.initialize_slider_robot()
        self.initialize_camera()

        # Define positions and movement control
        self.position_index = 0
        self.positions = self.config['positions']
        self.is_moving = False

        # Initialize sensor subscribers
        self.bridge = CvBridge()
        self.initialize_subscribers()

        # Sensor data storage
        self.sensor_data = {
            'gas': {},
            'temp_humidity': None,
            'depth_image_path': None,
            'infrared_image_path': None,
            'color_image_path': None,
        }

        # Set up timers for scheduled tasks
        self.setup_task_timers()

        self.get_logger().info('Rail Robot Controller Initialized')

    def load_configuration(self):
        with open('comprehensive_config.json', 'r') as f:
            self.config = json.load(f)

    def initialize_mqtt(self):
        mqtt_config = self.config['mqtt']
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect(mqtt_config['broker_address'], mqtt_config['port'], mqtt_config['keepalive'])

    def initialize_sensors(self):
        self.gas_sensors = {}
        for gas, sensor_config in self.config['gas_sensors'].items():
            self.gas_sensors[gas] = GasSensor(
                gas_name=gas,
                port=sensor_config['port'],
                baud=sensor_config['baud_rate'],
                slave_address=sensor_config['slave_address'],
                decimal_point=sensor_config['decimal_point']
            )

    def initialize_slider_robot(self):
        slider_config = self.config['slider_robot']
        motor_driver = MotorDriver(
            port=slider_config['port'],
            slave_address=slider_config['slave_address'],
            baud_rate=slider_config['baud_rate']
        )
        self.slider_robot = SliderRobot(motor_driver, slider_config['positions'])

    def initialize_camera(self):
        self.camera = OrbbecCamera(self.config['camera']['device_index'])

    def initialize_subscribers(self):
        self.subscription_temp_humidity = self.create_subscription(
            String,
            '/temp_humidity_sensor/data',
            self.temp_humidity_callback,
            10)

    def setup_task_timers(self):
        current_time = datetime.now()
        for task in self.config['daily_tasks']:
            task_time = datetime.strptime(task['time'], "%H:%M").time()
            task_datetime = datetime.combine(current_time.date(), task_time)
            
            if task_datetime <= current_time:
                task_datetime += timedelta(days=1)
            
            delay = (task_datetime - current_time).total_seconds()
            self.create_timer(delay, lambda t=task: self.execute_scheduled_task(t))

    def execute_scheduled_task(self, task):
        self.get_logger().info(f"Executing scheduled task: {task['description']}")
        task_type = task['type']
        actions = self.config['task_types'][task_type]

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

    def move_to_position(self, position):
        self.get_logger().info(f'Moving to position {position}')
        self.is_moving = True
        self.slider_robot.move_to_position(self.positions.index(position))
        self.is_moving = False
        self.get_logger().info(f'Arrived at {position}')

    def temp_humidity_callback(self, msg):
        self.get_logger().info(f'Received temp/humidity data: {msg.data}')
        self.save_sensor_data('temp_humidity', msg.data)

    def read_gas_sensors(self):
        for gas, sensor in self.gas_sensors.items():
            concentration = sensor.read_gas_concentration()
            if concentration is not None:
                self.save_sensor_data('gas', {gas: concentration})

    def capture_camera_data(self):
        self.camera.capture_data()
        timestamp = int(time.time())
        self.save_sensor_data('depth_image_path', self.config['image_paths']['depth'].format(timestamp=timestamp))
        self.save_sensor_data('infrared_image_path', self.config['image_paths']['infrared'].format(timestamp=timestamp))
        self.save_sensor_data('color_image_path', self.config['image_paths']['color'].format(timestamp=timestamp))

    def save_sensor_data(self, sensor_type, data):
        if sensor_type == 'gas':
            self.sensor_data['gas'].update(data)
        else:
            self.sensor_data[sensor_type] = data

        record = {
            'position': self.positions[self.position_index],
            'sensor_type': sensor_type,
            'data': data,
            'timestamp': int(time.time())
        }
        with open(self.config['data_file'], 'a') as f:
            f.write(json.dumps(record) + '\n')
        self.get_logger().info(f'Sensor data saved: {record}')

    def compress_and_upload_data(self):
        with open(self.config['data_file'], 'rb') as f_in:
            with gzip.open(self.config['data_file'] + '.gz', 'wb') as f_out:
                f_out.writelines(f_in)

        with open(self.config['data_file'] + '.gz', 'rb') as f:
            self.mqtt_client.publish("robot/sensor_data", f.read())
        self.get_logger().info('Data uploaded to MQTT broker')

    def execute_mission(self):
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