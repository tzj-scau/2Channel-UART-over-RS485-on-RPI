import unittest
from unittest.mock import MagicMock, patch
import rclpy
from rail_robot_controller import RailRobotController

class TestRailRobotController(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = RailRobotController()

    def tearDown(self):
        self.node.destroy_node()

    @patch('rail_robot_controller.GasSensor')
    @patch('rail_robot_controller.MotorDriver')
    @patch('rail_robot_controller.SliderRobot')
    @patch('rail_robot_controller.OrbbecCamera')
    def test_initialization(self, mock_camera, mock_slider, mock_motor, mock_gas):
        self.assertIsNotNone(self.node.mqtt_client)
        self.assertEqual(len(self.node.gas_sensors), 4)
        self.assertIsNotNone(self.node.slider_robot)
        self.assertIsNotNone(self.node.camera)

    @patch('rail_robot_controller.SliderRobot.move_to_position')
    def test_move_to_position(self, mock_move):
        self.node.move_to_position('A1')
        mock_move.assert_called_once_with(0)
        self.assertFalse(self.node.is_moving)

    @patch('rail_robot_controller.GasSensor.read_gas_concentration')
    def test_read_gas_sensors(self, mock_read):
        mock_read.return_value = 100.0
        self.node.read_gas_sensors()
        self.assertEqual(len(self.node.sensor_data['gas']), 4)

    @patch('rail_robot_controller.OrbbecCamera.capture_data')
    def test_capture_camera_data(self, mock_capture):
        self.node.capture_camera_data()
        mock_capture.assert_called_once()
        self.assertIsNotNone(self.node.sensor_data['depth_image_path'])
        self.assertIsNotNone(self.node.sensor_data['infrared_image_path'])
        self.assertIsNotNone(self.node.sensor_data['color_image_path'])

    @patch('builtins.open', new_callable=unittest.mock.mock_open)
    @patch('json.dumps')
    def test_save_sensor_data(self, mock_json, mock_open):
        mock_json.return_value = '{"test": "data"}'
        self.node.save_sensor_data('test', 'data')
        mock_open.assert_called_with('/home/pi/sensor_data.json', 'a')
        mock_open().write.assert_called_once_with('{"test": "data"}\n')

    @patch('gzip.open')
    @patch('rail_robot_controller.mqtt.Client.publish')
    def test_compress_and_upload_data(self, mock_publish, mock_gzip):
        self.node.compress_and_upload_data()
        mock_gzip.assert_called_once()
        mock_publish.assert_called_once()

    @patch('rail_robot_controller.RailRobotController.move_to_position')
    @patch('rail_robot_controller.RailRobotController.read_gas_sensors')
    @patch('rail_robot_controller.RailRobotController.capture_camera_data')
    @patch('rail_robot_controller.RailRobotController.compress_and_upload_data')
    def test_execute_mission(self, mock_upload, mock_capture, mock_read, mock_move):
        self.node.execute_mission()
        self.assertEqual(mock_move.call_count, 4)
        self.assertEqual(mock_read.call_count, 4)
        self.assertEqual(mock_capture.call_count, 4)
        mock_upload.assert_called_once()

if __name__ == '__main__':
    unittest.main()