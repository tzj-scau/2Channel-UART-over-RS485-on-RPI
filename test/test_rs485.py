import unittest
import serial
import time
from threading import Thread
import os, sys

ROOT_FOLDER = os.path.abspath(__file__)
ROOT_FOLDER = os.path.dirname(ROOT_FOLDER)
ROOT_FOLDER = os.path.dirname(ROOT_FOLDER)
sys.path.append(os.path.join(ROOT_FOLDER, 'third-party/2-CH-RS485-HAT/RaspberryPi/user_dev/python/lib/'))
from waveshare_2_CH_RS485_HAT import RS485

# Constants for testing
TEST_PORT = '/tmp/rs485-1'
TEST_BAUD = 9600

class TestRS485Communication(unittest.TestCase):

    def setUp(self):
        """Setup the RS485 and virtual serial port before each test."""
        # Initialize RS485 object
        self.rs485 = RS485.RS485()
        self.rs485.RS485_CH1_begin(Baud=TEST_BAUD, num_stop_config=0, word_len_config=0x11)
        
        # Initialize virtual serial port
        self.ser = serial.Serial(TEST_PORT, TEST_BAUD)
        
        # Ensure both are properly set up
        self.assertTrue(self.ser.is_open, "Failed to open virtual serial port.")

    def tearDown(self):
        """Cleanup after each test."""
        self.ser.close()
        del self.rs485

    def test_write_to_rs485(self):
        """Test writing data from the serial port to RS485 channel."""
        test_data = b'Hello RS485'
        
        # Write test data to the serial port
        self.ser.write(test_data)
        
        # Allow some time for data to be transmitted
        time.sleep(0.1)
        
        # Read from RS485 channel 1
        rs485_data = []
        for _ in range(len(test_data)):
            rs485_data.append(self.rs485.RS485_CH1_ReadByte())
        rs485_data = bytes(rs485_data)
        
        # Verify data matches
        self.assertEqual(test_data, rs485_data, f"Data mismatch: {rs485_data} != {test_data}")

    def test_read_from_rs485(self):
        """Test reading data from RS485 channel to the serial port."""
        test_data = b'RS485 to Serial'
        
        # Write test data directly to RS485 channel 1
        for byte in test_data:
            self.rs485.RS485_CH1_Write(byte)
        
        # Allow some time for data to be transmitted
        time.sleep(0.1)
        
        # Read from the serial port
        serial_data = self.ser.read(len(test_data))
        
        # Verify data matches
        self.assertEqual(test_data, serial_data, f"Data mismatch: {serial_data} != {test_data}")

    def test_bidirectional_communication(self):
        """Test bidirectional communication between serial port and RS485."""
        send_data = b'Test RS485'
        expected_response = send_data  # Assuming loopback for the test
        
        # Function to simulate RS485 device response
        def rs485_simulator():
            while True:
                byte = self.rs485.RS485_CH1_ReadByte()
                if byte != -1:  # Assuming -1 is returned for no data (this may need adjustment based on actual API)
                    self.rs485.RS485_CH1_Write(byte)  # Echo back the data
        
        # Start RS485 simulator thread
        simulator_thread = Thread(target=rs485_simulator, daemon=True)
        simulator_thread.start()

        # Write and read from the serial port
        self.ser.write(send_data)
        time.sleep(0.1)
        received_data = self.ser.read(len(send_data))

        # Check if the received data matches expected
        self.assertEqual(received_data, expected_response, f"Data mismatch: {received_data} != {expected_response}")

if __name__ == '__main__':
    unittest.main()
