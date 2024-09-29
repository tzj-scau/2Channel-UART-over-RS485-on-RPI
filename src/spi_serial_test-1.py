import serial
import time

PORT = '/tmp/rs485-pair'  # 使用回环测试端口
BAUD = 9600

def test_virtual_serial():
    """测试虚拟串口数据传输功能"""
    try:
        ser = serial.Serial(PORT, BAUD)
        print(f"Opened serial port {PORT} with baud rate {BAUD}.")

        # 测试数据写入
        test_data = b"Hello RS485!"
        ser.write(test_data)
        print(f"Sent data: {test_data}")

        # 等待数据响应
        time.sleep(1)

        if ser.in_waiting > 0:
            received_data = ser.read(ser.in_waiting)
            print(f"Received data: {received_data}")
        else:
            print("No data received.")

        ser.close()
        print("Serial port closed.")

    except serial.SerialException as e:
        print(f"Failed to open serial port: {e}")

if __name__ == '__main__':
    test_virtual_serial()
