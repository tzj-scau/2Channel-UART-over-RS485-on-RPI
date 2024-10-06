import sys
import os
import time
import serial
import subprocess
import threading
import logging
import crcmod

# 配置logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

# 获取项目根目录并添加第三方库路径
ROOT_FOLDER = os.path.abspath(os.path.dirname(os.path.dirname(__file__)))
sys.path.append(os.path.join(ROOT_FOLDER, 'third-party/2-CH-RS485-HAT/RaspberryPi/user_dev/python/lib/'))
logging.debug(f"Library path added: {os.path.join(ROOT_FOLDER, 'third-party/2-CH-RS485-HAT/RaspberryPi/user_dev/python/lib/')}")

from waveshare_2_CH_RS485_HAT import RS485

# 全局常量
PORT = '/tmp/rs485'
PORT1 = '/tmp/rs485-pair'
BAUD = 9600
RETRY_DELAY = 5  # 重试连接的时间间隔（秒）

def handle_traffic_to_port(ser, rs485):
    """从虚拟串口读取数据并写入到RS485设备"""
    logging.debug("Starting handle_traffic_to_port thread.")
    while True:
        try:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                logging.debug(f"Data read from virtual serial port: {data}")
                if data:
                    rs485.RS485_CH1_Write(data)
                    logging.debug("Data written to RS485 channel 1.")
        except serial.SerialException as e:
            logging.error(f"Serial read error: {e}")
            time.sleep(RETRY_DELAY)

def handle_traffic_from_port(ser, rs485):
    """从RS485读取数据并写入到虚拟串口"""
    logging.debug("Starting handle_traffic_from_port thread.")
    while True:
        try:
            data = rs485.RS485_CH1_ReadByte()
            if data:
                logging.debug(f"Data read from RS485 channel: {data}")
                ser.write(data)
                logging.debug(f"Data written to virtual serial port: {data}")
        except serial.SerialException as e:
            logging.error(f"Serial write error: {e}")
            time.sleep(RETRY_DELAY)

def reconnect_serial(port, baud):
    """尝试重新连接串口"""
    while True:
        try:
            ser = serial.Serial(port, baud)
            logging.info(f"Reconnected to serial port {port} with baud rate {baud}.")
            return ser
        except serial.SerialException as e:
            logging.error(f"Failed to reconnect to serial port: {e}")
            time.sleep(RETRY_DELAY)

def main():
    logging.info("Starting main process.")
    
    # 创建虚拟串口设备
    socat_command = ['sudo', 'socat', '-d', '-d', f'pty,raw,echo=0,link={PORT}', f'pty,raw,echo=0,link={PORT}-pair']
    logging.debug(f"Running socat command: {' '.join(socat_command)}")
    proc = subprocess.Popen(socat_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    # 等待 /tmp/rs485 端口创建完成
    time.sleep(1)  # 确保socat完成端口创建
    logging.debug("Waiting 1 second for /tmp/rs485 to be created.")

    # 尝试打开串口，若失败则重试
    ser = reconnect_serial(PORT, BAUD)

    # 初始化 RS485 设备
    rs485 = RS485.RS485()
    logging.info("RS485 object instantiated.")
    
    # 配置 RS485 通道1的参数：波特率9600, 8数据位, 1停止位, 无校验
    rs485.RS485_CH1_begin(Baud=BAUD, num_stop_config=0, word_len_config=0b11)
    logging.debug("RS485 channel 1 initialized with baud rate 9600.")
    time.sleep(0.01)
    
    # 创建并启动处理数据传输的线程
    read_thread = threading.Thread(target=handle_traffic_to_port, args=(ser, rs485))
    write_thread = threading.Thread(target=handle_traffic_from_port, args=(ser, rs485))
    read_thread.start()
    write_thread.start()
    logging.info("Read and write threads started.")

    try:
        while True:
            time.sleep(20)  # 主循环内每隔20秒输出一次串口状态
            if ser.in_waiting > 0:
                logging.debug(f"Data available on virtual serial port: {ser.in_waiting} bytes")
            else:
                logging.debug("No data available on virtual serial port.")
    except KeyboardInterrupt:
        logging.info("Program interrupted by user.")
    finally:
        # 清理资源：关闭串口和socat进程
        ser.close()
        proc.terminate()
        proc.wait()
        logging.info("Virtual serial port closed and socat process terminated.")

if __name__ == '__main__':
    main()
