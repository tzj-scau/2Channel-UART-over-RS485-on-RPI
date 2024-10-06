import sys
import os
import time
import serial
import subprocess
import threading
import logging


# 配置logging
logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s - %(levelname)s - %(message)s')

ROOT_FOLDER = os.path.abspath(__file__)
# 获取上一级父文件夹
ROOT_FOLDER = os.path.dirname(ROOT_FOLDER)

# 获取上两级父文件夹
ROOT_FOLDER = os.path.dirname(ROOT_FOLDER)
sys.path.append(os.path.join(ROOT_FOLDER, 'third-party/2-CH-RS485-HAT/RaspberryPi/user_dev/python/lib/'))
logging.debug(f"Library path added: {os.path.join(ROOT_FOLDER, 'third-party/2-CH-RS485-HAT/RaspberryPi/user_dev/python/lib/')}")

from waveshare_2_CH_RS485_HAT import RS485

PORT = '/tmp/rs485'
PORT2 = '/tmp/rs485-1'
BAUD = 9600

def handle_traffic_to_port(ser, rs485):
    logging.debug("Starting handle_traffic_to_port thread.")
    ser.out_waiting
    while True:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            logging.debug(f"Data read from serial port: {data}")
            if data:
                rs485.RS485_CH1_Write(data)
                logging.debug(f"Data written to RS485 channel 1.")
            else:
                logging.debug("No data was read from serial port.")

def handle_traffic_from_port(ser, rs485):
    logging.debug("Starting handle_traffic_from_port thread.")
    while True:
        data = rs485.RS485_CH1_ReadByte()
        logging.debug(f"Data read from RS485 channel: {data}")
        if data:
            ser.write(data)
            logging.debug(f"Data written to serial port: {data}")
        else:
            logging.debug("No data was read from RS485 channel.")

def main():
    logging.info("Starting main process.")
    
    # create virtual port 
    socat_command = ['sudo', 'socat', '-d', '-d', f'pty,raw,echo=0,link={PORT}', f'pty,raw,echo=0,link={PORT2}']
    logging.debug(f"Running socat command: {' '.join(socat_command)}")
    proc = subprocess.Popen(socat_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    # 等待 /tmp/rs485 端口创建完成
    time.sleep(1)  # 等待1秒，确保socat完成端口创建
    logging.debug("Waiting 1 second for /tmp/rs485 to be created.")

    try:
        ser = serial.Serial(PORT2, BAUD)
        logging.info(f"Opened serial port {PORT} with baud rate {BAUD}.")
    except serial.SerialException as e:
        logging.error(f"Failed to open serial port {PORT}: {e}")
        proc.terminate()
        proc.wait()
        sys.exit(1)
    
    # instantiate RS485 object, do I/O with this object
    rs485 = RS485.RS485()
    logging.info("RS485 object instantiated.")
    
    # baud rate 9600, databits 8, stop bits 1, parity bit disable
    rs485.RS485_CH1_begin(Baud=BAUD, num_stop_config=0, word_len_config=0b11)
    logging.debug("RS485 channel 1 initialized with baud rate 9600.")
    time.sleep(0.01)
    
    read_thread = threading.Thread(target=handle_traffic_to_port, args=(ser, rs485))
    write_thread = threading.Thread(target=handle_traffic_from_port, args=(ser, rs485))
    read_thread.start()
    write_thread.start()
    logging.info("Read and write threads started.")

    try:
        while True:
            time.sleep(20)  # 主循环内每隔5秒输出一次线程状态
            # logging.debug(f"Read thread alive: {read_thread.is_alive()}")
            # logging.debug(f"Write thread alive: {write_thread.is_alive()}")
            # logging.debug("Checking data on serial port...")
            if ser.in_waiting > 0:
                logging.debug(f"Data available on serial port: {ser.in_waiting} bytes")
            else:
                logging.debug("No data available on serial port.")
    except KeyboardInterrupt:
        logging.info("Stopping interception due to keyboard interrupt.")
    finally:
        # Close the port and terminate socat
        ser.close()
        proc.terminate()
        proc.wait()
        logging.info("Serial port closed and socat process terminated.")

if __name__ == '__main__':
    main()
