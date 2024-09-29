import sys
import os
import time
import serial
import subprocess
import threading
import logging

# 配置logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

# 全局常量
SYSTEM_PORT = '/dev/ttySC0'
VIRTUAL_PORT = '/tmp/rs485'
VIRTUAL_PORT_PAIR = '/tmp/rs485-pair'
# BAUD = 115200
BAUD = 9600
RETRY_DELAY = 5  # 重试连接的时间间隔（秒）

def handle_traffic_to_virtual_port(system_ser, virtual_ser):
    """从系统串口读取数据并写入到虚拟串口"""
    logging.debug("Starting handle_traffic_to_virtual_port thread.")
    while True:
        try:
            if system_ser.in_waiting > 0:
                data = system_ser.read(system_ser.in_waiting)
                logging.debug(f"Data read from system serial port: {data}")
                if data:
                    virtual_ser.write(data)
                    logging.debug("Data written to virtual serial port.")
        except serial.SerialException as e:
            logging.error(f"Serial read/write error: {e}")
            time.sleep(RETRY_DELAY)

def handle_traffic_from_virtual_port(system_ser, virtual_ser):
    """从虚拟串口读取数据并写入到系统串口"""
    logging.debug("Starting handle_traffic_from_virtual_port thread.")
    while True:
        try:
            if virtual_ser.in_waiting > 0:
                data = virtual_ser.read(virtual_ser.in_waiting)
                logging.debug(f"Data read from virtual serial port: {data}")
                if data:
                    system_ser.write(data)
                    logging.debug(f"Data written to system serial port: {data}")
        except serial.SerialException as e:
            logging.error(f"Serial read/write error: {e}")
            time.sleep(RETRY_DELAY)

def reconnect_serial(port, baud):
    """尝试重新连接串口"""
    while True:
        try:
            ser = serial.Serial(port, baud)
            logging.info(f"Connected to serial port {port} with baud rate {baud}.")
            return ser
        except serial.SerialException as e:
            logging.error(f"Failed to connect to serial port: {e}")
            time.sleep(RETRY_DELAY)

def main():
    logging.info("Starting main process.")
    
    # 创建虚拟串口设备
    socat_command = ['sudo', 'socat', '-d', '-d', f'pty,raw,echo=0,link={VIRTUAL_PORT}', f'pty,raw,echo=0,link={VIRTUAL_PORT_PAIR}']
    logging.debug(f"Running socat command: {' '.join(socat_command)}")
    proc = subprocess.Popen(socat_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    # 等待虚拟串口创建完成
    time.sleep(1)
    logging.debug(f"Waiting 1 second for {VIRTUAL_PORT} to be created.")

    # 连接系统串口和虚拟串口
    system_ser = reconnect_serial(SYSTEM_PORT, BAUD)
    virtual_ser = reconnect_serial(VIRTUAL_PORT, BAUD)

    # 创建并启动处理数据传输的线程
    to_virtual_thread = threading.Thread(target=handle_traffic_to_virtual_port, args=(system_ser, virtual_ser))
    from_virtual_thread = threading.Thread(target=handle_traffic_from_virtual_port, args=(system_ser, virtual_ser))
    to_virtual_thread.start()
    from_virtual_thread.start()
    logging.info("Data forwarding threads started.")

    try:
        while True:
            time.sleep(20)  # 主循环内每隔20秒输出一次串口状态
            logging.debug(f"System port in_waiting: {system_ser.in_waiting}, Virtual port in_waiting: {virtual_ser.in_waiting}")
    except KeyboardInterrupt:
        logging.info("Program interrupted by user.")
    finally:
        # 清理资源：关闭串口和socat进程
        system_ser.close()
        virtual_ser.close()
        proc.terminate()
        proc.wait()
        logging.info("Serial ports closed and socat process terminated.")

if __name__ == '__main__':
    main()