import os
import sys
import select
import serial
import minimalmodbus
import logging
import time
import fcntl
import threading

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

VIRTUAL_PORT_PAIR = '/tmp/vserial2'
BAUD_RATE = 9600
SLAVE_ADDRESS = 3
TEMPERATURE_REGISTER = 0x00  # 温度寄存器
HUMIDITY_REGISTER = 0x0B     # 湿度寄存器
READ_INTERVAL = 2  # 读取间隔，单位为秒

class ModbusSensorReader:
    def __init__(self, port, baudrate, slave_address):
        self.instrument = minimalmodbus.Instrument(port, slave_address)
        self.instrument.serial.baudrate = baudrate
        self.instrument.serial.timeout = 0.5
        self.instrument.mode = minimalmodbus.MODE_RTU
        
        self.fd = self.instrument.serial.fileno()
        self.epoll = select.epoll()
        self.epoll.register(self.fd, select.EPOLLIN | select.EPOLLET)
        
        # 设置串口文件描述符为非阻塞模式
        flags = fcntl.fcntl(self.fd, fcntl.F_GETFL)
        fcntl.fcntl(self.fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
        
        self.last_read_time = 0
        self.temperature = None
        self.humidity = None
        self.lock = threading.Lock()

    def read_temperature(self):
        try:
            with self.lock:
                # 修改为保留一位小数
                temperature = self.instrument.read_register(TEMPERATURE_REGISTER, number_of_decimals=1, functioncode=3, signed=True)
            logging.debug(f"Read temperature: {temperature:.1f}°C")  # 保留1位小数
            return temperature
        except Exception as e:
            logging.error(f"Failed to read temperature: {e}")
            return None

    def read_humidity(self):
        try:
            with self.lock:
                # 修改为保留一位小数
                humidity = self.instrument.read_register(HUMIDITY_REGISTER, number_of_decimals=1, functioncode=3, signed=True)
            logging.debug(f"Read humidity: {humidity:.1f}%")  # 保留1位小数
            return humidity
        except Exception as e:
            logging.error(f"Failed to read humidity: {e}")
        return None

    def update_sensors(self):
        current_time = time.time()
        if current_time - self.last_read_time >= READ_INTERVAL:
            temp = self.read_temperature()
            humidity = self.read_humidity()
            if temp is not None and humidity is not None:
                with self.lock:
                    self.temperature = temp
                    self.humidity = humidity
                self.last_read_time = current_time

    def get_temperature(self):
        with self.lock:
            return self.temperature

    def get_humidity(self):
        with self.lock:
            return self.humidity

    def close(self):
        self.epoll.unregister(self.fd)
        self.epoll.close()
        self.instrument.serial.close()

def sensor_reader_thread(reader):
    try:
        while True:
            events = reader.epoll.poll(1)
            for fileno, event in events:
                if event & select.EPOLLIN:
                    # 清除任何待处理的数据
                    reader.instrument.serial.read(reader.instrument.serial.in_waiting or 1)
            
            reader.update_sensors()
            
            # 短暂睡眠以降低CPU使用率
            time.sleep(0.1)
    except Exception as e:
        logging.error(f"Error in sensor reader thread: {e}")

def main():
    reader = ModbusSensorReader(VIRTUAL_PORT_PAIR, BAUD_RATE, SLAVE_ADDRESS)
    
    # 启动传感器读取线程
    thread = threading.Thread(target=sensor_reader_thread, args=(reader,))
    thread.daemon = True
    thread.start()

    # 主循环中显示保留1位小数的温度和湿度
    try:
        while True:
            temperature = reader.get_temperature()
            humidity = reader.get_humidity()
            if temperature is not None and humidity is not None:
                print(f"Current temperature: {temperature:.1f}°C, Humidity: {humidity:.1f}%")
            time.sleep(1)  # 每秒更新一次显示
    except KeyboardInterrupt:
        logging.info("Program interrupted by user.")
    finally:
        reader.close()
        logging.info("Sensor reader closed.")

if __name__ == "__main__":
    main()
