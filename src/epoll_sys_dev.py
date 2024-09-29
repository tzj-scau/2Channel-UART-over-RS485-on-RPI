import os
import sys
import select
import serial
import logging
import time
import fcntl
import subprocess
import atexit

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

SYSTEM_PORT = '/dev/ttySC0'
VIRTUAL_PORT = '/tmp/vserial1'
VIRTUAL_PORT_PAIR = '/tmp/vserial2'
BAUD_RATE = 9600

class EpollSerialHandler:
    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate, timeout=0)
        self.fd = self.ser.fileno()
        self.epoll = select.epoll()
        self.epoll.register(self.fd, select.EPOLLIN | select.EPOLLET)
        
        # Set the serial port file descriptor to non-blocking mode
        flags = fcntl.fcntl(self.fd, fcntl.F_GETFL)
        fcntl.fcntl(self.fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)

    def read_data(self):
        try:
            data = self.ser.read(self.ser.in_waiting or 1)
            if data:
                logging.debug(f"Read data from {self.ser.port}: {data}")
                return data
        except serial.SerialException as e:
            logging.error(f"Serial read error on {self.ser.port}: {e}")
        return None

    def write_data(self, data):
        try:
            self.ser.write(data)
            logging.debug(f"Wrote data to {self.ser.port}: {data}")
        except serial.SerialException as e:
            logging.error(f"Serial write error on {self.ser.port}: {e}")

    def close(self):
        self.epoll.unregister(self.fd)
        self.epoll.close()
        self.ser.close()

def create_virtual_serial_ports():
    cmd = f"socat -d -d pty,raw,echo=0,link={VIRTUAL_PORT} pty,raw,echo=0,link={VIRTUAL_PORT_PAIR}"
    process = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    # Wait for the virtual ports to be created
    time.sleep(1)
    
    def cleanup_socat():
        process.terminate()
        process.wait()
        logging.info("Socat process terminated.")

    atexit.register(cleanup_socat)
    
    logging.info(f"Virtual serial ports created: {VIRTUAL_PORT} and {VIRTUAL_PORT_PAIR}")

def main():
    create_virtual_serial_ports()

    system_handler = EpollSerialHandler(SYSTEM_PORT, BAUD_RATE)
    virtual_handler = EpollSerialHandler(VIRTUAL_PORT, BAUD_RATE)

    handlers = {
        system_handler.fd: (system_handler, virtual_handler),
        virtual_handler.fd: (virtual_handler, system_handler)
    }

    epoll = select.epoll()
    epoll.register(system_handler.fd, select.EPOLLIN | select.EPOLLET)
    epoll.register(virtual_handler.fd, select.EPOLLIN | select.EPOLLET)

    try:
        while True:
            events = epoll.poll(1)
            for fileno, event in events:
                if event & select.EPOLLIN:
                    source_handler, dest_handler = handlers[fileno]
                    data = source_handler.read_data()
                    if data:
                        dest_handler.write_data(data)

    except KeyboardInterrupt:
        logging.info("Program interrupted by user.")
    finally:
        system_handler.close()
        virtual_handler.close()
        epoll.close()
        logging.info("Serial ports closed.")

if __name__ == "__main__":
    main()