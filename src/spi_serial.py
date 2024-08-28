import sys
import os
import time
import serial
import subprocess
import threading

ROOT_FOLDER = os.path.abspath(__file__)
sys.path.append(os.path.join(ROOT_FOLDER, 'third-party/2-CH-RS485-HAT/RaspberryPi/user_dev/python/lib/'))

from waveshare_2_CH_RS485_HAT import RS485



PORT = '/tmp/rs485'
BAUD = 9600

def handle_traffic_to_port(ser, rs485):
    while True:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            rs485.RS485_CH1_Write(data)

def handle_traffic_from_port(ser, rs485):
    while True:
        if ser.out_waiting > 0:
            data = rs485.RS485_CH1_ReadByte()
            ser.write(data)

def main():
    # create virtual port 
    socat_command = ['socat', '-d', '-d', f'pty,raw,echo=0,link={PORT}']
    proc = subprocess.Popen(socat_command)
    ser = serial.Serial(PORT, BAUD)
    # instantiate RS485 object, do I/O with this object
    rs485 = RS485.RS485()
    # baud rate 9600, databits 8, stop bits 1, parity bit disable
    rs485.RS485_CH1_begin(Baud=BAUD, num_stop_config=0, word_len_config=0x11)
    time.sleep(0.01)
    
    read_thread = threading.Thread(target=handle_traffic_to_port, args=(ser, rs485))
    write_thread = threading.Thread(target=handle_traffic_from_port, args=(ser, rs485))
    read_thread.start()
    write_thread.start()
    try:
        read_thread.join()
        write_thread.join()

    except KeyboardInterrupt:
        print("Stopping interception...")

    finally:
        # Close the port
        ser.close()
        proc.terminate()
        proc.wait()
        




if __name__ == '__main__':
    main()