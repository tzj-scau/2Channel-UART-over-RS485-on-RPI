import minimalmodbus
import serial
import logging
import time

from spi_serial import PORT, BAUD

def test_serial_connection(ser):
    """简单的串口数据读取测试"""
    try:
        ser.timeout = 1  # 设置读取超时
        if ser.is_open:
            logging.info(f"Serial port {ser.port} is open.")
            while True:
                data = ser.read(10)  # 读取10个字节
                if data:
                    logging.info(f"Received data: {data}")
                else:
                    logging.info("No data received.")
                time.sleep(1)  # 每秒读取一次
        else:
            logging.error(f"Failed to open serial port {ser.port}.")
    except Exception as e:
        logging.error(f"Error during serial test: {e}")

def main():
    try:
        ser = serial.Serial(PORT, BAUD)
        logging.info('Testing serial connection...')
        test_serial_connection(ser)
        
        # default addr is 1 for the RSDS21 module
        instrument = minimalmodbus.Instrument(ser, slaveaddress=1) 
        logging.info('Printing serial info')
        logging.info(instrument)
        
        # refer to RSDS21 manual for register table
        try:
            temperature = instrument.read_register(0x00, functioncode=3, number_of_decimals=1)
            temperature1 = instrument.read_register(0x10, functioncode=3, number_of_decimals=1)
            temperature2 = instrument.read_register(0x40, functioncode=3, number_of_decimals=1)
            temperature3 = instrument.read_register(0x00, functioncode=4, number_of_decimals=1)
            print(f"Temperature: {temperature} °C")
        except Exception as e:
            logging.error(f"Failed to read temperature: {e}")

    except serial.SerialException as e:
        logging.error(f"Could not open serial port: {e}")

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    main()
