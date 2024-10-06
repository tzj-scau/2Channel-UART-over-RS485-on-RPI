import minimalmodbus
import serial
import logging
import time

from spi_serial import PORT, BAUD



def main():
    try:
        ser = serial.Serial(PORT, BAUD)
        logging.info('Testing serial connection...')
        
        # default addr is 1 for the RSDS21 module
        instrument = minimalmodbus.Instrument(ser, slaveaddress=1) 
        logging.info('Printing serial info')
        logging.info(instrument)
        
        # refer to RSDS21 manual for register table
        try:
            # temperature1 = instrument.read_register(0x10, functioncode=3, number_of_decimals=1)
            temperature1 = instrument.read_register(0x00, functioncode=3, number_of_decimals=1)
            # temperature1 = instrument.read_register(0x40, functioncode=3, number_of_decimals=1)
            print(f"Temperature: {temperature1} °C")
        except Exception as e:
            logging.error(f"Failed to read temperature: {e}")

    except serial.SerialException as e:
        logging.error(f"Could not open serial port: {e}")

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    main()
