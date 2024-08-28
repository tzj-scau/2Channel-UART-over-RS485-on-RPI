import minimalmodbus
import serial
import logging

from spi_serial import PORT, BAUD

def main():
    ser = serial.Serial(PORT, BAUD)
    # default addr is 1 for the RSDS21 module
    instrument = minimalmodbus.Instrument(ser, slaveaddress=1) 
    logging.info('Printing serial info')
    logging.info(instrument)
    # refer to RSDS21 manual for register table
    temperature = instrument.read_register(0x10, functioncode=3, number_of_decimals=1)
    print(temperature)


if __name__ == '__main__':
    main()
