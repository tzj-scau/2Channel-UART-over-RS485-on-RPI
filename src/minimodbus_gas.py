import minimalmodbus
import serial
import logging
import time
from spi_serial_sys_dev import VIRTUAL_PORT_PAIR, BAUD

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Sensor addresses
SENSOR_ADDRESSES = {
    'CO2': 1,
    'H2S': 2,
    'NH3': 3,
    'CO': 4
}

def setup_instrument(port, baud):
    try:
        instrument = minimalmodbus.Instrument(port, slaveaddress=0, debug=True)
        instrument.serial.baudrate = baud
        instrument.serial.timeout = 0.5
        instrument.mode = minimalmodbus.MODE_RTU
        return instrument
    except serial.SerialException as e:
        logging.error(f"Could not open serial port: {e}")
        return None

def read_sensor_value(instrument, sensor_address):
    try:
        instrument.address = sensor_address
        value = instrument.read_register(0x00, functioncode=3, number_of_decimals=1, signed=True)
        return value
    except Exception as e:
        logging.error(f"Failed to read sensor at address {sensor_address}: {e}")
        return None

def main():
    instrument = setup_instrument(VIRTUAL_PORT_PAIR, BAUD)
    if not instrument:
        return

    logging.info('Instrument setup successful')
    logging.info(f'Instrument info: {instrument}')

    while True:
        for sensor, address in SENSOR_ADDRESSES.items():
            value = read_sensor_value(instrument, address)
            if value is not None:
                print(f"{sensor}: {value}")
        
        print("-" * 20)  # Separator between readings
        time.sleep(2)  # Poll every 2 seconds

if __name__ == '__main__':
    main()