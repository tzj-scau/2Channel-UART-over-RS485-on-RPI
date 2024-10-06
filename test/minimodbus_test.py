import minimalmodbus
import serial
import logging
import time

# 开启调试模式，查看完整的 Modbus 通讯数据帧
# minimalmodbus.DEBUG = True
# minimalmodbus.logger.setLevel(logging.DEBUG)
from spi_serial_sys_dev import VIRTUAL_PORT,VIRTUAL_PORT_PAIR, BAUD

def setup_instrument(port, baud, slave_address=1):
    try:
        ser = serial.Serial(port, baud, timeout=1)
        instrument = minimalmodbus.Instrument(ser, slaveaddress=slave_address,debug=True)
        instrument.serial.timeout = 0.5  # 设置超时
        instrument.mode = minimalmodbus.MODE_RTU  # 使用RTU模式
        return instrument
    except serial.SerialException as e:
        logging.error(f"Could not open serial port: {e}")
        return None

def read_temperature(instrument):
    try:
        temperature = instrument.read_register(0x00, functioncode=3, number_of_decimals=1, signed=True)
        # temperature = instrument.read_register(0x21, functioncode=3, number_of_decimals=0, signed=False)
        return temperature
    except Exception as e:
        logging.error(f"Failed to read temperature: {e}")
        return None

def main():
    logging.basicConfig(level=logging.INFO)
    
    # instrument = setup_instrument(VIRTUAL_PORT_PAIR, BAUD)
    instrument = setup_instrument(VIRTUAL_PORT, BAUD)
    if not instrument:
        return

    logging.info('Instrument setup successful')
    logging.info(f'Instrument info: {instrument}')

    while True:
        temperature = read_temperature(instrument)
        if temperature is not None:
            print(f"Temperature: {temperature} °C")
        time.sleep(2)  # 每2秒读取一次温度

if __name__ == '__main__':
    main()