import serial
from spi_serial import PORT, BAUD
import pdb


ser = serial.Serial(PORT, BAUD)
ser.write("hello!".encode('utf-8'))
ser.flush()
pdb.set_trace()