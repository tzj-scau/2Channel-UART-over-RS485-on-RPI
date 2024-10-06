import minimalmodbus
import serial
import struct
import math
import time

# Constants
SLAVE_ADDRESS = 1
BAUD_RATE = 9600
VIRTUAL_PORT = '/tmp/vserial2'

# Function codes
FUNCTION_READ_INPUT_REGISTERS = 0x04
FUNCTION_WRITE_SINGLE_REGISTER = 0x06
FUNCTION_WRITE_MULTIPLE_REGISTERS = 0x10

# Register addresses (based on the Modbus-RTU instruction document)
class Registers:
    # Read registers
    VERSION = 0x0010
    RESISTANCE_INDUCTANCE = 0x0012
    PID_PARAMS = 0x0014
    HOMING_PARAMS = 0x001C
    BUS_VOLTAGE = 0x0024
    BUS_CURRENT = 0x0026
    PHASE_CURRENT = 0x0027
    ENCODER_RAW = 0x0029
    ENCODER_CALIBRATED = 0x0034
    TARGET_POSITION = 0x003C
    VELOCITY = 0x0044
    POSITION = 0x0046
    POSITION_ERROR = 0x004A
    TEMPERATURE = 0x004E
    MOTOR_STATUS = 0x0050
    HOMING_STATUS = 0x0052
    SYSTEM_STATUS = 0x0078
    DRIVER_PARAMS = 0x0062
    STOP = 0x00FE

    # Write registers
    CALIBRATE_ENCODER = 0x0006
    CLEAR_POSITION = 0x000A
    CLEAR_CLOG = 0x000E
    FACTORY_RESET = 0x000F
    SET_SUBDIVISION = 0x00A0
    SET_ID = 0x00A2
    SET_PID = 0x00B0
    SET_HOMING_PARAMS = 0x00C0
    ENABLE_CONTROL = 0x00E0
    TORQUE_CONTROL = 0x00E2
    VELOCITY_CONTROL = 0x00E6
    POSITION_CONTROL_DIRECT = 0x00F0
    POSITION_CONTROL_PLANNED = 0x00F6
    STOP = 0x00FE
    SYNC_MOVE = 0x00FF
    SET_HOMING_POINT = 0x00D8
    TRIGGER_HOMING = 0x00DA
    INTERRUPT_HOMING = 0x00DC
    MODIFY_DRIVER_PARAMS = 0x00A8

class Motor:
    def __init__(self, port, slave_address, baud_rate=9600):
        self.instrument = minimalmodbus.Instrument(port, slave_address)
        self.instrument.serial.baudrate = baud_rate
        self.instrument.serial.timeout = 1
        self.instrument.mode = minimalmodbus.MODE_RTU
        self.instrument.debug = True

    def enable_motor(self, enable=True, sync_flag=False):
        values = [0xAB << 8 | 0x01 if enable else 0x00, (0x01 if sync_flag else 0x00) << 8 | 0x00]
        try:
            self.instrument.write_registers(Registers.ENABLE_CONTROL, values, functioncode=FUNCTION_WRITE_MULTIPLE_REGISTERS)
            print(f"Motor {'enabled' if enable else 'disabled'}.")
        except minimalmodbus.NoResponseError:
            print("No response from instrument during enable_motor, continuing...")
        except minimalmodbus.ModbusException as e:
            print(f"Modbus error during enable_motor: {e}, continuing...")

    def stop(self, sync_flag=False):
        value = (0x98 << 8) | (0x01 if sync_flag else 0x00)
        try:
            self.instrument.write_register(Registers.STOP, value, functioncode=FUNCTION_WRITE_SINGLE_REGISTER)
            print("Motor stopped.")
        except minimalmodbus.NoResponseError:
            print("No response from instrument during stop, continuing...")
        except minimalmodbus.ModbusException as e:
            print(f"Modbus error during stop: {e}, continuing...")

    def trigger_homing(self, mode=0, sync_flag=False):
        value = (mode << 8) | (0x01 if sync_flag else 0x00)
        try:
            self.instrument.write_register(Registers.TRIGGER_HOMING, value, functioncode=FUNCTION_WRITE_SINGLE_REGISTER)
            print("Homing triggered.")
        except minimalmodbus.NoResponseError:
            print("No response from instrument during trigger_homing, continuing...")
        except minimalmodbus.ModbusException as e:
            print(f"Modbus error during trigger_homing: {e}, continuing...")

    def set_velocity(self, direction, velocity, acceleration=300, sync_flag=False):
        # direction: 0 for CW, 1 for CCW
        abs_velocity = abs(int(velocity))
        values = [direction, acceleration, abs_velocity, 0x01 if sync_flag else 0x00]
        try:
            self.instrument.write_registers(Registers.VELOCITY_CONTROL, values, functioncode=FUNCTION_WRITE_MULTIPLE_REGISTERS)
            print(f"Velocity set to {velocity} with acceleration {acceleration}.")
        except minimalmodbus.NoResponseError:
            print("No response from instrument during set_velocity, continuing...")
        except minimalmodbus.ModbusException as e:
            print(f"Modbus error during set_velocity: {e}, continuing...")

    def set_position(self, direction, counts, velocity, relative=True, sync_flag=False):
        # counts: target position in counts
        # velocity: target velocity in motor units
        # direction: 0 for CW, 1 for CCW
        low_word = counts & 0xFFFF
        high_word = (counts >> 16) & 0xFFFF
        values = [
            direction,
            velocity * 10,
            low_word,
            high_word,
            ((0x00 if relative else 0x01) << 8) | (0x01 if sync_flag else 0x00)
        ]
        try:
            self.instrument.write_registers(Registers.POSITION_CONTROL_DIRECT, values)
            print(f"Position set to {counts} counts with velocity {velocity}.")
        except minimalmodbus.NoResponseError:
            print("No response from instrument during set_position, continuing...")
        except minimalmodbus.ModbusException as e:
            print(f"Modbus error during set_position: {e}, continuing...")

    def set_position_planned(self, direction, counts, max_velocity, accel, decel, relative=True, sync_flag=False):
        position_low = counts & 0xFFFF
        position_high = (counts >> 16) & 0xFFFF
        max_velocity_scaled = int(max_velocity * 10)
        accel_scaled = int(accel * 10)
        decel_scaled = int(decel * 10)
        movement_flags = 0x0000  # 0 for relative, 1 for absolute
        movement_flags |= (0x01 if not relative else 0x00)
        sync_flag_value = 0x01 if sync_flag else 0x00
        values = [
            direction,
            accel_scaled,
            decel_scaled,
            max_velocity_scaled,
            position_low,
            position_high,
            (movement_flags << 8) | sync_flag_value
        ]
        try:
            self.instrument.write_registers(Registers.POSITION_CONTROL_PLANNED, values)
            print(f"Planned position set to {counts} counts with max velocity {max_velocity}.")
        except minimalmodbus.NoResponseError:
            print("No response from instrument during set_position_planned, continuing...")
        except minimalmodbus.ModbusException as e:
            print(f"Modbus error during set_position_planned: {e}, continuing...")

    def read_position(self):
        try:
            result = self.instrument.read_registers(Registers.POSITION, 3, functioncode=FUNCTION_READ_INPUT_REGISTERS)
            direction = result[0]
            position_counts = (result[2] << 16) | result[1]
            position_counts = -position_counts if direction else position_counts
            print(f"Current motor position: {position_counts} counts.")
            return position_counts
        except minimalmodbus.NoResponseError:
            print("No response from instrument during read_position, returning None.")
            return None
        except minimalmodbus.ModbusException as e:
            print(f"Modbus error during read_position: {e}, returning None.")
            return None

    def read_velocity(self):
        try:
            result = self.instrument.read_registers(Registers.VELOCITY, 2, functioncode=FUNCTION_READ_INPUT_REGISTERS)
            direction = result[0]
            velocity = result[1]
            velocity = -velocity if direction else velocity
            print(f"Current motor velocity: {velocity} RPM.")
            return velocity
        except minimalmodbus.NoResponseError:
            print("No response from instrument during read_velocity, returning None.")
            return None
        except minimalmodbus.ModbusException as e:
            print(f"Modbus error during read_velocity: {e}, returning None.")
            return None

    def read_motor_status(self):
        try:
            status = self.instrument.read_register(Registers.MOTOR_STATUS, functioncode=FUNCTION_READ_INPUT_REGISTERS)
            status_dict = {
                'enabled': bool(status & 0x01),
                'in_position': bool(status & 0x02),
                'motor_stalled': bool(status & 0x04),
                'stall_protection': bool(status & 0x08)
            }
            print(f"Motor status: {status_dict}")
            return status_dict
        except minimalmodbus.NoResponseError:
            print("No response from instrument during read_motor_status, returning None.")
            return None
        except minimalmodbus.ModbusException as e:
            print(f"Modbus error during read_motor_status: {e}, returning None.")
            return None

    # Add other motor methods as needed...
