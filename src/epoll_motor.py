# motor_driver.py

import minimalmodbus
import struct

# Constants
SLAVE_ADDRESS = 1
BAUD_RATE = 9600
VIRTUAL_PORT = '/tmp/vserial2'

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
    SET_HOMING_POINT = 0x00D8
    TRIGGER_HOMING = 0x00DA
    INTERRUPT_HOMING = 0x00DC
    MODIFY_DRIVER_PARAMS = 0x00A8


class MotorDriver:
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
        except minimalmodbus.NoResponseError:
            print("No response from instrument during enable_motor, continuing...")
        except minimalmodbus.ModbusException as e:
            print(f"Modbus error during enable_motor: {e}, continuing...")

    def stop(self, sync_flag=False):
        """Immediately stop the motor"""
        value = (0x98 << 8) | (0x01 if sync_flag else 0x00)
        try:
            self.instrument.write_register(Registers.STOP, value, functioncode=FUNCTION_WRITE_SINGLE_REGISTER)
        except minimalmodbus.NoResponseError:
            print("No response from instrument during stop, continuing...")
        except minimalmodbus.ModbusException as e:
            print(f"Modbus error during stop: {e}, continuing...")

    def trigger_homing(self, mode=0, sync_flag=False):
        """Trigger homing operation"""
        value = (mode << 8) | (0x01 if sync_flag else 0x00)
        try:
            self.instrument.write_register(Registers.TRIGGER_HOMING, value, functioncode=FUNCTION_WRITE_SINGLE_REGISTER)
        except minimalmodbus.NoResponseError:
            print("No response from instrument during trigger_homing, continuing...")
        except minimalmodbus.ModbusException as e:
            print(f"Modbus error during trigger_homing: {e}, continuing...")

    def set_velocity(self, velocity, acceleration=300, sync_flag=False):
        """Set motor velocity"""
        direction = 0 if velocity >= 0 else 1
        abs_velocity = abs(int(velocity))
        values = [direction, acceleration, abs_velocity, 0x01 if sync_flag else 0x00]
        try:
            self.instrument.write_registers(Registers.VELOCITY_CONTROL, values)
        except minimalmodbus.NoResponseError:
            print("No response from instrument during set_velocity, continuing...")
        except minimalmodbus.ModbusException as e:
            print(f"Modbus error during set_velocity: {e}, continuing...")

    def set_position(self, counts, direction, velocity=300, relative=True, sync_flag=False):
        """Set target position in counts"""
        # Split counts into low and high words
        low_word = counts & 0xFFFF
        high_word = (counts >> 16) & 0xFFFF

        values = [
            direction,
            velocity * 10,
            low_word,
            high_word,
            ((0x00 if relative else 0x01) << 8) | (0x01 if sync_flag else 0x00),
        ]
        try:
            self.instrument.write_registers(Registers.POSITION_CONTROL_DIRECT, values)
        except minimalmodbus.NoResponseError:
            print("No response from instrument during set_position, continuing...")
        except minimalmodbus.ModbusException as e:
            print(f"Modbus error during set_position: {e}, continuing...")

    def set_position_planned(
        self, counts, direction, max_velocity_rpm, accel_rpm_per_s, decel_rpm_per_s, relative=True, sync_flag=False
    ):
        """Set target position with trapezoidal speed profile"""
        # Scale parameters as per device requirements
        max_velocity_scaled = int(max_velocity_rpm * 10)
        accel_scaled = int(accel_rpm_per_s * 10)
        decel_scaled = int(decel_rpm_per_s * 10)

        # Split counts into low and high words
        position_low = counts & 0xFFFF
        position_high = (counts >> 16) & 0xFFFF

        movement_flags = 0x0000  # Initialize flags
        movement_flags |= 0x01 if not relative else 0x00  # Absolute or relative
        sync_flag_value = 0x01 if sync_flag else 0x00  # Sync flag

        values = [
            direction,
            accel_scaled,
            decel_scaled,
            max_velocity_scaled,
            position_low,
            position_high,
            (movement_flags << 8) | sync_flag_value,
        ]

        try:
            self.instrument.write_registers(Registers.POSITION_CONTROL_PLANNED, values)
        except minimalmodbus.NoResponseError:
            print("No response from instrument during set_position_planned, continuing...")
        except minimalmodbus.ModbusException as e:
            print(f"Modbus error during set_position_planned: {e}, continuing...")

    def read_position_raw(self):
        """Read raw position counts and direction"""
        try:
            result = self.instrument.read_registers(Registers.POSITION, 3, functioncode=FUNCTION_READ_INPUT_REGISTERS)
            direction = result[0]
            position_counts = (result[2] << 16) | result[1]
            return direction, position_counts
        except minimalmodbus.NoResponseError:
            print("No response from instrument during read_position, returning None.")
            return None, None
        except minimalmodbus.ModbusException as e:
            print(f"Modbus error during read_position: {e}, returning None.")
            return None, None

    def read_velocity(self):
        """Read current velocity"""
        try:
            result = self.instrument.read_registers(Registers.VELOCITY, 2, functioncode=FUNCTION_READ_INPUT_REGISTERS)
            direction = result[0]
            velocity = result[1]
            return -velocity if direction else velocity
        except minimalmodbus.NoResponseError:
            print("No response from instrument during read_velocity, returning None.")
            return None
        except minimalmodbus.ModbusException as e:
            print(f"Modbus error during read_velocity: {e}, returning None.")
            return None

    def read_motor_status(self):
        """Read motor status"""
        try:
            status = self.instrument.read_register(Registers.MOTOR_STATUS, functioncode=FUNCTION_READ_INPUT_REGISTERS)
            return {
                'enabled': bool(status & 0x01),
                'in_position': bool(status & 0x02),
                'motor_stalled': bool(status & 0x04),
                'stall_protection': bool(status & 0x08)
            }
        except minimalmodbus.NoResponseError:
            print("No response from instrument during read_motor_status, returning None.")
            return None
        except minimalmodbus.ModbusException as e:
            print(f"Modbus error during read_motor_status: {e}, returning None.")
            return None

    def read_homing_status(self):
        """Read homing status"""
        try:
            status = self.instrument.read_register(Registers.HOMING_STATUS, functioncode=FUNCTION_READ_INPUT_REGISTERS)
            return {
                'encoder_ready': bool(status & 0x01),
                'calibration_ready': bool(status & 0x02),
                'homing_in_progress': bool(status & 0x04),
                'homing_failed': bool(status & 0x08),
                'high_precision': bool(status & 0x80)
            }
        except minimalmodbus.NoResponseError:
            print("No response from instrument during read_homing_status, returning None.")
            return None
        except minimalmodbus.ModbusException as e:
            print(f"Modbus error during read_homing_status: {e}, returning None.")
            return None

    def read_system_status(self):
        """Read system status"""
        try:
            result = self.instrument.read_registers(Registers.SYSTEM_STATUS, 17, functioncode=FUNCTION_READ_INPUT_REGISTERS)
            return {
                'bus_voltage': result[2],
                'bus_current': result[3],
                'phase_current': result[4],
                'encoder_raw': result[5],
                'encoder_calibrated': result[6],
                'target_position': struct.unpack('>i', struct.pack('>HH', result[8], result[7]))[0] / 10.0,
                'real_velocity': struct.unpack('>i', struct.pack('>HH', result[10], result[9]))[0],
                'real_position': struct.unpack('>i', struct.pack('>HH', result[12], result[11]))[0] / 10.0,
                'position_error': struct.unpack('>i', struct.pack('>HH', result[14], result[13]))[0] / 100.0,
                'temperature': result[15] if result[16] == 0 else -result[15],
                'homing_status': result[16] & 0xFF,
                'motor_status': (result[16] >> 8) & 0xFF
            }
        except minimalmodbus.NoResponseError:
            print("No response from instrument during read_system_status, returning None.")
            return None
        except minimalmodbus.ModbusException as e:
            print(f"Modbus error during read_system_status: {e}, returning None.")
            return None

    def read_driver_params(self):
        """Read driver parameters"""
        try:
            result = self.instrument.read_registers(Registers.DRIVER_PARAMS, 17, functioncode=FUNCTION_READ_INPUT_REGISTERS)
            return {
                'lock_key': result[1] & 0xFF,
                'control_mode': (result[1] >> 8) & 0xFF,
                'pulse_port': result[2] & 0xFF,
                'serial_port': (result[2] >> 8) & 0xFF,
                'en_pin_level': result[3] & 0xFF,
                'motor_direction': (result[3] >> 8) & 0xFF,
                'subdivision': result[4],
                'interpolation': result[5] & 0xFF,
                'auto_screen_off': (result[5] >> 8) & 0xFF,
                'lpfilter': result[6] & 0xFF,
                'open_loop_current': result[7],
                'closed_loop_current': result[8],
                'max_speed': result[9],
                'current_loop_bandwidth': result[10],
                'uart_baud': result[11] & 0xFF,
                'can_baud': (result[11] >> 8) & 0xFF,
                'checksum_mode': result[12] & 0xFF,
                'response_mode': (result[12] >> 8) & 0xFF,
                'position_precision': result[13] & 0xFF,
                'clog_protection': (result[13] >> 8) & 0xFF,
                'clog_speed': result[14],
                'clog_current': result[15],
                'clog_time': result[16],
            }
        except minimalmodbus.NoResponseError:
            print("No response from instrument during read_driver_params, returning None.")
            return None
        except minimalmodbus.ModbusException as e:
            print(f"Modbus error during read_driver_params: {e}, returning None.")
            return None

    def modify_driver_params(self, params, store=True):
        """Modify driver parameters"""
        values = [0x01 if store else 0x00]
        values.extend([
            (params.get('lock_key', 0) & 0xFF) | ((params.get('control_mode', 0) & 0xFF) << 8),
            (params.get('pulse_port', 0) & 0xFF) | ((params.get('serial_port', 0) & 0xFF) << 8),
            (params.get('en_pin_level', 0) & 0xFF) | ((params.get('motor_direction', 0) & 0xFF) << 8),
            params.get('subdivision', 0),
            (params.get('interpolation', 0) & 0xFF) | ((params.get('auto_screen_off', 0) & 0xFF) << 8),
            params.get('lpfilter', 0) & 0xFF,
            params.get('open_loop_current', 0),
            params.get('closed_loop_current', 0),
            params.get('max_speed', 0),
            params.get('current_loop_bandwidth', 0),
            (params.get('uart_baud', 0) & 0xFF) | ((params.get('can_baud', 0) & 0xFF) << 8),
            (params.get('checksum_mode', 0) & 0xFF) | ((params.get('response_mode', 0) & 0xFF) << 8),
            (params.get('position_precision', 0) & 0xFF) | ((params.get('clog_protection', 0) & 0xFF) << 8),
            params.get('clog_speed', 0),
            params.get('clog_current', 0),
            params.get('clog_time', 0),
        ])
        try:
            self.instrument.write_registers(Registers.MODIFY_DRIVER_PARAMS, values, functioncode=FUNCTION_WRITE_MULTIPLE_REGISTERS)
        except minimalmodbus.NoResponseError:
            print("No response from instrument during modify_driver_params, continuing...")
        except minimalmodbus.ModbusException as e:
            print(f"Modbus error during modify_driver_params: {e}, continuing...")

    def calibrate_encoder(self):
        """Calibrate the encoder"""
        try:
            self.instrument.write_register(Registers.CALIBRATE_ENCODER, 0x01, functioncode=FUNCTION_WRITE_SINGLE_REGISTER)
            print("Encoder calibration command sent.")
        except minimalmodbus.NoResponseError:
            print("No response from instrument during calibrate_encoder, continuing...")
        except minimalmodbus.ModbusException as e:
            print(f"Modbus error during calibrate_encoder: {e}, continuing...")

    def clear_position(self):
        """Clear the current position"""
        try:
            self.instrument.write_register(Registers.CLEAR_POSITION, 0x01, functioncode=FUNCTION_WRITE_SINGLE_REGISTER)
            print("Position cleared.")
        except minimalmodbus.NoResponseError:
            print("No response from instrument during clear_position, continuing...")
        except minimalmodbus.ModbusException as e:
            print(f"Modbus error during clear_position: {e}, continuing...")

    def factory_reset(self):
        """Perform factory reset"""
        try:
            self.instrument.write_register(Registers.FACTORY_RESET, 0x01, functioncode=FUNCTION_WRITE_SINGLE_REGISTER)
            print("Factory reset command sent.")
        except minimalmodbus.NoResponseError:
            print("No response from instrument during factory_reset, continuing...")
        except minimalmodbus.ModbusException as e:
            print(f"Modbus error during factory_reset: {e}, continuing...")

    def clear_clog(self):
        """Clear clog protection"""
        try:
            self.instrument.write_register(Registers.CLEAR_CLOG, 0x01, functioncode=FUNCTION_WRITE_SINGLE_REGISTER)
            print("Clog protection cleared.")
        except minimalmodbus.NoResponseError:
            print("No response from instrument during clear_clog, continuing...")
        except minimalmodbus.ModbusException as e:
            print(f"Modbus error during clear_clog: {e}, continuing...")

    def set_pid_params(self, pp_trapezoid, pp_direct, vp, vi, store=True):
        """Set PID parameters"""
        values = [0x01 if store else 0x00, pp_trapezoid, pp_direct, vp, vi]
        try:
            self.instrument.write_registers(Registers.SET_PID, values, functioncode=FUNCTION_WRITE_MULTIPLE_REGISTERS)
            print("PID parameters updated.")
        except minimalmodbus.NoResponseError:
            print("No response from instrument during set_pid_params, continuing...")
        except minimalmodbus.ModbusException as e:
            print(f"Modbus error during set_pid_params: {e}, continuing...")

    def set_homing_params(self, mode, direction, velocity, timeout, store=True):
        """Set homing parameters"""
        values = [
            0x01 if store else 0x00,
            mode,
            direction,
            velocity,
            timeout & 0xFFFF,
            (timeout >> 16) & 0xFFFF,
        ]
        try:
            self.instrument.write_registers(Registers.SET_HOMING_PARAMS, values, functioncode=FUNCTION_WRITE_MULTIPLE_REGISTERS)
            print("Homing parameters updated.")
        except minimalmodbus.NoResponseError:
            print("No response from instrument during set_homing_params, continuing...")
        except minimalmodbus.ModbusException as e:
            print(f"Modbus error during set_homing_params: {e}, continuing...")
