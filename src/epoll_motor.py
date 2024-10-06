import minimalmodbus
import serial
import logging
import time
import struct

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

VIRTUAL_PORT_PAIR = '/tmp/vserial2'


class MinimalModbusMotorControl:
    def __init__(self, port="/tmp/vserial2", baudrate=9600, motor_ids=None):
        self.port = port
        self.baudrate = baudrate
        self.motor_ids = motor_ids if motor_ids else [1, 2]
        self.instruments = {}
        self.setup_instruments()

    def setup_instruments(self):
        for motor_id in self.motor_ids:
            try:
                instrument = minimalmodbus.Instrument(self.port, slaveaddress=motor_id)
                instrument.serial.baudrate = self.baudrate
                instrument.serial.timeout = 0.5
                instrument.mode = minimalmodbus.MODE_RTU
                instrument.debug=True
                self.instruments[motor_id] = instrument
                logging.info(f'Instrument setup successful for motor {motor_id}')
            except serial.SerialException as e:
                logging.error(f"Could not open serial port for motor {motor_id}: {e}")

    def _write_registers(self, motor_id, values):
        try:
            self.instruments[motor_id].write_registers(motor_id, values)
            self._debug_output(motor_id, values)
        except Exception as e:
            logging.error(f"Failed to write registers for motor {motor_id}: {e}")

    def _read_registers(self, motor_id, count):
        try:
            return self.instruments[motor_id].read_registers(0, count)
        except Exception as e:
            logging.error(f"Failed to read registers for motor {motor_id}: {e}")
            return None

    def _debug_output(self, motor_id, values):
        hex_values = ' '.join([f'{value:02X}' for value in values])
        logging.info(f"485 command for motor {motor_id}: {hex_values}")

    def read_sys_params(self, motor_id):
        # 构造读取系统参数的请求
        request = [
            0x04,  # 功能码
            0x00, 0x78,  # 寄存器地址
            0x00, 0x11,  # 寄存器数量
        ]
        self._write_registers(motor_id, request)
        time.sleep(0.1)
        response = self._read_registers(motor_id, 39)  # 读取39个寄存器（如图中注释所示）
        
        if response:
            # 解析响应数据
            params = {
                "总字节数": response[2],
                "参数个数": response[3],
                "总线电压": response[4] * 256 + response[5],
                "总线电流": response[6] * 256 + response[7],
                "相电流": response[8] * 256 + response[9],
                "编码器原始值": response[10] * 256 + response[11],
                "编码器线性值": response[12] * 256 + response[13],
                "目标位置符号": response[14] * 256 + response[15],
                "电机目标位置": response[16] * 256 + response[17],
                "实时转速符号": response[18] * 256 + response[19],
                "电机实时转速": response[20] * 256 + response[21],
                "实时位置符号": response[22] * 256 + response[23],
                "电机实时位置": response[24] * 256 + response[25],
                "位置误差符号": response[26] * 256 + response[27],
                "实时温度符号": response[28] * 256 + response[29],
                "实时温度": response[30] * 256 + response[31],
                "回零状态标志位": response[32] * 256 + response[33],
                "电机状态标志位": response[34] * 256 + response[35],
            }
            return params
        else:
            return None

    def reset_cur_pos_to_zero(self, motor_id):
        self._write_registers(motor_id, [0x0A, 0x6D, 0x6B])

    def reset_clog_pro(self, motor_id):
        self._write_registers(motor_id, [0x0E, 0x52, 0x6B])

    def modify_ctrl_mode(self, motor_id, save_flag, ctrl_mode):
        self._write_registers(motor_id, [0x46, 0x69, 1 if save_flag else 0, ctrl_mode, 0x6B])

    def en_control(self, motor_id, state, sync_flag):
        self._write_registers(motor_id, [0xF3, 0xAB, 1 if state else 0, 1 if sync_flag else 0, 0x6B])

    def vel_control(self, motor_id, direction, velocity, acceleration, sync_flag):
        self._write_registers(motor_id, [0xF6, direction, velocity >> 8, velocity & 0xFF, 
                                         acceleration, 1 if sync_flag else 0, 0x6B])

    def pos_control(self, motor_id, direction, velocity, acceleration, pulse_count, relative_flag, sync_flag):
        self._write_registers(motor_id, [0xFD, direction, velocity >> 8, velocity & 0xFF, 
                                         acceleration, pulse_count >> 24, (pulse_count >> 16) & 0xFF, 
                                         (pulse_count >> 8) & 0xFF, pulse_count & 0xFF, 
                                         1 if relative_flag else 0, 1 if sync_flag else 0, 0x6B])

    def stop_now(self, motor_id, sync_flag):
        self._write_registers(motor_id, [0xFE, 0x98, 1 if sync_flag else 0, 0x6B])

    def synchronous_motion(self, motor_id):
        self._write_registers(motor_id, [0xFF, 0x66, 0x6B])

    def origin_set_o(self, motor_id, save_flag):
        self._write_registers(motor_id, [0x93, 0x88, 1 if save_flag else 0, 0x6B])

    def origin_modify_params(self, motor_id, save_flag, o_mode, o_dir, o_vel, o_tm, sl_vel, sl_ma, sl_ms, pot_flag):
        self._write_registers(motor_id, [0x4C, 0xAE, 1 if save_flag else 0, o_mode, o_dir, 
                                         o_vel >> 8, o_vel & 0xFF, o_tm >> 24, (o_tm >> 16) & 0xFF, 
                                         (o_tm >> 8) & 0xFF, o_tm & 0xFF, sl_vel >> 8, sl_vel & 0xFF, 
                                         sl_ma >> 8, sl_ma & 0xFF, sl_ms >> 8, sl_ms & 0xFF, 
                                         1 if pot_flag else 0, 0x6B])

    def origin_trigger_return(self, motor_id, o_mode, sync_flag):
        self._write_registers(motor_id, [0x9A, o_mode, 1 if sync_flag else 0, 0x6B])

    def origin_interrupt(self, motor_id):
        self._write_registers(motor_id, [0x9C, 0x48, 0x6B])

    def real_time_location(self):
        positions = {}
        for motor_id in self.motor_ids:
            pos = self.read_sys_params(motor_id, 'S_CPOS')
            if pos:
                motor_cur_pos = struct.unpack('>I', struct.pack('>HH', pos[2], pos[3]))[0] * 360.0 / 65536.0
                if pos[1]:
                    motor_cur_pos = -motor_cur_pos
                positions[f'Motor{motor_id}'] = f'{motor_cur_pos:.1f}'
            else:
                positions[f'Motor{motor_id}'] = 'Failed to read'
        
        print(', '.join([f'{k}: {v}' for k, v in positions.items()]))
        return positions

def main():
    motor_control = MinimalModbusMotorControl(motor_ids=[1,])

    for motor_id in motor_control.motor_ids:
        print(f"Reading parameters for Motor {motor_id}:")
        params = motor_control.read_sys_params(motor_id)
        if params:
            for key, value in params.items():
                print(f"  {key}: {value}")
        else:
            print("  Failed to read parameters")
        print("-" * 40)

if __name__ == '__main__':
    main()