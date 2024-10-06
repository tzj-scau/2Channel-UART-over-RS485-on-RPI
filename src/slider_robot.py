import minimalmodbus
import serial
import time
import struct
import math  # 需要引入 math 模块进行数学计算

# Constants
SLAVE_ADDRESS = 1
BAUD_RATE = 9600  # Changed to match the default in the document
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


class SliderRobot:
    def __init__(self, port, slave_address, baud_rate=9600, reduction_ratio=19, wheel_diameter=34.0, counts_per_revolution=3200):
        self.instrument = minimalmodbus.Instrument(port, slave_address)
        self.instrument.serial.baudrate = baud_rate
        self.instrument.serial.timeout = 1
        # self.instrument.serial.parity = serial.PARITY_NONE
        # self.instrument.serial.bytesize = 8
        # self.instrument.serial.stopbits = 1
        self.instrument.mode = minimalmodbus.MODE_RTU
        self.instrument.debug = True

        # 添加机械参数
        self.reduction_ratio = reduction_ratio  # 减速比
        self.wheel_diameter = wheel_diameter    # 轮子直径，单位：毫米
        self.counts_per_revolution = counts_per_revolution  # 每转的计数（步数），根据电机和驱动器的设置

    def enable_motor(self, enable=True, sync_flag=False):
        values = [0xAB << 8 | 0x01 if enable else 0x00, (0x01 if sync_flag else 0x00) << 8 | 0x00]
        try:
            self.instrument.write_register(Registers.ENABLE_CONTROL, values, functioncode=FUNCTION_WRITE_MULTIPLE_REGISTERS)
        except minimalmodbus.NoResponseError:
            print("No response from instrument during enable_motor, continuing...")
        except minimalmodbus.ModbusException as e:
            print(f"Modbus error during enable_motor: {e}, continuing...")

    def stop(self, sync_flag=False):
        """立即停止电机"""
        value = (0x98 << 8) | (0x01 if sync_flag else 0x00)
        try:
            self.instrument.write_register(Registers.STOP, value, functioncode=FUNCTION_WRITE_MULTIPLE_REGISTERS)
        except minimalmodbus.NoResponseError:
            print("No response from instrument during stop, continuing...")
        except minimalmodbus.ModbusException as e:
            print(f"Modbus error during stop: {e}, continuing...")

    def trigger_homing(self, mode=0, sync_flag=False):
        """触发回零操作"""
        value = (mode << 8) | (0x01 if sync_flag else 0x00)
        try:
            self.instrument.write_register(Registers.TRIGGER_HOMING, value, functioncode=FUNCTION_WRITE_MULTIPLE_REGISTERS)
        except minimalmodbus.NoResponseError:
            print("No response from instrument during trigger_homing, continuing...")
        except minimalmodbus.ModbusException as e:
            print(f"Modbus error during trigger_homing: {e}, continuing...")

    def set_velocity(self, velocity, acceleration=300, sync_flag=False):
        """设置速度"""
        direction = 0 if velocity >= 0 else 1
        abs_velocity = abs(int(velocity))
        values = [direction, acceleration, abs_velocity, 0x01 if sync_flag else 0x00]
        try:
            self.instrument.write_register(Registers.VELOCITY_CONTROL, values, functioncode=FUNCTION_WRITE_MULTIPLE_REGISTERS)
        except minimalmodbus.NoResponseError:
            print("No response from instrument during set_velocity, continuing...")
        except minimalmodbus.ModbusException as e:
            print(f"Modbus error during set_velocity: {e}, continuing...")

    def set_position(self, position, velocity=300, relative=True, sync_flag=False):
        """设置目标位置"""
        direction = 0 if position >= 0 else 1
        abs_position_mm = abs(position)

        # 计算每转的线性移动距离（单位：毫米）
        linear_movement_per_rev = (math.pi * self.wheel_diameter) / self.reduction_ratio

        # 计算每毫米对应的电机计数（步数）
        counts_per_mm = self.counts_per_revolution / linear_movement_per_rev

        # 计算目标位置对应的电机计数（步数）
        counts = int(abs_position_mm * counts_per_mm)

        # 将计数分解为高低位两个16位寄存器
        low_word = counts & 0xFFFF
        high_word = (counts >> 16) & 0xFFFF

        # 构建发送给驱动器的值列表
        values = [
            direction,
            velocity*10,
            low_word,
            high_word,
            ((0x00 if relative else 0x01) << 8) | (0x01 if sync_flag else 0x00)
        ]
        try:
            # self.instrument.write_register(Registers.POSITION_CONTROL_DIRECT, values, functioncode=FUNCTION_WRITE_MULTIPLE_REGISTERS)
            self.instrument.write_registers(Registers.POSITION_CONTROL_DIRECT, values)
        except minimalmodbus.NoResponseError:
            print("No response from instrument during set_position, continuing...")
        except minimalmodbus.ModbusException as e:
            print(f"Modbus error during set_position: {e}, continuing...")


    def set_position_planned(self, position_mm, max_velocity_rpm, accel_rpm_per_s, decel_rpm_per_s, relative=True, sync_flag=False):
        """设置带梯形加减速曲线的目标位置（线性位移模式）"""
        
        # 确定方向 (0: CW, 1: CCW)
        direction = 0 if position_mm >= 0 else 1
        abs_position_mm = abs(position_mm)

        # 从 set_position 函数继承的线性位移计算
        # 计算每转的线性移动距离（单位：毫米）
        linear_movement_per_rev = (math.pi * self.wheel_diameter) / self.reduction_ratio

        # 计算每毫米对应的电机计数（步数）
        counts_per_mm = self.counts_per_revolution / linear_movement_per_rev

        # 计算目标位置对应的电机计数（步数）
        counts = int(abs_position_mm * counts_per_mm)

        # 根据文档要求将速度放大 10 倍
        max_velocity_scaled = int(max_velocity_rpm * 10)
        accel_scaled = int(accel_rpm_per_s * 10)
        decel_scaled = int(decel_rpm_per_s * 10)

        # 拆分电机步数（计数）寄存器，低位在前
        position_low = counts & 0xFFFF
        position_high = (counts >> 16) & 0xFFFF

        # 相对/绝对运动标志和同步标志，分别占两个寄存器
        movement_flags = 0x0000  # 初始化为0
        movement_flags |= (0x01 if not relative else 0x00)  # 绝对运动为1，相对运动为0
        sync_flag_value = 0x01 if sync_flag else 0x00      # 同步标志

        # 构建发送给驱动器的寄存器值列表（符合文档中的寄存器顺序）
        values = [
            direction,          # 寄存器1: 方向 (0: CW, 1: CCW)
            accel_scaled,       # 寄存器2: 加速 (RPM/s)
            decel_scaled,       # 寄存器3: 减速 (RPM/s)
            max_velocity_scaled,# 寄存器4: 最大速度 (RPM)
            position_low,       # 寄存器5: 位置角度低位（步数低位）
            position_high,      # 寄存器6: 位置角度高位（步数高位）
            (movement_flags << 8) | sync_flag_value  # 寄存器7: 相对/绝对运动标志 + 同步标志
        ]
        
        try:
            # 使用 Function Code 0x10 (Write Multiple Registers)，寄存器地址 0x00F6，寄存器数量 7
            self.instrument.write_registers(Registers.POSITION_CONTROL_PLANNED, values)
        except minimalmodbus.NoResponseError:
            print("No response from instrument during set_position_planned, continuing...")
        except minimalmodbus.ModbusException as e:
            print(f"Modbus error during set_position_planned: {e}, continuing...")






    def read_position(self):
        """读取当前位置，增加错误处理"""
        try:
            # 调用标准的 read_registers 方式读取寄存器
            result = self.instrument.read_registers(Registers.POSITION, 3, functioncode=4)
            direction = result[0]
            position_counts = (result[2] << 16) | result[1]

            # 计算每转的线性移动距离（mm/rev）
            linear_movement_per_rev = (math.pi * self.wheel_diameter) / self.reduction_ratio

            # 计算每计数对应的线性距离（mm/count）
            mm_per_count = linear_movement_per_rev / self.counts_per_revolution

            # 将计数转换为线性位置（毫米）
            position_mm = position_counts * mm_per_count

            return -position_mm if direction else position_mm

        except minimalmodbus.NoResponseError:
            print("No response from instrument during read_position, returning None.")
            return None

        except minimalmodbus.InvalidResponseError as e:
            print("Invalid response received, ignoring CRC error and processing data...")

            try:
                # 直接发送预构建的 Modbus 请求帧
                request = bytes([0x01, 0x04, 0x00, 0x46, 0x00, 0x03, 0x51, 0xDE])
                
                # 根据请求的寄存器数量计算要读取的字节数：1(地址) + 1(功能码) + 1(字节数) + 3*2(寄存器数据) + 2(CRC)
                number_of_bytes_to_read = 1 + 1 + 1 + (3 * 2) + 2  # 1+1+1+(3*2)+2 = 11 字节

                # 调用 MinimalModbus 的 _communicate 方法，直接发送构建好的请求
                raw_response = self.instrument._communicate(request, number_of_bytes_to_read)
                print(f"Raw response: {raw_response}")
                
                # 手动解析数据
                if len(raw_response) >= 9:
                    direction = int(raw_response[3])
                    position_low = int(raw_response[4] << 8 | raw_response[5])  # 低位
                    position_high = int(raw_response[6] << 8 | raw_response[7])  # 高位

                    # 计算 position_counts
                    position_counts = (position_high << 16) | position_low

                    # 计算每转的线性移动距离（mm/rev）
                    linear_movement_per_rev = (math.pi * self.wheel_diameter) / self.reduction_ratio

                    # 计算每计数对应的线性距离（mm/count）
                    mm_per_count = linear_movement_per_rev / self.counts_per_revolution

                    # 将计数转换为线性位置（毫米）
                    position_mm = position_counts * mm_per_count

                    return -position_mm if direction else position_mm

                else:
                    print("Response too short to parse.")
                    return None

            except Exception as parse_error:
                print(f"Failed to parse response data: {parse_error}, returning None.")
                return None

        except minimalmodbus.ModbusException as e:
            print(f"Modbus error during read_position: {e}, returning None.")
            return None




    def read_velocity(self):
        """读取当前速度"""
        try:
            result = self.instrument.read_register(Registers.VELOCITY, 2, functioncode=FUNCTION_READ_INPUT_REGISTERS)
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
        """读取电机状态"""
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
        """读取回零状态"""
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
        """读取系统状态"""
        try:
            result = self.instrument.read_registers(Registers.SYSTEM_STATUS, 17)
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
        """读取驱动器参数"""
        try:
            result = self.instrument.read_registers(Registers.DRIVER_PARAMS, 17)
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
        """修改驱动器参数"""
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
            self.instrument.write_register(Registers.MODIFY_DRIVER_PARAMS, values, functioncode=FUNCTION_WRITE_MULTIPLE_REGISTERS)
        except minimalmodbus.NoResponseError:
            print("No response from instrument during modify_driver_params, continuing...")
        except minimalmodbus.ModbusException as e:
            print(f"Modbus error during modify_driver_params: {e}, continuing...")


from typing import List, Callable
import time

class SliderRobotManager:
    def __init__(self, slider_robot: SliderRobot, positions: List[float]):
        self.robot = slider_robot
        self.positions = positions  # 栏位位置列表
        self.current_position_index = 0  # 当前位置索引
        self.is_homed = False  # 是否已经回零
        self.callback_queue = []  # 回调函数队列

    def home(self, sync_flag=False):
        """执行回零操作"""
        print("Homing the robot...")
        self.robot.trigger_homing(mode=0, sync_flag=sync_flag)
        while True:
            status = self.robot.read_homing_status()
            if not status['homing_failed'] and not status['homing_in_progress']:
                break
            time.sleep(0.1)
        self.is_homed = True
        self.current_position_index = 0
        print("Homing completed.")

    def clear_position(self):
        """清零当前位置"""
        print("Clearing current position...")
        self.robot.instrument.write_register(Registers.CLEAR_POSITION, 0x01, functioncode=FUNCTION_WRITE_SINGLE_REGISTER)
        self.current_position_index = 0
        print("Position cleared.")

    def move_to_position(self, position_index: int, use_planned_motion: bool = True, callback: Callable = None):
        """移动到指定栏位"""
        if not self.is_homed:
            print("Robot not homed. Homing first...")
            self.home()

        if 0 <= position_index < len(self.positions):
            target_position = self.positions[position_index]
            print(f"Moving to position {position_index} ({target_position})...")
            
            if use_planned_motion:
                self.robot.set_position_planned(target_position, max_velocity_rpm=300, accel_rpm_per_s=250, decel_rpm_per_s=250, sync_flag=False)
            else:
                self.robot.set_position(target_position, velocity=300, relative=False)
            
            # 等待移动完成
            while not self._is_in_position():
                time.sleep(0.5)
            
            self.current_position_index = position_index
            print(f"Moved to position {position_index}.")

            if callback:
                self.callback_queue.append(callback)
        else:
            print("Invalid position index.")

    def _is_in_position(self,) -> bool:
        """检查是否到达目标位置"""
        time.sleep(3)  # 给电机一些时间来更新状态
        return self.robot.read_motor_status()['in_position']

    def move_to_next_position(self, use_planned_motion: bool = True, callback: Callable = None):
        """移动到下一个栏位"""
        next_position = (self.current_position_index + 1) % len(self.positions)
        self.move_to_position(next_position, use_planned_motion, callback)

    def move_to_previous_position(self, use_planned_motion: bool = True, callback: Callable = None):
        """移动到上一个栏位"""
        prev_position = (self.current_position_index - 1) % len(self.positions)
        self.move_to_position(prev_position, use_planned_motion, callback)

    def execute_callbacks(self):
        """执行所有排队的回调函数"""
        while self.callback_queue:
            callback = self.callback_queue.pop(0)
            callback()

    def take_photo(self):
        """模拟拍照操作"""
        print(f"Taking photo at position {self.current_position_index}...")
        time.sleep(1)  # 模拟拍照时间
        print("Photo taken.")

    def scan_all_positions(self, use_planned_motion: bool = True, callback: Callable = None):
        """扫描所有栏位并执行回调"""
        for i in range(len(self.positions)):
            self.move_to_position(i, use_planned_motion, callback)
            self.execute_callbacks()

    def get_current_position(self):
        """获取当前位置信息"""
        return {
            "index": self.current_position_index,
            "position": self.robot.read_position(),
            "target_position": self.positions[self.current_position_index],
            "velocity": self.robot.read_velocity(),
            "motor_status": self.robot.read_motor_status(),
            "homing_status": self.robot.read_homing_status()
        }

    def emergency_stop(self):
        """紧急停止"""
        print("Emergency stop triggered!")
        self.robot.stop(sync_flag=False)
        self.callback_queue.clear()  # 清除所有待执行的回调

    def resume(self):
        """从紧急停止中恢复"""
        print("Resuming operation...")
        self.robot.enable_motor(enable=True)
        print("Robot enabled. Ready to receive new commands.")

    def set_pid_params(self, pp_trapezoid, pp_direct, vp, vi, store=True):
        """设置PID参数"""
        self.robot.set_pid_params(pp_trapezoid, pp_direct, vp, vi, store)
        print("PID parameters updated.")

    def set_homing_params(self, mode, direction, velocity, timeout, store=True):
        """设置回零参数"""
        self.robot.set_homing_params(mode, direction, velocity, timeout, store)
        print("Homing parameters updated.")

    def get_system_status(self):
        """获取系统状态"""
        return self.robot.read_system_status()

    def get_driver_params(self):
        """获取驱动器参数"""
        return self.robot.read_driver_params()

    def modify_driver_params(self, params, store=True):
        """修改驱动器参数"""
        self.robot.modify_driver_params(params, store)
        print("Driver parameters updated.")

    def calibrate_encoder(self):
        """校准编码器"""
        print("Calibrating encoder...")
        self.robot.instrument.write_register(Registers.CALIBRATE_ENCODER, 0x01, functioncode=FUNCTION_WRITE_SINGLE_REGISTER)
        time.sleep(2)  # 等待校准完成
        print("Encoder calibration completed.")

    def clear_clog(self):
        """清除堵转保护"""
        print("Clearing clog protection...")
        self.robot.instrument.write_register(Registers.CLEAR_CLOG, 0x01, functioncode=FUNCTION_WRITE_SINGLE_REGISTER)
        print("Clog protection cleared.")

    def factory_reset(self):
        """恢复出厂设置"""
        print("Performing factory reset...")
        self.robot.instrument.write_register(Registers.FACTORY_RESET, 0x01, functioncode=FUNCTION_WRITE_SINGLE_REGISTER)
        time.sleep(2)  # 等待重置完成
        print("Factory reset completed. Please reconfigure the robot.")




import time
from typing import List

def test_callback():
    print("Callback function executed!")

def main():
    # 初始化 SliderRobot，传入机械参数
    robot = SliderRobot(
        port='/tmp/vserial2',
        slave_address=1,
        baud_rate=9600,
        reduction_ratio=19,
        wheel_diameter=34.0,
        counts_per_revolution=3200  # 根据您的电机和驱动器设置调整此值
    )
    
    # 定义栏位位置（以毫米为单位）
    positions: List[float] = [1000.0, 2000.0, 3000.0, 4000.0]
    
    # 其余代码保持不变...

    
    # 初始化 SliderRobotManager
    manager = SliderRobotManager(robot, positions)
    
    try:
        # 1. 回零操作
        print("\n--- Testing Homing ---")
        manager.home()
        
        # 2. 移动到各个位置
        print("\n--- Testing Position Movement ---")
        for i in range(len(positions)):
            manager.move_to_position(i, use_planned_motion=True, callback=test_callback)
            manager.execute_callbacks()
            manager.take_photo()
        
        # 3. 测试下一个和上一个位置移动
        print("\n--- Testing Next and Previous Position Movement ---")
        manager.move_to_next_position(callback=test_callback)
        manager.execute_callbacks()
        manager.move_to_previous_position(callback=test_callback)
        manager.execute_callbacks()
        
        # 4. 获取当前位置信息
        print("\n--- Current Position Info ---")
        position_info = manager.get_current_position()
        print(position_info)
        
        # 5. 测试系统状态和驱动器参数
        print("\n--- System Status ---")
        system_status = manager.get_system_status()
        print(system_status)
        
        print("\n--- Driver Parameters ---")
        driver_params = manager.get_driver_params()
        print(driver_params)
        
        # # 6. 修改PID参数（注意：这里使用示例值，实际使用时需要根据具体情况调整）
        # print("\n--- Updating PID Parameters ---")
        # manager.set_pid_params(pp_trapezoid=100, pp_direct=200, vp=300, vi=400)
        
        # # 7. 测试紧急停止和恢复
        # print("\n--- Testing Emergency Stop and Resume ---")
        # manager.move_to_position(2, use_planned_motion=True)  # 开始移动
        # time.sleep(0.5)  # 等待一小段时间
        # manager.emergency_stop()  # 紧急停止
        # time.sleep(1)  # 等待一段时间
        # manager.resume()  # 恢复操作
        
        # 8. 测试扫描所有位置
        print("\n--- Scanning All Positions ---")
        manager.scan_all_positions(callback=test_callback)
        
        # 9. 测试清除位置
        print("\n--- Clearing Position ---")
        manager.clear_position()
        
        # # 10. 测试校准编码器
        # print("\n--- Calibrating Encoder ---")
        # manager.calibrate_encoder()
        
    except Exception as e:
        print(f"An error occurred: {e}")
    
    finally:
        # 确保在程序结束时停止电机
        manager.emergency_stop()
        print("Test completed. Motor stopped.")

if __name__ == "__main__":
    main()