import minimalmodbus
import serial
import logging
import time
import json

# 配置日志
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

# Modbus 寄存器地址（根据Emm42_V5.0驱动器的Modbus映射表定义）
POSITION_REGISTER = 0xFD
VELOCITY_REGISTER = 0xF6
ENABLE_REGISTER = 0xF3
STOP_REGISTER = 0xFE
SYNC_TRIGGER_REGISTER = 0xFF

class ModbusMotorController:
    def __init__(self, port, baud, slave_address):
        self.instrument = self.setup_instrument(port, baud, slave_address)
        self.address = slave_address

    def setup_instrument(self, port, baud, slave_address):
        try:
            instrument = minimalmodbus.Instrument(port, slave_address)
            instrument.serial.baudrate = baud
            instrument.serial.timeout = 0.5
            instrument.mode = minimalmodbus.MODE_RTU
            return instrument
        except Exception as e:
            logging.error(f"无法连接到电机 {slave_address}: {e}")
            return None

    def set_position(self, target_position, speed, acceleration, relative=True, sync=False):
        try:
            direction = 1 if target_position >= 0 else 0
            abs_position = abs(int(target_position))
            data = [direction, (speed >> 8) & 0xFF, speed & 0xFF, acceleration,
                    (abs_position >> 24) & 0xFF, (abs_position >> 16) & 0xFF,
                    (abs_position >> 8) & 0xFF, abs_position & 0xFF,
                    1 if relative else 0, 1 if sync else 0]
            self.instrument.write_registers(POSITION_REGISTER, data)
            logging.info(f"电机 {self.address} 设置位置: {target_position}")
        except Exception as e:
            logging.error(f"设置位置失败: {e}")

    def set_velocity(self, velocity, acceleration, sync=False):
        try:
            direction = 1 if velocity >= 0 else 0
            abs_velocity = abs(int(velocity))
            data = [direction, (abs_velocity >> 8) & 0xFF, abs_velocity & 0xFF, acceleration,
                    1 if sync else 0]
            self.instrument.write_registers(VELOCITY_REGISTER, data)
            logging.info(f"电机 {self.address} 设置速度: {velocity}")
        except Exception as e:
            logging.error(f"设置速度失败: {e}")

    def stop_motor(self, sync=False):
        try:
            self.instrument.write_registers(STOP_REGISTER, [0x98, 1 if sync else 0])
            logging.info(f"电机 {self.address} 停止")
        except Exception as e:
            logging.error(f"停止电机失败: {e}")

    def enable_motor(self, enable=True, sync=False):
        try:
            self.instrument.write_registers(ENABLE_REGISTER, [0xAB, 1 if enable else 0, 1 if sync else 0])
            logging.info(f"电机 {self.address} {'使能' if enable else '禁用'}")
        except Exception as e:
            logging.error(f"{'使能' if enable else '禁用'}电机失败: {e}")

    def trigger_sync_motion(self):
        try:
            self.instrument.write_register(SYNC_TRIGGER_REGISTER, 0x66)
            logging.info("触发同步运动")
        except Exception as e:
            logging.error(f"触发同步运动失败: {e}")

    def get_position(self):
        try:
            response = self.instrument.read_registers(0x36, 3)
            sign = -1 if response[0] else 1
            position = sign * (response[1] << 16 | response[2])
            return position * 360.0 / 65536.0  # 转换为角度
        except Exception as e:
            logging.error(f"读取位置失败: {e}")
            return None

class RailController:
    def __init__(self, config_file):
        self.config = self.load_config(config_file)
        self.motor = ModbusMotorController(self.config['port'], self.config['baud'], self.config['motor_address'])
        self.positions = self.config['positions']
        self.current_position_index = 0
        self.multi_sync_enabled = self.config.get('multi_sync_enabled', False)

    def load_config(self, config_file):
        with open(config_file, 'r') as f:
            return json.load(f)

    def move_to_next_position(self):
        if self.current_position_index < len(self.positions):
            next_position = self.positions[self.current_position_index]
            self.motor.set_position(
                next_position,
                self.config['speed'],
                self.config['acceleration'],
                relative=False,
                sync=self.multi_sync_enabled
            )
            if self.multi_sync_enabled:
                self.motor.trigger_sync_motion()
            self.wait_for_completion()
            self.current_position_index += 1
            logging.info(f"移动到位置 {self.current_position_index} 完成")
        else:
            logging.info("所有位置已到达")

    def wait_for_completion(self):
        target_position = self.positions[self.current_position_index]
        while abs(self.motor.get_position() - target_position) > 0.5:  # 0.5度误差
            time.sleep(0.1)

    def reset(self):
        self.current_position_index = 0
        self.motor.set_position(0, self.config['speed'], self.config['acceleration'], relative=False, sync=self.multi_sync_enabled)
        if self.multi_sync_enabled:
            self.motor.trigger_sync_motion()
        self.wait_for_completion()
        logging.info("重置到初始位置")

    def run(self):
        self.motor.enable_motor(True, sync=self.multi_sync_enabled)
        if self.multi_sync_enabled:
            self.motor.trigger_sync_motion()
        while self.current_position_index < len(self.positions):
            self.move_to_next_position()
            # 这里可以添加到达位置后的回调函数，比如拍照等操作
        self.motor.enable_motor(False, sync=self.multi_sync_enabled)
        if self.multi_sync_enabled:
            self.motor.trigger_sync_motion()

def main():
    logging.info("开始初始化电机控制系统")
    rail_controller = RailController('/home/wsn640/tzj/2Channel-UART-over-RS485-on-RPI/src/config.json')
    
    try:
        rail_controller.run()
    except Exception as e:
        logging.error(f"运行过程中发生错误: {e}")
    finally:
        rail_controller.motor.stop_motor(sync=rail_controller.multi_sync_enabled)
        if rail_controller.multi_sync_enabled:
            rail_controller.motor.trigger_sync_motion()
        logging.info("电机控制系统关闭")

if __name__ == '__main__':
    main()
    
    
'''这个修改后的代码包含了以下主要变化：

修改了 ModbusMotorController 类以适应 Emm42_V5.0 驱动器的 Modbus 寄存器映射。
添加了多机协同驱动功能。在每个控制命令中添加了 sync 参数，当启用时会设置多机同步标志。
添加了 trigger_sync_motion 方法来触发同步运动。
在 RailController 类中添加了 multi_sync_enabled 属性，可以通过配置文件控制是否启用多机同步。
更新了主程序逻辑以适应这些变化，包括在适当的时候触发同步运动。
要使用这个程序，你需要创建一个 config.json 文件，包含以下内容：


Configuration File (config.json)
点击以打开 code
你可以根据需要修改这个配置文件。如果你想禁用多机同步，只需将 "multi_sync_enabled" 设置为 false。

这个实现允许你控制单个电机，同时也支持多机协同驱动。当 multi_sync_enabled 为 true 时，程序会在每个命令后发送同步触发信号，确保多个驱动板同步运行。

Copy
Re
'''