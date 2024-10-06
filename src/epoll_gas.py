import minimalmodbus
import serial
import logging
import time

class GasSensor:
    """
    气体传感器类，用于与Modbus RTU协议的气体传感器进行通信。
    """

    # 技术文档中的常量
    CONCENTRATION_REGISTER = 0x6001
    ALARM_STATUS_REGISTER = 0x6000
    RUNNING_STATUS_REGISTER = 0x6006

    # 零点屏蔽的阈值（PPM）
    ZERO_POINT_THRESHOLD = 3
    # 每个命令之间的最小时间间隔（秒）
    MIN_TIME_INTERVAL = 0.5

    # 运行状态定义
    RUNNING_STATUS = {
        0x0000: "模组忙，正在预热或执行命令",
        0x0001: "模组空闲或执行成功",
        0x0002: "执行失败",
        0x0003: "不支持的命令",
        0x0004: "参数无效",
        0x0005: "执行超时",
        0x0100: "执行命令步骤0",
        0x0101: "执行命令步骤1",
        0x0102: "执行命令步骤2",
        0x0103: "执行命令步骤3"
    }

    # 报警状态定义，当bit15=1时
    ALARM_STATUS_BIT15_1 = {
        0x4000: "空闲",
        0x2000: "空闲",
        0x1000: "空闲",
        0x0800: "空闲",
        0x0400: "空闲",
        0x0200: "空闲",
        0x0100: "空闲",
        0x0080: "空闲",
        0x0040: "高浓度保护",
        0x0020: "传感器丢失或损坏",
        0x0010: "超量程",
        0x0008: "TWA报警",
        0x0004: "STEL报警",
        0x0002: "高报",
        0x0001: "低报"
    }

    def __init__(self, gas_name, port, baud, slave_address, decimal_point=0):
        """
        初始化气体传感器实例。

        :param gas_name: 气体名称，例如 'CO2'。
        :param port: 串口端口，例如 '/dev/ttyUSB0'。
        :param baud: 波特率，例如 9600。
        :param slave_address: Modbus从站地址。
        :param decimal_point: 小数点位数，默认为 0。
        """
        self.gas_name = gas_name
        self.port = port
        self.baud = baud
        self.slave_address = slave_address
        self.decimal_point = decimal_point

        self.instrument = self.setup_instrument()
        self.last_command = None
        self.running_status = None
        self.alarm_status = None
        self.next_read_time = time.time()
        self.read_interval = 5  # 默认读取间隔（秒）
        self.fail_count = 0

    def setup_instrument(self):
        """
        设置 Modbus 仪器。
        """
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.5)
            instrument = minimalmodbus.Instrument(ser, slaveaddress=self.slave_address)
            instrument.mode = minimalmodbus.MODE_RTU
            instrument.serial.timeout = 0.5
            instrument.debug = True  # 启用调试模式，详细日志
            return instrument
        except serial.SerialException as e:
            logging.error(f"无法打开传感器 {self.slave_address} 的串口: {e}")
            return None

    def check_running_status(self):
        """
        检查传感器的运行状态。
        """
        try:
            time.sleep(self.MIN_TIME_INTERVAL)  # 确保命令之间的最小时间间隔
            status = self.instrument.read_register(self.RUNNING_STATUS_REGISTER, functioncode=3)
            self.running_status = status
            status_desc = self.RUNNING_STATUS.get(status, "未知状态")
            logging.debug(f"{self.gas_name} 运行状态: {status_desc}")
            return status
        except Exception as e:
            logging.error(f"{self.gas_name} 读取运行状态失败: {e}")
            return None

    def check_alarm_status(self):
        """
        检查传感器的报警状态。
        """
        try:
            time.sleep(self.MIN_TIME_INTERVAL)  # 确保命令之间的最小时间间隔
            status = self.instrument.read_register(self.ALARM_STATUS_REGISTER, functioncode=3)
            self.alarm_status = status

            if status & 0x8000:  # bit15 = 1, 预热完成
                alarms = []
                for bit, description in self.ALARM_STATUS_BIT15_1.items():
                    if status & bit:
                        alarms.append(description)
                if not alarms:
                    alarms.append("无报警")
                logging.debug(f"{self.gas_name} 报警状态: 预热完成, {', '.join(alarms)}")
            else:  # bit15 = 0, 正在预热
                preheating_time = status & 0x7FFF
                logging.debug(f"{self.gas_name} 预热中，剩余时间: {preheating_time * 0.1:.1f} 秒")
            return status
        except Exception as e:
            logging.error(f"{self.gas_name} 读取报警状态失败: {e}")
            return None

    def read_gas_concentration(self):
        """
        读取气体浓度值。
        """
        try:
            time.sleep(self.MIN_TIME_INTERVAL)  # 确保命令之间的最小时间间隔
            concentration = self.instrument.read_register(self.CONCENTRATION_REGISTER, functioncode=3)
            actual_concentration = concentration / (10 ** self.decimal_point)
            # 如果浓度值低于阈值，则将其屏蔽为0
            if abs(actual_concentration) <= self.ZERO_POINT_THRESHOLD:
                actual_concentration = 0
            return actual_concentration
        except Exception as e:
            logging.error(f"{self.gas_name} 读取气体浓度失败: {e}")
            return None

    def execute_command(self, command):
        """
        执行指定的命令。

        :param command: 要执行的命令，例如 'read_concentration'。
        """
        self.last_command = command
        result = None
        try:
            if command == "read_concentration":
                result = self.read_gas_concentration()
                if result is not None:
                    logging.info(f"{self.gas_name} 执行成功: {self.last_command}")
                else:
                    # 如果执行失败，检查运行状态
                    status = self.check_running_status()
                    # 如果状态正常，假设是零点屏蔽
                    if status == 0x0001:  # 模组空闲或执行成功
                        result = 0
                        logging.info(f"{self.gas_name} 被零点屏蔽，气体浓度设为 0")
        except Exception as e:
            logging.error(f"{self.gas_name} 执行命令失败: {command}, 错误: {e}")
            status = self.check_running_status()
            if status == 0x0001:
                result = 0
                logging.info(f"{self.gas_name} 被零点屏蔽，气体浓度设为 0")  # 假设是零点屏蔽
        return result

import logging
import time

def main():
    logging.basicConfig(level=logging.DEBUG)

    # 配置参数
    VIRTUAL_PORT_PAIR = '/tmp/vserial2'
    BAUD = 9600
    PREHEAT_TIME = 5  # 预热时间（秒）

    # 传感器地址和小数点位数配置
    SENSOR_CONFIG = {
        'CO2': {'address': 5, 'decimal_point': 0},
        'H2S': {'address': 6, 'decimal_point': 0},
        'NH3': {'address': 7, 'decimal_point': 0},
        'CO': {'address': 4, 'decimal_point': 0}
    }

    sensors = {}

    # 初始化传感器
    for gas, config in SENSOR_CONFIG.items():
        sensor = GasSensor(
            gas_name=gas,
            port=VIRTUAL_PORT_PAIR,
            baud=BAUD,
            slave_address=config['address'],
            decimal_point=config['decimal_point']
        )
        if sensor.instrument:
            sensors[gas] = sensor
            logging.info(f'{gas} 传感器设置成功')

    logging.info(f"开始预热阶段，等待 {PREHEAT_TIME} 秒")
    time.sleep(PREHEAT_TIME)
    logging.info("预热完成，开始传感器轮询")

    # 轮询读取传感器数据
    while True:
        current_time = time.time()
        for gas, sensor in sensors.items():
            if current_time >= sensor.next_read_time:
                concentration = sensor.execute_command("read_concentration")
                if concentration is not None:
                    print(f"{gas} 浓度: {concentration}")
                    sensor.fail_count = 0  # 成功时重置失败计数
                    sensor.read_interval = 5  # 重置为默认间隔
                else:
                    sensor.fail_count += 1
                    sensor.read_interval = min(60, sensor.read_interval * 2)  # 间隔加倍，最多60秒

                # 计划下一次读取时间
                sensor.next_read_time = current_time + sensor.read_interval

        time.sleep(1)  # 短暂休眠，避免循环过紧

if __name__ == '__main__':
    main()

