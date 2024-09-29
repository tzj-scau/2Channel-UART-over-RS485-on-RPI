import minimalmodbus
import serial
import logging
import time
from spi_serial_sys_dev import VIRTUAL_PORT_PAIR, BAUD

# Sensor addresses
SENSOR_ADDRESSES = {
    'CO2': 1,
    'H2S': 2,
    'NH3': 3,
    'CO': 4
}

# Decimal points for each gas type
DECIMAL_POINTS = {
    'CO2': 1,
    'H2S': 0,
    'NH3': 0,
    'CO': 0
}

# Constants from the technical documentation
CONCENTRATION_REGISTER = 0x6001
ALARM_STATUS_REGISTER = 0x6000
RUNNING_STATUS_REGISTER = 0x6006

# User-configurable preheating time (in seconds)
PREHEAT_TIME = 5
# Threshold for zero point masking (PPM)
ZERO_POINT_THRESHOLD = 3
# Minimum time interval between each command (in seconds)
MIN_TIME_INTERVAL = 0.5

# Running status definitions
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

# Alarm status definitions for when bit15 = 1
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

class SensorState:
    def __init__(self):
        self.last_command = None
        self.running_status = None
        self.alarm_status = None
        self.next_read_time = time.time()  # Time when the sensor should next be read
        self.read_interval = 5  # Default read interval in seconds
        self.fail_count = 0  # Count consecutive failures

def setup_instrument(port, baud, slave_address):
    try:
        ser = serial.Serial(port, baud, timeout=0.5)
        instrument = minimalmodbus.Instrument(ser, slaveaddress=slave_address)
        instrument.mode = minimalmodbus.MODE_RTU
        instrument.serial.timeout = 0.5
        instrument.debug = True  # Enable debug mode for detailed logging
        return instrument
    except serial.SerialException as e:
        logging.error(f"无法打开传感器 {slave_address} 的串口: {e}")
        return None

def check_running_status(instrument, sensor_state, gas_name):
    try:
        time.sleep(MIN_TIME_INTERVAL)  # Ensure minimum time interval between commands
        status = instrument.read_register(RUNNING_STATUS_REGISTER, functioncode=3)
        sensor_state.running_status = status
        status_desc = RUNNING_STATUS.get(status, "未知状态")
        logging.debug(f"{gas_name} 运行状态: {status_desc}")
        return status
    except Exception as e:
        logging.error(f"{gas_name} 读取运行状态失败: {e}")
        return None

def check_alarm_status(instrument, sensor_state, gas_name):
    try:
        time.sleep(MIN_TIME_INTERVAL)  # Ensure minimum time interval between commands
        status = instrument.read_register(ALARM_STATUS_REGISTER, functioncode=3)
        sensor_state.alarm_status = status
        
        if status & 0x8000:  # bit15 = 1, preheating complete
            alarms = []
            for bit, description in ALARM_STATUS_BIT15_1.items():
                if status & bit:
                    alarms.append(description)
            if not alarms:
                alarms.append("无报警")
            logging.debug(f"{gas_name} 报警状态: 预热完成, {', '.join(alarms)}")
        else:  # bit15 = 0, preheating
            preheating_time = status & 0x7FFF
            logging.debug(f"{gas_name} 预热中，剩余时间: {preheating_time * 0.1:.1f} 秒")
        
        return status
    except Exception as e:
        logging.error(f"{gas_name} 读取报警状态失败: {e}")
        return None

def read_gas_concentration(instrument, gas_name):
    try:
        time.sleep(MIN_TIME_INTERVAL)  # Ensure minimum time interval between commands
        concentration = instrument.read_register(CONCENTRATION_REGISTER, functioncode=3)
        decimal_point = DECIMAL_POINTS[gas_name]
        actual_concentration = concentration / (10 ** decimal_point)

        # Masking low values to zero if below threshold
        if abs(actual_concentration) <= ZERO_POINT_THRESHOLD:
            actual_concentration = 0
        return actual_concentration
    except Exception as e:
        logging.error(f"{gas_name} 读取气体浓度失败: {e}")
        return None

def execute_command(instrument, command, sensor_state, gas_name):
    sensor_state.last_command = command

    result = None
    try:
        if command == "read_concentration":
            # Directly execute the command
            result = read_gas_concentration(instrument, gas_name)
            if result is not None:
                logging.info(f"{gas_name} 执行成功: {sensor_state.last_command}")
            else:
                # If execution fails, check running status
                status = check_running_status(instrument, sensor_state, gas_name)
                # If status is normal, assume zero point masking
                if status == 0x0001:  # 模组空闲或执行成功
                    result = 0
                    logging.info(f"{gas_name} 被零点屏蔽，气体浓度设为 0")
    except Exception as e:
        logging.error(f"{gas_name} 执行命令失败: {command}, 错误: {e}")
        status = check_running_status(instrument, sensor_state, gas_name)
        if status == 0x0001:
            result = 0
            logging.info(f"{gas_name} 被零点屏蔽，气体浓度设为 0")  # Assume zero point masking

    return result

def main():
    logging.basicConfig(level=logging.DEBUG)
    
    instruments = {}
    sensor_states = {}
    
    for gas, address in SENSOR_ADDRESSES.items():
        instrument = setup_instrument(VIRTUAL_PORT_PAIR, BAUD, address)
        if instrument:
            instruments[gas] = instrument
            sensor_states[gas] = SensorState()
            logging.info(f'{gas} 传感器设置成功')
    
    logging.info(f"开始预热阶段，等待 {PREHEAT_TIME} 秒")
    time.sleep(PREHEAT_TIME)
    logging.info("预热完成，开始传感器轮询")

    while True:
        current_time = time.time()
        for gas, instrument in instruments.items():
            sensor_state = sensor_states[gas]

            if current_time >= sensor_state.next_read_time:  # Check if it's time to read this sensor
                # Directly execute the command without pre-checking status
                concentration = execute_command(instrument, "read_concentration", sensor_state, gas)
                if concentration is not None:
                    print(f"{gas} 浓度: {concentration}")
                    sensor_state.fail_count = 0  # Reset failure count on success
                    sensor_state.read_interval = 5  # Reset to default interval
                else:
                    sensor_state.fail_count += 1
                    sensor_state.read_interval = min(60, sensor_state.read_interval * 2)  # Double interval up to 60 seconds

                # Schedule the next read time
                sensor_state.next_read_time = current_time + sensor_state.read_interval

        time.sleep(1)  # Sleep for a short time to avoid tight looping

if __name__ == '__main__':
    main()
