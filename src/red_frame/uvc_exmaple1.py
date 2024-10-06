import usb.core
import usb.util
import struct
import uvc
import logging
from rich import print
from rich.logging import RichHandler
import sys
from datetime import datetime
import numpy as np
from PIL import Image
import io
import csv
import os


# 设置日志配置
logging.basicConfig(
    level=logging.DEBUG,  # 设置默认的日志级别为 DEBUG
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
    handlers=[RichHandler(rich_tracebacks=True)]
)

# 创建一个 logger 对象
logger = logging.getLogger(__name__)


# UVC Protocol Constants
SET_REQUEST_TYPE = 0x21
GET_REQUEST_TYPE = 0xA1

# bRequest values
SET_CUR = 0x01
GET_CUR = 0x81
GET_MIN = 0x82
GET_MAX = 0x83
GET_RES = 0x84
GET_LEN = 0x85
GET_INFO = 0x86
GET_DEF = 0x87

# CS_ID (Control Selector ID) values
XU_CS_ID_SYSTEM = 0x01
XU_CS_ID_IMAGE = 0x02
XU_CS_ID_THERMAL = 0x03
XU_CS_ID_PROTOCOL_VER = 0x04
XU_CS_ID_COMMAND_SWITCH = 0x05
XU_CS_ID_ERROR_CODE = 0x06

# Sub-function IDs for XU_CS_ID_SYSTEM (CS ID=0x01)
SYSTEM_DEVICE_INFO = 0x01
SYSTEM_REBOOT = 0x02
SYSTEM_RESET = 0x03
SYSTEM_HARDWARE_SERVER = 0x04
SYSTEM_LOCALTIME = 0x05
SYSTEM_UPDATE_FIRMWARE = 0x06
SYSTEM_DIAGNOSED_DATA = 0x07

# Sub-function IDs for XU_CS_ID_IMAGE (CS ID=0x02)
IMAGE_BRIGHTNESS = 0x01
IMAGE_CONTRAST = 0x02
IMAGE_BACKGROUND_CORRECT = 0x03
IMAGE_MANUAL_CORRECT = 0x04
IMAGE_ENHANCEMENT = 0x05
IMAGE_VIDEO_ADJUST = 0x06

# Sub-function IDs for XU_CS_ID_THERMAL (CS ID=0x03)
THERMAL_THERMOMETRY_BASIC_PARAM = 0x01
THERMAL_THERMOMETRY_MODE = 0x02
THERMAL_THERMOMETRY_REGIONS = 0x03
THERMAL_ALG_VERSION = 0x04
THERMAL_STREAM_PARAM = 0x05
THERMAL_TEMPERATURE_CORRECT = 0x06
THERMAL_BLACK_BODY = 0x07
THERMAL_BODYTEMP_COMPENSATION = 0x08
THERMAL_JPEGPIC_WITH_APPENDDATA = 0x09
THERMAL_ROI_MAX_TEMPERATURE_SEARCH = 0x0A
THERMAL_P2P_PARAM = 0x0B
THERMAL_THERMOMETRY_CALIBRATION_FILE = 0x0E
THERMAL_THERMOMETRY_EXPERT_REGIONS = 0x0F
THERMAL_THERMOMETRY_EXPERT_CORRECTION_PARAM = 0x10
THERMAL_THERMOMETRY_EXPERT_CORRECTION_START = 0x11

# Error Codes
ERROR_CODES = {
    0x00: "Normal",
    0x01: "Device not completed previous request",
    0x02: "Device in invalid state for request",
    0x03: "Insufficient power mode",
    0x04: "SET_CUR parameter out of range",
    0x05: "Unsupported Unit ID",
    0x06: "Unsupported CS ID",
    0x07: "Unsupported request type",
    0x08: "SET_CUR parameter invalid",
    0x09: "Unsupported sub-function",
    0x0A: "Device function execution error",
    0x0B: "Device internal protocol process error",
    0x0C: "Large data transfer process error",
    0x0D: "SET_CUR request wLength field abnormal",
    0xFF: "Unknown"
}

# Mapping for b_request values
B_REQUEST_NAMES = {
    SET_CUR: "SET_CUR",
    GET_CUR: "GET_CUR",
    GET_MIN: "GET_MIN",
    GET_MAX: "GET_MAX",
    GET_RES: "GET_RES",
    GET_LEN: "GET_LEN",
    GET_INFO: "GET_INFO",
    GET_DEF: "GET_DEF"
}

# Mapping for CS_ID values
CS_ID_NAMES = {
    XU_CS_ID_SYSTEM: "XU_CS_ID_SYSTEM",
    XU_CS_ID_IMAGE: "XU_CS_ID_IMAGE",
    XU_CS_ID_THERMAL: "XU_CS_ID_THERMAL",
    XU_CS_ID_PROTOCOL_VER: "XU_CS_ID_PROTOCOL_VER",
    XU_CS_ID_COMMAND_SWITCH: "XU_CS_ID_COMMAND_SWITCH",
    XU_CS_ID_ERROR_CODE: "XU_CS_ID_ERROR_CODE"
}

# Mapping for Sub-function IDs
SUB_FUNCTION_NAMES = {
    XU_CS_ID_SYSTEM: {
        0x01: "SYSTEM_DEVICE_INFO",
        0x02: "SYSTEM_REBOOT",
        0x03: "SYSTEM_RESET",
        0x04: "SYSTEM_HARDWARE_SERVER",
        0x05: "SYSTEM_LOCALTIME",
        0x06: "SYSTEM_UPDATE_FIRMWARE",
        0x07: "SYSTEM_DIAGNOSED_DATA"
    },
    XU_CS_ID_IMAGE: {
        0x01: "IMAGE_BRIGHTNESS",
        0x02: "IMAGE_CONTRAST",
        0x03: "IMAGE_BACKGROUND_CORRECT",
        0x04: "IMAGE_MANUAL_CORRECT",
        0x05: "IMAGE_ENHANCEMENT",
        0x06: "IMAGE_VIDEO_ADJUST"
    },
    XU_CS_ID_THERMAL: {
        0x01: "THERMAL_THERMOMETRY_BASIC_PARAM",
        0x02: "THERMAL_THERMOMETRY_MODE",
        0x03: "THERMAL_THERMOMETRY_REGIONS",
        0x04: "THERMAL_ALG_VERSION",
        0x05: "THERMAL_STREAM_PARAM",
        0x06: "THERMAL_TEMPERATURE_CORRECT",
        0x07: "THERMAL_BLACK_BODY",
        0x08: "THERMAL_BODYTEMP_COMPENSATION",
        0x09: "THERMAL_JPEGPIC_WITH_APPENDDATA",
        0x0A: "THERMAL_ROI_MAX_TEMPERATURE_SEARCH",
        0x0B: "THERMAL_P2P_PARAM",
        0x0E: "THERMAL_THERMOMETRY_CALIBRATION_FILE",
        0x0F: "THERMAL_THERMOMETRY_EXPERT_REGIONS",
        0x10: "THERMAL_THERMOMETRY_EXPERT_CORRECTION_PARAM",
        0x11: "THERMAL_THERMOMETRY_EXPERT_CORRECTION_START"
    }
}

class USBCommunication:
    def __init__(self, device):
        self.device = device

    def send_request(self, request_type, b_request, cs_id, sub_function_id=0, data=None, length=1):
        w_value = (cs_id << 8) | sub_function_id
        w_index = (0x0A << 8) | 0x00  # XU_UNIT_ID is 0x0A

        request_type_name = "SET_REQUEST_TYPE" if request_type == SET_REQUEST_TYPE else "GET_REQUEST_TYPE"
        b_request_name = B_REQUEST_NAMES.get(b_request, "Unknown b_request")
        cs_id_name = CS_ID_NAMES.get(cs_id, "Unknown CS_ID")
        sub_function_name = SUB_FUNCTION_NAMES.get(cs_id, {}).get(sub_function_id, "Unknown Sub-function")
        
        logger.debug(f"Sending command: request_type={hex(request_type)} ({request_type_name}), "
                     f"b_request={hex(b_request)} ({b_request_name}), "
                     f"CS_ID={hex(cs_id)} ({cs_id_name}), "
                     f"Sub-function={hex(sub_function_id)} ({sub_function_name})")

        return self.device.ctrl_transfer(
            request_type,
            b_request,
            w_value,
            w_index,
            data if data else length
        )

class USBProtocol:
    def __init__(self, communication):
        self.communication = communication
        self.current_cs_id = None
        self.current_sub_function_id = None

    def get_length(self, cs_id, sub_function_id):
        """Gets the length of data for a specific request."""
        try:
            length_data = self.communication.send_request(
                GET_REQUEST_TYPE,
                GET_LEN,
                cs_id,
                sub_function_id,
                length=2  # GET_LEN always expects 2 bytes
            )
            return length_data[0] | (length_data[1] << 8)
        except Exception as e:
            print(f"Failed to get length for CS_ID={hex(cs_id)}, Sub-function={hex(sub_function_id)}: {e}")
            return 0

    def switch_functionality(self, cs_id, sub_function_id):
        """Switches the device functionality if needed."""
        if cs_id != self.current_cs_id or sub_function_id != self.current_sub_function_id:
            try:
                data = struct.pack('BB', cs_id, sub_function_id)
                self.communication.send_request(
                    SET_REQUEST_TYPE, 
                    SET_CUR, 
                    XU_CS_ID_COMMAND_SWITCH, 
                    0,
                    data=data
                )
                self.current_cs_id = cs_id
                self.current_sub_function_id = sub_function_id
                print(f"Switched to CS_ID={hex(cs_id)}, Sub-function={hex(sub_function_id)}")
            except Exception as e:
                print(f"Failed to switch functionality: {e}")

    def get_protocol_version(self):
        """Gets the protocol version from the device."""
        try:
            version_data = self.send_request(
                GET_REQUEST_TYPE,
                GET_CUR,
                XU_CS_ID_PROTOCOL_VER,
                0
            )
            # print(f"Raw version data: {version_data}")
            
            if version_data:
                # Convert array of integers to bytes
                version_bytes = bytes(version_data)
                # Decode bytes to string, stopping at null terminator
                version_str = version_bytes.decode('ascii').rstrip('\x00')
                # print(f"Decoded version string: {version_str}")
                return version_str
            else:
                print("No version data received")
                return None
        except Exception as e:
            print(f"Failed to get protocol version: {e}")
            return None

    def send_request(self, request_type, b_request, cs_id, sub_function_id, data=None):
        """Sends a request with proper functionality switching and length getting."""
        self.switch_functionality(cs_id, sub_function_id)

        if b_request == GET_LEN:
            length_data = self.communication.send_request(
                request_type,
                b_request,
                cs_id,
                sub_function_id,
                length=2
            )
            length = length_data[0] | (length_data[1] << 8)
            print(f"length: {length}")
            return length

        length = self.get_length(cs_id, sub_function_id)
        print("length:",length)

        try:
            if b_request in [GET_CUR, GET_MIN, GET_MAX, GET_RES, GET_DEF, GET_INFO]:
                return self.communication.send_request(
                    request_type,
                    b_request,
                    cs_id,
                    sub_function_id,
                    length=length
                )
            elif b_request == SET_CUR:
                if data:
                    if len(data) != length:
                        raise ValueError(f"Data length ({len(data)}) does not match expected length ({length})")
                    return self.communication.send_request(
                        request_type,
                        b_request,
                        cs_id,
                        sub_function_id,
                        data=data
                    )
                else:
                    return self.communication.send_request(
                        request_type,
                        b_request,
                        cs_id,
                        sub_function_id,
                        length=length
                    )
        except Exception as e:
            print(f"Error in send_request: {e}")
            return None
  
    # System Management Functions
    def get_device_info(self):
        """Gets the device information."""
        try:
            info_data = self.send_request(
                GET_REQUEST_TYPE,
                GET_CUR,
                XU_CS_ID_SYSTEM,
                SYSTEM_DEVICE_INFO
            )
            logging.debug(f"Raw device info data: {info_data}")
            
            if not info_data:
                logging.warning("No device info data received")
                return None

            # Convert array of integers to bytes and decode to ASCII
            info_str = bytes(info_data).decode('ascii', errors='ignore').rstrip('\x00')
            logging.debug(f"Decoded device info string: {info_str}")
            
            # Split the info string into components and remove empty elements
            components = [comp for comp in info_str.split('\x00') if comp]
            logging.debug(f"Split components: {components}")
            
            # Define expected fields
            fields = ["firmwareVersion", "encoderVersion", "hardwareVersion", 
                    "deviceName", "protocolVersion", "serialNumber"]
            
            # Create dictionary with available data
            device_info = {field: components[i] if i < len(components) else ""
                        for i, field in enumerate(fields)}
            
            return device_info

        except Exception as e:
            logging.error(f"Failed to get device info: {e}", exc_info=True)
            return None

    def reboot_device(self):
        """Reboots the device."""
        self.send_request(
            SET_REQUEST_TYPE,
            SET_CUR,
            XU_CS_ID_SYSTEM,
            SYSTEM_REBOOT
        )

    def reset_to_default(self):
        """Resets the device to default settings."""
        self.send_request(
            SET_REQUEST_TYPE,
            SET_CUR,
            XU_CS_ID_SYSTEM,
            SYSTEM_RESET
        )

    def set_hardware_server(self, usb_mode):
        """Sets the hardware server parameters."""
        data = struct.pack('B', usb_mode)
        self.send_request(
            SET_REQUEST_TYPE,
            SET_CUR,
            XU_CS_ID_SYSTEM,
            SYSTEM_HARDWARE_SERVER,
            data
        )

    def set_system_time(self, dt=None):
        """Sets the system time. If no datetime is provided, uses the current time."""
        if dt is None:
            dt = datetime.now()

        # millisecond, second, minute, hour, day, month, year, external_time_source = struct.unpack('<HBBBBBHB', time_data)


        time_data = struct.pack('<HBBBBBHB',
            dt.microsecond // 1000,  # milliseconds
            dt.second,
            dt.minute,
            dt.hour,
            dt.day,
            dt.month,
            dt.year,
            0  # externalTimeSourceEnabled, assuming it's disabled
        )

        try:
            self.send_request(
                SET_REQUEST_TYPE,
                SET_CUR,
                XU_CS_ID_SYSTEM,
                SYSTEM_LOCALTIME,
                time_data
            )
            logging.info("Time set successfully")
            return True
        except Exception as e:
            logging.error(f"Failed to set system time: {e}", exc_info=True)
            return False

    def get_system_time(self):
        """Gets the current system time and returns it in a human-readable format."""
        try:
            time_data = self.send_request(
                GET_REQUEST_TYPE,
                GET_CUR,
                XU_CS_ID_SYSTEM,
                SYSTEM_LOCALTIME
            )

            if not time_data:
                logging.warning("No time data received")
                return None

            logging.debug(f"Raw time data: {time_data}")

            # Unpack the binary data according to the correct format
            millisecond, second, minute, hour, day, month, year, external_time_source = struct.unpack('<HBBBBBHB', time_data)

            # Create datetime object
            dt = datetime(year, month, day, hour, minute, second, millisecond * 1000)

            # Format the datetime as a string
            formatted_time = dt.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

            return formatted_time
            return {
                "raw_data": time_data.hex(),
                "formatted_time": formatted_time,
                "datetime_obj": dt,
                "components": {
                    "year": year,
                    "month": month,
                    "day": day,
                    "hour": hour,
                    "minute": minute,
                    "second": second,
                    "millisecond": millisecond,
                    "external_time_source": external_time_source
                }
            }
        except struct.error as e:
            logging.error(f"Failed to unpack time data: {e}", exc_info=True)
            return None
        except Exception as e:
            logging.error(f"Failed to get system time: {e}", exc_info=True)
            return None

    def get_firmware_update_status(self):
        """Gets the firmware update status."""
        return self.send_request(
            GET_REQUEST_TYPE,
            GET_CUR,
            XU_CS_ID_SYSTEM,
            SYSTEM_UPDATE_FIRMWARE
        )

    def get_diagnostic_data(self):
        """Gets the diagnostic data."""
        return self.send_request(
            GET_REQUEST_TYPE,
            GET_CUR,
            XU_CS_ID_SYSTEM,
            SYSTEM_DIAGNOSED_DATA
        )

    # Image Management Functions
    def set_image_brightness(self, channel_id, brightness):
        """Sets the image brightness."""
        data = struct.pack('BB', channel_id, brightness)
        self.send_request(
            SET_REQUEST_TYPE,
            SET_CUR,
            XU_CS_ID_IMAGE,
            IMAGE_BRIGHTNESS,
            data
        )

    def set_image_contrast(self, channel_id, contrast):
        """Sets the image contrast."""
        data = struct.pack('BB', channel_id, contrast)
        self.send_request(
            SET_REQUEST_TYPE,
            SET_CUR,
            XU_CS_ID_IMAGE,
            IMAGE_CONTRAST,
            data
        )

    def perform_background_correction(self, channel_id):
        """Performs background correction."""
        data = struct.pack('B', channel_id)
        self.send_request(
            SET_REQUEST_TYPE,
            SET_CUR,
            XU_CS_ID_IMAGE,
            IMAGE_BACKGROUND_CORRECT,
            data
        )

    def perform_manual_correction(self, channel_id):
        """Performs manual correction."""
        data = struct.pack('B', channel_id)
        self.send_request(
            SET_REQUEST_TYPE,
            SET_CUR,
            XU_CS_ID_IMAGE,
            IMAGE_MANUAL_CORRECT,
            data
        )

    def set_image_enhancement(self, channel_id, params):
        """Sets image enhancement parameters."""
        data = struct.pack('BBBBBBBB', channel_id, *params)
        self.send_request(
            SET_REQUEST_TYPE,
            SET_CUR,
            XU_CS_ID_IMAGE,
            IMAGE_ENHANCEMENT,
            data
        )

    def set_video_adjustment(self, channel_id, params):
        """Sets video adjustment parameters."""
        data = struct.pack('BBBB', channel_id, *params)
        self.send_request(
            SET_REQUEST_TYPE,
            SET_CUR,
            XU_CS_ID_IMAGE,
            IMAGE_VIDEO_ADJUST,
            data
        )

    # Thermal Management Functions
    def set_thermometry_basic_params(self, params):
        """Sets basic thermometry parameters."""
        data = struct.pack('BBBBBBBBfffffBfBff', *params)
        self.send_request(
            SET_REQUEST_TYPE,
            SET_CUR,
            XU_CS_ID_THERMAL,
            THERMAL_THERMOMETRY_BASIC_PARAM,
            data
        )

    def set_thermometry_mode(self, channel_id, mode, roi_enabled):
        """Sets thermometry mode."""
        data = struct.pack('BBB', channel_id, mode, roi_enabled)
        self.send_request(
            SET_REQUEST_TYPE,
            SET_CUR,
            XU_CS_ID_THERMAL,
            THERMAL_THERMOMETRY_MODE,
            data
        )

    # Large Data Transfer Method
    def large_data_transfer(self, cs_id, sub_function_id, data):
        """Handles large data transfer."""
        total_length = len(data)
        header = struct.pack('<BI', 0x01, total_length)  # Data type 0x01, followed by total length
        self.send_request(SET_REQUEST_TYPE, SET_CUR, cs_id, sub_function_id, header)

        packet_size = 1024  # Assuming 1KB packets
        for i in range(0, total_length, packet_size):
            packet = data[i:i+packet_size]
            packet_header = struct.pack('<BI', 0x02, i//packet_size + 1)  # Data type 0x02, followed by packet number
            self.send_request(SET_REQUEST_TYPE, SET_CUR, cs_id, sub_function_id, packet_header + packet)
            
            

    
    def parse_received_bytes(self,data):
        # 确保数据长度足够
        if len(data) < 5:
            raise ValueError("数据长度不足，无法解析")

        # 解析数据
        data_type = data[0]  # 数据类型, 1字节
        total_length = struct.unpack('<I', data[1:])[0]  # 数据总长, 4字节, 小端字节序

        # 返回解析结果
        return total_length
 
    
    def get_thermal_image_with_data(self, channel_id=0):
        """Gets the thermal image with appended temperature data using large data transfer."""
        try:
            # Step 1: Get the total data length
            length_data = self.send_request(
                GET_REQUEST_TYPE,
                GET_CUR,
                XU_CS_ID_THERMAL,
                THERMAL_JPEGPIC_WITH_APPENDDATA
            )
            print(f"Received length data (raw bytes): {length_data}")
            
            total_length = self.parse_received_bytes(length_data)
            print(f"Total data length: {total_length}")
            
            # Step 2: Prepare to receive data
            received_data = bytearray()
            while len(received_data) < total_length:
                # Request the next chunk of data
                chunk = self.send_request(
                    GET_REQUEST_TYPE,
                    GET_CUR,
                    XU_CS_ID_THERMAL,
                    THERMAL_JPEGPIC_WITH_APPENDDATA
                )
                
                # Parse packet header
                data_type, packet_number = struct.unpack('<BI', chunk[:5])
                print(f"Received packet: Type={data_type}, Number={packet_number}")
                
                # Extract and append actual data
                received_data.extend(chunk[5:])
                print(f"Total received: {len(received_data)} bytes")

            # Parse the received data
            header_size = struct.calcsize('<BIIIIBBII')
            print("header_size",header_size)
            if len(received_data) < header_size:
                raise ValueError(f"Received data is too short. Expected at least {header_size} bytes, got {len(received_data)}")

            header = struct.unpack('<BIIIIBBII', received_data[:header_size])
            
            channel_id, jpeg_len, jpeg_width, jpeg_height, p2p_data_len, is_freeze_data, p2p_data_type, scale, offset = header

            print(f"Parsed header: channel_id={channel_id}, jpeg_len={jpeg_len}, "
                f"jpeg_width={jpeg_width}, jpeg_height={jpeg_height}, "
                f"p2p_data_len={p2p_data_len}, is_freeze_data={is_freeze_data}, "
                f"p2p_data_type={p2p_data_type}, scale={scale}, offset={offset}")

            # Extract JPEG image
            jpeg_data = received_data[header_size:header_size+jpeg_len]
            print(f"JPEG data length: {len(jpeg_data)}")
            image = Image.open(io.BytesIO(jpeg_data))

            # Extract temperature data
            temp_data_start = header_size + jpeg_len
            temp_data = received_data[temp_data_start:temp_data_start+p2p_data_len]
            print(f"Temperature data length: {len(temp_data)}")

            # Convert temperature data to numpy array
            if p2p_data_type == 2:  # short integer
                temp_array = np.frombuffer(temp_data, dtype=np.uint16).reshape(jpeg_height, jpeg_width)
                temp_array = temp_array / scale + offset - 273.15
            elif p2p_data_type == 4:  # float
                temp_array = np.frombuffer(temp_data, dtype=np.float32).reshape(jpeg_height, jpeg_width)

            print(f"Temperature array shape: {temp_array.shape}")
            print(f"Temperature range: min={np.min(temp_array):.2f}, max={np.max(temp_array):.2f}, mean={np.mean(temp_array):.2f}")

            return {
                'image': image,
                'temperature_data': temp_array,
                'is_freeze_data': bool(is_freeze_data)
            }

        except Exception as e:
            print(f"Error in get_thermal_image_with_data: {e}")
            return None

    import os
    import numpy as np
    from datetime import datetime

    def save_thermal_data(self, thermal_data, base_filename):
        """Saves the thermal image and temperature data with timestamp."""
        if thermal_data is None:
            print("No thermal data to save.")
            return

        # Create a timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        # Create a directory for the output if it doesn't exist
        output_dir = os.path.join('thermal_output', timestamp)
        os.makedirs(output_dir, exist_ok=True)

        # Combine base filename with timestamp
        filename = f"{base_filename}_{timestamp}"

        # Save the image if it exists
        if thermal_data['image'] is not None:
            image_filename = os.path.join(output_dir, f"{filename}_thermal.jpg")
            thermal_data['image'].save(image_filename)
            print(f"Thermal image saved as {image_filename}")
        else:
            print("No thermal image to save.")

        # Save the temperature data as CSV with 2 decimal places
        csv_filename = os.path.join(output_dir, f"{filename}_temperature.csv")
        np.savetxt(csv_filename, thermal_data['temperature_data'], delimiter=',', fmt='%.2f')
        print(f"Temperature data saved as {csv_filename}")

        # Save metadata
        metadata_filename = os.path.join(output_dir, f"{filename}_metadata.txt")
        with open(metadata_filename, 'w') as metafile:
            metafile.write(f"Timestamp: {timestamp}\n")
            if thermal_data['image'] is not None:
                metafile.write(f"Image size: {thermal_data['image'].size}\n")
            else:
                metafile.write("Image size: No image data\n")
            metafile.write(f"Temperature array shape: {thermal_data['temperature_data'].shape}\n")
            metafile.write(f"Is freeze frame: {thermal_data['is_freeze_data']}\n")
            metafile.write(f"Min temperature: {np.min(thermal_data['temperature_data']):.2f}°C\n")
            metafile.write(f"Max temperature: {np.max(thermal_data['temperature_data']):.2f}°C\n")
            metafile.write(f"Average temperature: {np.mean(thermal_data['temperature_data']):.2f}°C\n")
        print(f"Metadata saved as {metadata_filename}")

        print(f"All files saved in directory: {output_dir}")
            
class ErrorCodeHandler:
    def __init__(self, communication):
        self.communication = communication

    def get_error_code(self):
        try:
            response = self.communication.send_request(
                GET_REQUEST_TYPE,
                GET_CUR,
                XU_CS_ID_ERROR_CODE
            )
            error_code = response[0]
            logger.info(f"Device Error Code: {ERROR_CODES.get(error_code, 'Unknown Error Code')}")
            return error_code
        except Exception as e:
            logger.error(f"Failed to retrieve error code: {e}", exc_info=True)
            return None


class UVCDeviceManager:
    """Manages the UVC device initialization and control."""

    def find_device(self):
        devices = uvc.device_list()
        print("Available devices:", devices)

        if not devices:
            print("No UVC devices found.")
            return None

        device_info = devices[0]
        print(f"Using device: {device_info['name']}")

        vendor_id = device_info['idVendor']
        product_id = device_info['idProduct']

        dev = usb.core.find(idVendor=vendor_id, idProduct=product_id)

        if dev is None:
            raise ValueError("UVC Device not found via pyusb")

        for i in range(dev.get_active_configuration().bNumInterfaces):
            if dev.is_kernel_driver_active(i):
                try:
                    dev.detach_kernel_driver(i)
                except usb.core.USBError as e:
                    sys.exit(f"Could not detach kernel driver from interface({i}): {str(e)}")

        return dev


class Main:
    def __init__(self):
        self.device_manager = UVCDeviceManager()
        self.communication = None
        self.protocol = None
        self.error_handler = None

    def initialize_components(self):
        dev = self.device_manager.find_device()
        if not dev:
            print("Failed to find and initialize UVC device.")
            return False

        self.communication = USBCommunication(dev)
        self.protocol = USBProtocol(self.communication)
        self.error_handler = ErrorCodeHandler(self.communication)
        print("UVC device connected and initialized")
        return True

    def run(self):
        if not self.initialize_components():
            return

        try:
            # Get protocol version
            version = self.protocol.get_protocol_version()
            if version:
                print(f"Protocol Version: {version}")
            else:
                print("Failed to get protocol version")
                return

            # Get device info
            device_info = self.protocol.get_device_info()
            if device_info:
                print("Device Info:")
                for key, value in device_info.items():
                    print(f"  {key}: {value}")
            else:
                print("Failed to get device info")


            # self.protocol.set_system_time()
            
            # self.error_handler.get_error_code()
            
            # print("Time set successfully")

            # Example: Get current time
            current_time = self.protocol.get_system_time()
            print("Current time:", current_time)

            # # Example: Set image brightness
            # self.protocol.set_image_brightness(1, 75)  # Channel 1, brightness 75
            # print("Image brightness set")

            # # Example: Perform background correction
            # self.protocol.perform_background_correction(1)  # Channel 1
            # print("Background correction performed")

            # Example: Get thermal image with temperature data
            thermal_data = self.protocol.get_thermal_image_with_data()
            if thermal_data:
                # Save the thermal data
                self.protocol.save_thermal_data(thermal_data, "thermal_capture")
            else:
                print("Failed to get thermal image and data")

            # Check error code
            self.error_handler.get_error_code()

        except Exception as e:
            print(f"An error occurred during operation: {e}")
            self.error_handler.get_error_code()

if __name__ == "__main__":
    # 设置日志级别
    log_level = logging.INFO  # 可以根据需要更改为 DEBUG, WARNING, ERROR, 或 CRITICAL
    logging.getLogger().setLevel(log_level)
    
    main = Main()
    main.run()