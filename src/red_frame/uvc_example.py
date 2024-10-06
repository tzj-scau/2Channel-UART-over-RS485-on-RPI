import usb.core
import usb.util
import struct
import uvc
import logging
from rich import print
from rich.logging import RichHandler
import sys

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
        
        print(f"Sending command: request_type={hex(request_type)} ({request_type_name}), "
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

    def get_protocol_version(self):
        """Gets the protocol version from the device."""
        try:
            version_data = self.send_request(
                GET_REQUEST_TYPE,
                GET_CUR,
                XU_CS_ID_PROTOCOL_VER,
                0,
                length=2
            )
            major = version_data[0]
            minor = version_data[1]
            return f"{major}.{minor}"
        except Exception as e:
            print(f"Failed to get protocol version: {e}")
            return None

    def send_request(self, request_type, b_request, cs_id, sub_function_id, data=None, length=1):
        """Sends a request with proper functionality switching and length getting."""
        self.switch_functionality(cs_id, sub_function_id)

        try:
            return self.communication.send_request(
                request_type,
                b_request,
                cs_id,
                sub_function_id,
                data=data,
                length=length
            )
        except Exception as e:
            print(f"Error in send_request: {e}")
            return None

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
            except Exception as e:
                print(f"Failed to switch functionality: {e}")

    def get_device_info(self):
        """Gets the device information."""
        try:
            info_data = self.send_request(
                GET_REQUEST_TYPE,
                GET_CUR,
                XU_CS_ID_SYSTEM,
                SUB_FUNCTION_NAMES[XU_CS_ID_SYSTEM]["SYSTEM_DEVICE_INFO"],
                length=324
            )
            if info_data:
                return {
                    'firmwareVersion': info_data[:64].decode('ascii').rstrip('\x00'),
                    'encoderVersion': info_data[64:128].decode('ascii').rstrip('\x00'),
                    'hardwareVersion': info_data[128:192].decode('ascii').rstrip('\x00'),
                    'deviceName': info_data[192:256].decode('ascii').rstrip('\x00'),
                    'protocolVersion': info_data[256:260].decode('ascii').rstrip('\x00'),
                    'serialNumber': info_data[260:324].decode('ascii').rstrip('\x00')
                }
            else:
                return None
        except Exception as e:
            print(f"Failed to get device info: {e}")
            return None

    # ... (其他方法保持不变)

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

            # ... (其他操作)

        except Exception as e:
            print(f"An error occurred during operation: {e}")
            self.error_handler.get_error_code()

if __name__ == "__main__":
    logging.basicConfig(
        level=logging.NOTSET,
        handlers=[RichHandler(level="WARNING")],
        format="%(message)s",
        datefmt="[%X]",
    )
    main = Main()
    main.run()