import usb.core
import usb.util
from typing import List, Union
import uvc

# cap = uvc.Capture(dev['uid'])

class USBXUControl:
    # Constants
    BM_REQUEST_TYPE_SET = 0x21
    BM_REQUEST_TYPE_GET = 0xA1

    # bRequest constants
    SET_CUR = 0x01
    GET_CUR = 0x81
    GET_LEN = 0x85

    # XU_CS_IDs (wValue constants)
    XU_CS_ID_PROTOCOL_VER = 0x0400
    XU_CS_ID_COMMAND_SWITCH = 0x0500
    XU_CS_ID_ERROR_CODE = 0x0600

    def __init__(self, vendor_id: int = 0x2BDF, product_id: int = 0x0101, 
                 extension_unit_id: int = 0x0A, interface_id: int = 0x00):
        self.vendor_id = vendor_id
        self.product_id = product_id
        self.extension_unit_id = extension_unit_id
        self.interface_id = interface_id
        self.dev = None
        self.connect()

    def connect(self):
        """Connect to the USB device."""
        self.dev = usb.core.find(idVendor=self.vendor_id, idProduct=self.product_id)
        if self.dev is None:
            raise ValueError('Device not found')
        
        # Set the active configuration. With no arguments, the first
        # configuration will be the active one
        self.dev.set_configuration()

        # Get an endpoint instance
        cfg = self.dev.get_active_configuration()
        intf = cfg[(self.interface_id, 0)]

        self.ep = usb.util.find_descriptor(
            intf,
            # Match the first OUT endpoint
            custom_match = \
            lambda e: \
                usb.util.endpoint_direction(e.bEndpointAddress) == \
                usb.util.ENDPOINT_OUT)

        assert self.ep is not None, "Endpoint not found"

    def _calculate_windex(self) -> int:
        """Calculate wIndex based on extension unit and interface IDs."""
        return (self.extension_unit_id << 8) | self.interface_id

    def send_control_request(self, bRequest: int, wValue: int, data: List[int]):
        """Send a control request to the USB device."""
        wIndex = self._calculate_windex()
        wLength = len(data)
        self.dev.ctrl_transfer(self.BM_REQUEST_TYPE_SET, bRequest, wValue, wIndex, data, wLength)

    def get_control_data(self, bRequest: int, wValue: int, length: int) -> List[int]:
        """Get data from the USB device using a control request."""
        wIndex = self._calculate_windex()
        return self.dev.ctrl_transfer(self.BM_REQUEST_TYPE_GET, bRequest, wValue, wIndex, length)

    def set_parameter(self, xu_cs_id: int, data: List[int]):
        """Set a parameter on the device."""
        self.send_control_request(self.SET_CUR, xu_cs_id, data)

    def get_parameter(self, xu_cs_id: int, length: int) -> List[int]:
        """Get a parameter from the device."""
        return self.get_control_data(self.GET_CUR, xu_cs_id, length)

    def write_data(self, data: bytes):
        """Write data to the OUT endpoint."""
        self.ep.write(data)

# Example usage
if __name__ == "__main__":
    try:
        xu_control = USBXUControl()  # Now uses default VID and PID from the Hardware ID
        
        # Set a parameter
        xu_control.set_parameter(xu_control.XU_CS_ID_COMMAND_SWITCH, [0x01, 0x02])
        
        # Get a parameter
        response = xu_control.get_parameter(xu_control.XU_CS_ID_PROTOCOL_VER, 2)
        print("Response:", response)

        # Write some data
        xu_control.write_data(b'Hello, USB device!')
    
    except ValueError as e:
        print(f"Error: {e}")
    except usb.core.USBError as e:
        print(f"USB Error: {e}")