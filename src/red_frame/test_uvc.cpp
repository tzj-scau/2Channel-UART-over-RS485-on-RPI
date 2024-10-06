#include <libuvc/libuvc.h>
#include <iostream>
#include <vector>
#include <cstring>
#include <unistd.h>

// UVC Extension Unit 2.0 Protocol Constants
const uint8_t XU_CS_ID_COMMAND_SWITCH = 0x05;
const uint8_t XU_UNIT_ID = 0x0A;

// Function to send UVC Extension Unit request
uvc_error_t send_xu_command(uvc_device_handle_t *devh, uint8_t cs_id, uint8_t sub_function_id, 
                            const uint8_t *data_in, size_t data_in_len, 
                            uint8_t *data_out, size_t *data_out_len) {
    uint8_t bmRequestType = 0x21;  // Host to Device, Class request, Interface recipient
    uint8_t bRequest = 0x01;       // SET_CUR
    uint16_t wValue = (cs_id << 8) | 0x00;
    uint16_t wIndex = (XU_UNIT_ID << 8) | 0x00;
    
    // Step 1: Function switch
    uint8_t switch_data[2] = {XU_CS_ID_COMMAND_SWITCH, sub_function_id};
    uvc_error_t res = uvc_set_ctrl(devh, bmRequestType, bRequest, wValue, wIndex, switch_data, sizeof(switch_data));
    if (res < 0) {
        std::cerr << "Failed to switch function: " << uvc_strerror(res) << std::endl;
        return res;
    }

    // Step 2: Send actual command
    if (data_in && data_in_len > 0) {
        res = uvc_set_ctrl(devh, bmRequestType, bRequest, wValue, wIndex, data_in, data_in_len);
        if (res < 0) {
            std::cerr << "Failed to send command: " << uvc_strerror(res) << std::endl;
            return res;
        }
    }

    // Step 3: Get response if needed
    if (data_out && data_out_len && *data_out_len > 0) {
        bmRequestType = 0xA1;  // Device to Host, Class request, Interface recipient
        bRequest = 0x81;       // GET_CUR
        res = uvc_get_ctrl(devh, bmRequestType, bRequest, wValue, wIndex, data_out, data_out_len);
        if (res < 0) {
            std::cerr << "Failed to get response: " << uvc_strerror(res) << std::endl;
            return res;
        }
    }

    return UVC_SUCCESS;
}

// Callback function for video stream
void cb(uvc_frame_t *frame, void *ptr) {
    std::cout << "Frame received: " << frame->sequence << std::endl;
    // Process frame here
}

int main() {
    uvc_context_t *ctx;
    uvc_device_t *dev;
    uvc_device_handle_t *devh;
    uvc_stream_ctrl_t ctrl;
    uvc_error_t res;

    // Initialize UVC
    res = uvc_init(&ctx, NULL);
    if (res < 0) {
        std::cerr << "Failed to initialize UVC: " << uvc_strerror(res) << std::endl;
        return -1;
    }

    // Find device
    res = uvc_find_device(ctx, &dev, 0, 0, NULL);
    if (res < 0) {
        std::cerr << "Failed to find device: " << uvc_strerror(res) << std::endl;
        uvc_exit(ctx);
        return -1;
    }

    // Open device
    res = uvc_open(dev, &devh);
    if (res < 0) {
        std::cerr << "Failed to open device: " << uvc_strerror(res) << std::endl;
        uvc_unref_device(dev);
        uvc_exit(ctx);
        return -1;
    }

    // Print device info
    uvc_print_diag(devh, stderr);

    // Set up stream parameters
    res = uvc_get_stream_ctrl_format_size(devh, &ctrl, UVC_FRAME_FORMAT_YUYV, 640, 480, 30);
    if (res < 0) {
        std::cerr << "Failed to get stream control: " << uvc_strerror(res) << std::endl;
        uvc_close(devh);
        uvc_unref_device(dev);
        uvc_exit(ctx);
        return -1;
    }

    // Start streaming
    res = uvc_start_streaming(devh, &ctrl, cb, NULL, 0);
    if (res < 0) {
        std::cerr << "Failed to start streaming: " << uvc_strerror(res) << std::endl;
        uvc_close(devh);
        uvc_unref_device(dev);
        uvc_exit(ctx);
        return -1;
    }

    // Example: Set time
    uint8_t set_time_data[7] = {0x03, 0x2E, 0x10, 0x15, 0x11, 0x0E, 0x40};  // Example time data
    res = send_xu_command(devh, 0x01, 0x00, set_time_data, sizeof(set_time_data), NULL, NULL);
    if (res == UVC_SUCCESS) {
        std::cout << "Time set successfully" << std::endl;
    }

    // Example: Get time
    uint8_t get_time_data[7];
    size_t get_time_data_len = sizeof(get_time_data);
    res = send_xu_command(devh, 0x01, 0x00, NULL, 0, get_time_data, &get_time_data_len);
    if (res == UVC_SUCCESS) {
        std::cout << "Current time: ";
        for (size_t i = 0; i < get_time_data_len; ++i) {
            printf("%02X ", get_time_data[i]);
        }
        std::cout << std::endl;
    }

    // Stream for 10 seconds
    sleep(10);

    // Stop streaming
    uvc_stop_streaming(devh);
    std::cout << "Streaming stopped" << std::endl;

    // Clean up
    uvc_close(devh);
    uvc_unref_device(dev);
    uvc_exit(ctx);

    return 0;
}