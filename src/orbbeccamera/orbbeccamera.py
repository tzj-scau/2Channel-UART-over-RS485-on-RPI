import os
import cv2
import numpy as np
from plyfile import PlyData, PlyElement
from pyorbbecsdk import *
from utils import frame_to_bgr_image
from config import FileConfig
import threading
import time
import schedule

config = FileConfig()
save_points_dir = config.get_save_path('ply')
save_depth_image_dir = config.get_save_path('depth')
save_color_image_dir = config.get_save_path('rgb')
save_inf_image_dir = config.get_save_path('inf')

# Create directories if they don't exist
for dir_path in [save_points_dir, save_depth_image_dir, save_color_image_dir, save_inf_image_dir]:
    os.makedirs(dir_path, exist_ok=True)

class OrbbecCamera:
    """
    A class to interface with Orbbec cameras, capture frames, and save data.
    """

    def __init__(self, device_index=0):
        """
        Initialize the Orbbec camera.

        Args:
            device_index (int): Index of the device to use.
        """
        self.device_index = device_index
        self.ctx = Context()
        self.device_list = self.ctx.query_devices()
        if self.device_list.get_count() <= device_index:
            raise ValueError(f"Device index {device_index} out of range")
        self.device = self.device_list.get_device_by_index(device_index)
        self.pipeline = Pipeline(self.device)
        self.config = Config()
        self.has_color_sensor = False
        self.running = False
        self.lock = threading.Lock()
        self._initialize_device()

    def _initialize_device(self):
        """
        Initialize the device streams and start the pipeline.
        """
        with self.lock:
            if self.running:
                return
            try:
                # Enable color stream if available
                profile_list = self.pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
                color_profile = profile_list.get_default_video_stream_profile()
                self.config.enable_stream(color_profile)
                self.has_color_sensor = True
            except OBError as e:
                print(f"Color sensor not found: {e}")
                self.has_color_sensor = False

            # Enable depth stream
            profile_list = self.pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
            depth_profile = profile_list.get_default_video_stream_profile()
            self.config.enable_stream(depth_profile)

            # Start the pipeline
            self.pipeline.start(self.config)
            self.running = True

    def get_device(self):
        """
        Get the current device.

        Returns:
            Device: The current device.
        """
        return self.device

    def stop_stream(self):
        """
        Stop the data stream from the camera.
        """
        with self.lock:
            if not self.running:
                return
            self.pipeline.stop()
            self.running = False

    def capture_data(self):
        """
        Capture a single set of frames and save the data.
        """
        if not self.running:
            print("Device is not running")
            return

        frames = self.pipeline.wait_for_frames(500)
        if frames is None:
            print("No frames received")
            return

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame() if self.has_color_sensor else None
        camera_param = self.pipeline.get_camera_param()

        timestamp = depth_frame.get_timestamp()
        self.save_points_to_ply(frames, camera_param, timestamp)
        self.save_depth_frame(depth_frame, timestamp)

        if color_frame:
            self.save_color_frame(color_frame, timestamp)

    def save_data(self, save_image_num):
        """
        Continuously capture and save frames until the specified number of images have been saved.

        Args:
            save_image_num (int): The number of images to save.
        """
        saved_color_cnt = saved_depth_cnt = 0
        try:
            while self.running:
                frames = self.pipeline.wait_for_frames(100)
                if frames is None:
                    continue

                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame() if self.has_color_sensor else None
                camera_param = self.pipeline.get_camera_param()

                if depth_frame and saved_depth_cnt < save_image_num:
                    timestamp = depth_frame.get_timestamp()
                    if self.save_points_to_ply(frames, camera_param, timestamp):
                        saved_depth_cnt += 1
                    self.save_depth_frame(depth_frame, timestamp)

                if color_frame and saved_color_cnt < save_image_num:
                    self.save_color_frame(color_frame, timestamp)
                    saved_color_cnt += 1

                if saved_depth_cnt >= save_image_num and (not self.has_color_sensor or saved_color_cnt >= save_image_num):
                    break
        except KeyboardInterrupt:
            print("Interrupted by user")
        finally:
            self.stop_stream()

    def save_points_to_ply(self, frames, camera_param, timestamp):
        """
        Save the point cloud data to a PLY file.

        Args:
            frames (FrameSet): The frames containing the point cloud data.
            camera_param (OBCameraParam): Camera parameters.
            timestamp (int): The timestamp of the frame.

        Returns:
            bool: True if data was saved, False otherwise.
        """
        points = frames.get_point_cloud(camera_param)
        if len(points) == 0:
            print("No depth points")
            return False

        points_array = np.array([tuple(point) for point in points], dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
        points_filename = os.path.join(save_points_dir, f"points_{timestamp}.ply")
        el = PlyElement.describe(points_array, 'vertex')
        PlyData([el], text=True).write(points_filename)
        return True

    def save_depth_frame(self, frame, timestamp):
        """
        Save the depth frame data to a file.

        Args:
            frame (DepthFrame): The depth frame to save.
            timestamp (int): The timestamp of the frame.
        """
        data = np.frombuffer(frame.get_data(), dtype=np.uint16).reshape((frame.get_height(), frame.get_width()))
        data = (data * frame.get_depth_scale()).astype(np.uint16)
        raw_filename = os.path.join(
            save_depth_image_dir,
            f"depth_{frame.get_width()}x{frame.get_height()}_{timestamp}.raw"
        )
        data.tofile(raw_filename)

    def save_color_frame(self, frame, timestamp):
        """
        Save the color frame as an image file.

        Args:
            frame (ColorFrame): The color frame to save.
            timestamp (int): The timestamp of the frame.
        """
        image = frame_to_bgr_image(frame)
        if image is None:
            print("Failed to convert frame to image")
            return
        filename = os.path.join(
            save_color_image_dir,
            f"color_{frame.get_width()}x{frame.get_height()}_{timestamp}.png"
        )
        cv2.imwrite(filename, image)

    def __del__(self):
        """
        Destructor to ensure the stream is stopped when the object is deleted.
        """
        self.stop_stream()

if __name__ == "__main__":
    camera = OrbbecCamera(0)
    try:
        schedule.every(5).minutes.do(camera.save_data, 10)

        while True:
            schedule.run_pending()
            time.sleep(1)
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        camera.stop_stream()
