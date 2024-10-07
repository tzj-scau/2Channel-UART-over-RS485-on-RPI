import os
import cv2
import numpy as np
from plyfile import PlyData, PlyElement
from pyorbbecsdk import *
from utils import frame_to_bgr_image
from config import FileConfig
import threading, time, schedule

config = FileConfig()
save_points_dir = config.get_save_path('ply')
save_depth_image_dir = config.get_save_path('depth')
save_color_image_dir = config.get_save_path('rgb')
save_inf_image_dir = config.get_save_path('inf')

for dir_path in [save_points_dir, save_depth_image_dir, save_color_image_dir, save_inf_image_dir]:
    if not os.path.exists(dir_path):
        os.mkdir(dir_path)

class Camera:
    def __init__(self, device_index):
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
        self.__initialize_device()

    def __initialize_device(self):
        with self.lock:
            if self.running:
                return
            try:
                profile_list = self.pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
                color_profile = profile_list.get_default_video_stream_profile()
                self.config.enable_stream(color_profile)
                self.has_color_sensor = True
            except OBError as e:
                print(e)
                self.has_color_sensor = False

            profile_list = self.pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
            depth_profile = profile_list.get_default_video_stream_profile()
            self.config.enable_stream(depth_profile)

            self.pipeline.start(self.config)
            self.running = True

    def get_device(self):
        return self.device

    def stop_stream(self):
        with self.lock:
            if not self.running:
                return
            self.pipeline.stop()
            self.running = False

    def capture_data(self):
        saved_color_cnt = saved_depth_cnt = 0
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
        self.save_depth_frame(depth_frame, saved_depth_cnt, timestamp)
        saved_depth_cnt += 1

        if color_frame:
            self.save_color_frame(color_frame, saved_color_cnt, color_frame.get_timestamp())
            saved_color_cnt += 1

    def save_data(self, save_image_num):
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
                    saved_depth_cnt += self.save_points_to_ply(frames, camera_param, timestamp)
                    self.save_depth_frame(depth_frame, saved_depth_cnt, timestamp)

                if color_frame and saved_color_cnt < save_image_num:
                    self.save_color_frame(color_frame, saved_color_cnt, color_frame.get_timestamp())
                    saved_color_cnt += 1

                if saved_depth_cnt >= save_image_num and (not self.has_color_sensor or saved_color_cnt >= save_image_num):
                    break
        except KeyboardInterrupt:
            print("Interrupted by user")
        # finally:
        #     self.stop_stream()

    def save_points_to_ply(self, frames: FrameSet, camera_param: OBCameraParam, timestamp) -> int:
        points = frames.get_point_cloud(camera_param)
        if len(points) == 0:
            print("no depth points")
            return 0
        points_array = np.array([tuple(point) for point in points], dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
        points_filename = os.path.join(save_points_dir, f"points_{timestamp}.ply")
        el = PlyElement.describe(points_array, 'vertex')
        PlyData([el], text=True).write(points_filename)
        return 1

    def save_depth_frame(self, frame: DepthFrame, index, timestamp):
        data = np.frombuffer(frame.get_data(), dtype=np.uint16).reshape((frame.get_height(), frame.get_width()))
        data = (data * frame.get_depth_scale()).astype(np.uint16)
        raw_filename = os.path.join(save_depth_image_dir,
                                    f"depth_{frame.get_width()}x{frame.get_height()}_{index}_{timestamp}.raw")
        data.tofile(raw_filename)

    def save_color_frame(self, frame: ColorFrame, index, timestamp):
        image = frame_to_bgr_image(frame)
        if image is None:
            print("failed to convert frame to image")
            return
        filename = os.path.join(save_color_image_dir,
                                f"color_{frame.get_width()}x{frame.get_height()}_{index}_{timestamp}.png")
        cv2.imwrite(filename, image)

    def __del__(self):
        self.stop_stream()

if __name__ == "__main__":
    camera = Camera(0)
    # camera.save_data()
    # camera.stop_stream()
    try:
        schedule.every(5).minutes.do(camera.save_data, 10)

        while True:
            schedule.run_pending()
            time.sleep(1)
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        camera.stop_stream()