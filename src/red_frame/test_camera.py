import logging
import os
import time
from typing import NamedTuple, Optional

import cv2
import uvc.uvc_bindings as uvc
from rich.logging import RichHandler
from rich.traceback import install as install_rich_traceback


class CameraSpec(NamedTuple):
    name: str
    width: int
    height: int
    fps: int
    bandwidth_factor: float = 1.0


def main(camera_spec: CameraSpec):
    devices = uvc.device_list()
    camera = init_camera_from_list(devices, camera_spec)
    if not camera:
        raise RuntimeError(
            "Could not initialize the specified camera. Available: "
            f"{[dev['name'] for dev in devices]}"
        )

    try:
        last_update = time.perf_counter()

        while True:
            try:
                frame = camera.get_frame(timeout=0.001)
            except TimeoutError:
                continue
            except uvc.InitError as err:
                logging.debug(f"Failed to init {camera_spec}: {err}")
                break
            except uvc.StreamError as err:
                logging.debug(f"Failed to get a frame for {camera_spec}: {err}")
                continue
            except Exception as e:
                logging.error(f"Unexpected error: {e}")
                continue
            else:
                try:
                    data = frame.bgr if hasattr(frame, "bgr") else frame.gray
                    if frame.data_fully_received:
                        cv2.imwrite(f'{camera_spec.name}_frame.jpg', data)
                except cv2.error as cv_err:
                    logging.error(f"OpenCV error processing frame: {cv_err}")
                    continue

            if (time.perf_counter() - last_update) > 1 / 60:
                last_update = time.perf_counter()

    except KeyboardInterrupt:
        pass

    camera.close()


def init_camera_from_list(devices, camera: CameraSpec) -> Optional[uvc.Capture]:
    logging.debug(f"Searching {camera}...")
    for device in devices:
        if device["name"] == camera.name:
            logging.debug(f"Found match by name")
            capture = uvc.Capture(device["uid"])
            capture.bandwidth_factor = camera.bandwidth_factor
            for mode in capture.available_modes:
                if mode[:3] == (camera.width, camera.height, camera.fps):
                    capture.frame_mode = mode
                    logging.debug(f"Setting mode: {mode}")
                    return capture
            else:
                logging.warning(
                    f"None of the available modes matched: {capture.available_modes}"
                )
            capture.close()
    else:
        logging.warning(f"No matching camera with name {camera.name!r} found")


if __name__ == "__main__":
    os.environ["LIBUSB_DEBUG"] = "3"
    install_rich_traceback()
    logging.basicConfig(
        level=logging.NOTSET,
        handlers=[RichHandler(level="DEBUG")],
        format="%(message)s",
        datefmt="[%X]",
    )

    camera_spec = CameraSpec(
        name="HikCamera",  # Use the specific camera name
        width=640,
        height=360,
        fps=30,
        bandwidth_factor=2.0,  # Consider reducing this if errors persist
    )
    
    main(camera_spec)
