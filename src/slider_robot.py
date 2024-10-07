import logging
from epoll_motor import MotorDriver, Registers
import math
import time
from typing import List

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

class SliderRobot:
    def __init__(self, motor_driver: MotorDriver, positions: List[float], reduction_ratio=19, wheel_diameter=34.0, counts_per_revolution=3200):
        self.motor_driver = motor_driver
        self.reduction_ratio = reduction_ratio
        self.wheel_diameter = wheel_diameter
        self.counts_per_revolution = counts_per_revolution
        self.mm_per_count = self.calculate_mm_per_count()
        self.positions = positions
        self.current_position_index = 0
        self.is_homed = False

    def calculate_mm_per_count(self):
        linear_movement_per_rev = (math.pi * self.wheel_diameter) / self.reduction_ratio
        return linear_movement_per_rev / self.counts_per_revolution

    def enable_motor(self, enable=True, sync_flag=False):
        self.motor_driver.enable_motor(enable, sync_flag)
        logging.debug(f"Motor {'enabled' if enable else 'disabled'}")

    def stop(self, sync_flag=False):
        self.motor_driver.stop(sync_flag)
        logging.debug("Motor stopped")

    def trigger_homing(self, mode=0, sync_flag=False):
        self.motor_driver.trigger_homing(mode, sync_flag)
        logging.info("Homing the robot...")
        while True:
            status = self.read_homing_status()
            if status and not status['homing_failed'] and not status['homing_in_progress']:
                break
            time.sleep(0.1)
        self.is_homed = True
        self.current_position_index = 0
        logging.info("Homing completed")

    def set_velocity(self, velocity_mm_per_s, acceleration=300, sync_flag=False):
        counts_per_s = velocity_mm_per_s / self.mm_per_count
        self.motor_driver.set_velocity(counts_per_s, acceleration, sync_flag)
        logging.debug(f"Velocity set to {velocity_mm_per_s} mm/s")

    def set_position(self, position_mm, velocity=300, relative=True, sync_flag=False):
        direction = 0 if position_mm >= 0 else 1
        abs_position_mm = abs(position_mm)
        counts = int(abs_position_mm / self.mm_per_count)
        self.motor_driver.set_position(counts, direction, velocity, relative, sync_flag)
        logging.debug(f"Position set to {position_mm} mm")

    def set_position_planned(self, position_mm, max_velocity_rpm, accel_rpm_per_s, decel_rpm_per_s, relative=True, sync_flag=False):
        direction = 0 if position_mm >= 0 else 1
        abs_position_mm = abs(position_mm)
        counts = int(abs_position_mm / self.mm_per_count)
        self.motor_driver.set_position_planned(
            counts, direction, max_velocity_rpm, accel_rpm_per_s, decel_rpm_per_s, relative, sync_flag
        )
        logging.debug(f"Planned position set to {position_mm} mm")

    def read_position(self):
        direction, position_counts = self.motor_driver.read_position_raw()
        if direction is None or position_counts is None:
            return None
        position_mm = position_counts * self.mm_per_count
        return -position_mm if direction else position_mm

    def read_velocity(self):
        velocity_counts_per_s = self.motor_driver.read_velocity()
        if velocity_counts_per_s is None:
            return None
        velocity_mm_per_s = velocity_counts_per_s * self.mm_per_count
        return velocity_mm_per_s

    def read_motor_status(self):
        return self.motor_driver.read_motor_status()

    def read_homing_status(self):
        return self.motor_driver.read_homing_status()

    def read_system_status(self):
        return self.motor_driver.read_system_status()

    def read_driver_params(self):
        return self.motor_driver.read_driver_params()

    def modify_driver_params(self, params, store=True):
        self.motor_driver.modify_driver_params(params, store)
        logging.info("Driver parameters modified")

    def calibrate_encoder(self):
        self.motor_driver.calibrate_encoder()
        time.sleep(2)
        logging.info("Encoder calibration completed")

    def clear_position(self):
        self.motor_driver.clear_position()
        self.current_position_index = 0
        logging.info("Position cleared")

    def factory_reset(self):
        self.motor_driver.factory_reset()
        time.sleep(2)
        logging.info("Factory reset completed. Please reconfigure the robot.")

    def clear_clog(self):
        self.motor_driver.clear_clog()
        logging.info("Clog protection cleared")

    def set_pid_params(self, pp_trapezoid, pp_direct, vp, vi, store=True):
        self.motor_driver.set_pid_params(pp_trapezoid, pp_direct, vp, vi, store)
        logging.info("PID parameters updated")

    def set_homing_params(self, mode, direction, velocity, timeout, store=True):
        self.motor_driver.set_homing_params(mode, direction, velocity, timeout, store)
        logging.info("Homing parameters updated")

    def home(self, sync_flag=False):
        self.trigger_homing(mode=0, sync_flag=sync_flag)

    def move_to_position(self, position_index: int, use_planned_motion: bool = True):
        if not self.is_homed:
            logging.warning("Robot not homed. Homing first...")
            self.home()

        if 0 <= position_index < len(self.positions):
            target_position = self.positions[position_index]
            logging.info(f"Moving to position {position_index} ({target_position} mm)...")

            if use_planned_motion:
                self.set_position_planned(
                    target_position,
                    max_velocity_rpm=300,
                    accel_rpm_per_s=250,
                    decel_rpm_per_s=250,
                    relative=False,
                    sync_flag=False,
                )
            else:
                self.set_position(target_position, velocity=300, relative=False)

            while not self._is_in_position():
                time.sleep(0.5)

            self.current_position_index = position_index
            logging.info(f"Moved to position {position_index}")
        else:
            logging.error("Invalid position index")

    def _is_in_position(self) -> bool:
        time.sleep(3)
        status = self.read_motor_status()
        return status['in_position'] if status else False

    def move_to_next_position(self, use_planned_motion: bool = True):
        next_position = (self.current_position_index + 1) % len(self.positions)
        self.move_to_position(next_position, use_planned_motion)

    def move_to_previous_position(self, use_planned_motion: bool = True):
        prev_position = (self.current_position_index - 1) % len(self.positions)
        self.move_to_position(prev_position, use_planned_motion)

    def scan_all_positions(self, use_planned_motion: bool = True):
        for i in range(len(self.positions)):
            self.move_to_position(i, use_planned_motion)

    def get_current_position_info(self):
        return {
            "index": self.current_position_index,
            "position": self.read_position(),
            "target_position": self.positions[self.current_position_index],
            "velocity": self.read_velocity(),
            "motor_status": self.read_motor_status(),
            "homing_status": self.read_homing_status()
        }

    def emergency_stop(self):
        logging.warning("Emergency stop triggered!")
        self.stop(sync_flag=False)

    def resume(self):
        logging.info("Resuming operation...")
        self.enable_motor(enable=True)
        logging.info("Robot enabled. Ready to receive new commands.")
        
        

def main():
    # Initialize MotorDriver
    motor_driver = MotorDriver(
        port='/tmp/vserial2',
        slave_address=1,
        baud_rate=9600
    )

    # Define positions in millimeters
    positions = [1000.0, 2000.0, 3000.0, 4000.0]

    # Initialize SliderRobot
    robot = SliderRobot(
        motor_driver=motor_driver,
        positions=positions,
        reduction_ratio=19,
        wheel_diameter=34.0,
        counts_per_revolution=3200
    )

    try:
        # 1. Homing operation
        logging.info("Testing Homing")
        robot.home()

        # 2. Move to each position
        logging.info("Testing Position Movement")
        for i in range(len(positions)):
            robot.move_to_position(i, use_planned_motion=True)

        # 3. Test next and previous position movement
        logging.info("Testing Next and Previous Position Movement")
        robot.move_to_next_position()
        robot.move_to_previous_position()

        # 4. Get current position information
        logging.info("Current Position Info")
        position_info = robot.get_current_position_info()
        logging.info(f"Position info: {position_info}")

        # 5. Test system status and driver parameters
        logging.info("System Status")
        system_status = robot.read_system_status()
        logging.info(f"System status: {system_status}")

        logging.info("Driver Parameters")
        driver_params = robot.read_driver_params()
        logging.info(f"Driver parameters: {driver_params}")

        # # 6. Modify PID parameters
        # logging.info("Updating PID Parameters")
        # robot.set_pid_params(pp_trapezoid=100, pp_direct=200, vp=300, vi=400)

        # 7. Test emergency stop and resume
        logging.info("Testing Emergency Stop and Resume")
        robot.move_to_position(2, use_planned_motion=True)
        time.sleep(0.5)
        robot.emergency_stop()
        time.sleep(1)
        robot.resume()

        # 8. Test scanning all positions
        logging.info("Scanning All Positions")
        robot.scan_all_positions()

        # 9. Test clearing position
        logging.info("Clearing Position")
        robot.clear_position()

        # 10. Test calibrating encoder
        logging.info("Calibrating Encoder")
        robot.calibrate_encoder()

    except Exception as e:
        logging.error(f"An error occurred: {e}")

    finally:
        # Ensure the motor is stopped at the end
        robot.emergency_stop()
        logging.info("Test completed. Motor stopped.")

if __name__ == "__main__":
    main()