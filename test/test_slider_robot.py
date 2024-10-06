from slider_robot import SliderRobot
from slider_robot import SliderRobotManager

def test_slider_robot_manager():
    # 创建SliderRobot实例
    robot = SliderRobot(port='/tmp/vserial2', slave_address=1, baud_rate=9600)

    # 定义栏位位置
    positions = [0, 100, 200, 300, 400]  # 单位: mm

    # 创建SliderRobotManager实例
    manager = SliderRobotManager(robot, positions)

    # 回零
    # manager.home()
    # 清零当前位置
    manager.clear_position()
    # 定义拍照回调函数
    def photo_callback():
        manager.take_photo()
        print(f"Current position: {manager.get_current_position()}")

    # 移动到第2个位置并拍照
    manager.move_to_position(1, photo_callback)
    manager.execute_callbacks()

    # 移动到下一个位置并拍照
    manager.move_to_next_position(photo_callback)
    manager.execute_callbacks()

    # 扫描所有位置并拍照
    manager.scan_all_positions(photo_callback)

    # 测试紧急停止和恢复
    manager.emergency_stop()
    manager.resume()

    # 移动到第一个位置
    manager.move_to_position(0)

if __name__ == "__main__":
    test_slider_robot_manager()