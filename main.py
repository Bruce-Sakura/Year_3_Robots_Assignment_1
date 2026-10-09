import rclpy

from src.controller.robot_auto_controller import RobotAutoController


def main():
    rclpy.init()
    robot = None

    try:
        robot = RobotAutoController()
        # External navigation goal: only x and y, no final heading adjustment.
        robot.go_to(1.0, 1.0)

    except TimeoutError as error:
        if robot is not None:
            robot.get_logger().error(str(error))

    except KeyboardInterrupt:
        pass

    finally:
        if robot is not None:
            robot.stop()
            robot.close_log()
            robot.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
