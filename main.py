# import rclpy
#
# from src.controller.robot_auto_controller import RobotAutoController
#
#
# def main():
#     rclpy.init()
#     robot = None
#
#     try:
#         robot = RobotAutoController()
#         # External navigation goal: only x and y, no final heading adjustment.
#         robot.go_to(1.0, 1.0)
#
#     except TimeoutError as error:
#         if robot is not None:
#             robot.get_logger().error(str(error))
#
#     except KeyboardInterrupt:
#         pass
#
#     finally:
#         if robot is not None:
#             robot.stop()
#             robot.close_log()
#             robot.destroy_node()
#
#         if rclpy.ok():
#             rclpy.shutdown()
#
#
# if __name__ == "__main__":
#     main()

import rclpy

from src.controller.robot_auto_controller import RobotAutoController

# Waypoints in the world coordinate system (metres)
WAYPOINTS = [
    (1.0, 2.0),
    (2.0, 1.0),
    (3.0, 3.0)
]


def main():
    rclpy.init()
    robot = None

    try:
        robot = RobotAutoController()

        for index, (x, y) in enumerate(WAYPOINTS, start=1):

            robot.get_logger().info(
                f"Navigating to waypoint {index}: ({x}, {y})"
            )

            success = robot.go_to(
                x,
                y,
                timeout=90.0
            )

            if not success:
                robot.get_logger().error(
                    f"Failed to reach waypoint {index}"
                )
                break

            robot.get_logger().info(
                f"Waypoint {index} completed"
            )

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
