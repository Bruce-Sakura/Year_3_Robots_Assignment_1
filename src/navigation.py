import math


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def calculate_motion(
    x, y, theta,
    target_x, target_y, target_theta,
    left_wheel_radius,
    right_wheel_radius,
    wheel_separation
):
    """Calculate robot velocity and wheel angular velocities."""

    # Step 1: Calculate target distance
    dx = target_x - x
    dy = target_y - y
    distance = math.hypot(dx, dy)

    # Step 2: Calculate robot velocity
    if distance > 0.03:
        desired_angle = math.atan2(dy, dx)
        angle_error = normalize_angle(desired_angle - theta)

        # Proportional angular control
        angular = max(-0.8, min(0.8, 2.0 * angle_error))

        # Rotate before driving when heading error is large
        if abs(angle_error) > 0.2:
            linear = 0.0
        else:
            linear = min(0.15, 0.8 * distance)

        arrived = False

    else:
        # Arrived at target position
        linear = 0.0

        if target_theta is None:
            angular = 0.0
            arrived = True
        else:
            angle_error = normalize_angle(target_theta - theta)

            angular = max(-0.8, min(0.8, 2.0 * angle_error))
            arrived = abs(angle_error) < 0.05

            if arrived:
                angular = 0.0

    # Step 3: Differential-drive inverse kinematics
    left_speed = (
        linear - angular * wheel_separation / 2
    ) / left_wheel_radius

    right_speed = (
        linear + angular * wheel_separation / 2
    ) / right_wheel_radius

    return linear, angular, left_speed, right_speed, arrived


if __name__ == "__main__":
    result = calculate_motion(
        x=0.0,
        y=0.0,
        theta=0.0,

        target_x=1.0,
        target_y=1.0,
        target_theta=1.0,

        left_wheel_radius=0.03,
        right_wheel_radius=0.03,
        wheel_separation=0.18
    )

    print(result)