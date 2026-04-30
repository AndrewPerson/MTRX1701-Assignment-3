# function: control_model
# inputs:
#   right_sensor_dist - distance from right sensor to track
#   left_sensor_dist - distance from left sensor to track
#   v_max - maximum velocity
def control_model(
    right_sensor_dist: float,
    left_sensor_dist: float,
    v_max: float,
    use_proportional: bool = False,
):
    if use_proportional:
        return control_model_proportional(right_sensor_dist, left_sensor_dist, v_max)
    else:
        return control_model_bang_bang(right_sensor_dist, left_sensor_dist, v_max)


# function: control_model_bang_bang
# inputs:
#   right_sensor_dist - distance from right sensor to track
#   left_sensor_dist - distance from left sensor to track
#   v_max - maximum velocity
def control_model_bang_bang(
    right_sensor_dist: float, left_sensor_dist: float, v_max: float
):
    # left_sensor_dist is intended to be < 0
    # right_sensor_dist is intended to be > 0

    if right_sensor_dist <= 0:
        v_r = 0
        v_l = v_max
    elif left_sensor_dist >= 0:
        v_l = 0
        v_r = v_max
    else:
        v_r = v_max
        v_l = v_max

    return v_r, v_l


# function: control_model_proportional
# inputs:
#   right_sensor_dist - distance from right sensor to track
#   left_sensor_dist - distance from left sensor to track
#   v_max - maximum velocity
def control_model_proportional(
    right_sensor_dist: float,
    left_sensor_dist: float,
    v_max: float,
):
    sensitivity = 10

    right_vel = v_max
    left_vel = v_max

    # left_sensor_dist is intended to be < 0
    # right_sensor_dist is intended to be > 0
    # error > 0 => robot is on the right side of the line
    # error < 0 => robot is on the left side of the line

    error = right_sensor_dist + left_sensor_dist

    # error = inf => right_vel = 1, left_vel = -1 (turn left)
    # error = -inf => right_vel = -1, left_vel = 1 (turn right)
    # error = 0 => right_vel = 1, left_vel = 1 (straight ahead)

    left_vel = 1 if error <= 0 else 2 / (1 + sensitivity * error) - 1
    right_vel = 1 if error >= 0 else 2 / (1 + sensitivity * -error) - 1

    return max(-v_max, min(v_max * right_vel, v_max)), max(
        0, min(v_max * left_vel, v_max)
    )
