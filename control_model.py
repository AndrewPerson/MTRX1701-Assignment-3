# function: control_model
# inputs:
#   right_sensor_dist - distance from right sensor to track
#   left_sensor_dist - distance from left sensor to track
#   v_max - maximum velocity
def control_model(
    right_sensor_dist: float,
    left_sensor_dist: float,
    v_max: float,
    use_proportional=False,
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
    sensitivity = 2.0

    right_vel = v_max
    left_vel = v_max

    if right_sensor_dist <= 0:
        error = abs(right_sensor_dist)
        right_vel = v_max - (sensitivity * error)

        if right_vel < 0:
            right_vel = 0
    elif left_sensor_dist <= 0:
        error = abs(left_sensor_dist)
        left_vel = v_max - (sensitivity * error)

        if left_vel < 0:
            left_vel = 0

    return right_vel, left_vel
