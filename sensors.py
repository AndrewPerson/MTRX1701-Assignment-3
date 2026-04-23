import numpy as np
import math


# function: relative_sensor_positions
# inputs:
#   robot_state - vehicle state at time k [x, y, heading]
#   sensor_radius_from_center - range from centre of robot to sensor
#   sensor_angle_from_center - angle from centre of robot to sensor
# returns:
#   r_r, r_l - the relative positions of the right and left sensors compared to the robot's centre
def relative_sensor_positions(
    robot_state, sensor_radius_from_center, sensor_angle_from_center
):
    heading = robot_state[2]

    r_r = sensor_radius_from_center * np.array(
        [
            math.cos(heading - sensor_angle_from_center),
            math.sin(heading - sensor_angle_from_center),
        ]
    )
    r_l = sensor_radius_from_center * np.array(
        [
            math.cos(heading + sensor_angle_from_center),
            math.sin(heading + sensor_angle_from_center),
        ]
    )

    return r_r, r_l


# function: sensor_positions
# inputs:
#   robot_state - vehicle state at time k [x, y, heading]
#   sensor_radius_from_center - range from centre of robot to sensor
#   sensor_angle_from_center - angle from centre of robot to sensor
# returns:
#   s_r, s_l - the positions of the right and left sensors
def sensor_positions(robot_state, sensor_radius_from_center, sensor_angle_from_center):
    pos = robot_state[:2]

    r_r, r_l = relative_sensor_positions(
        robot_state, sensor_radius_from_center, sensor_angle_from_center
    )

    s_r, s_l = pos + r_r, pos + r_l

    return s_r, s_l
