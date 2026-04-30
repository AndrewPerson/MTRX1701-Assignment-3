import numpy as np
from sensors import sensor_positions
from track_model import track_model
from control_model import control_model
from vehicle_model import vehicle_model, control_vector


def simulate(
    timestamps,
    dt: float,
    initial_robot_state,
    robot_width: float,
    sensor_radius_from_center: float,
    sensor_angle_from_center: float,
    ellipse_x: float,
    ellipse_y: float,
    ellipse_radius_a: float,
    ellipse_radius_b: float,
    thickness: float,
    v_max: float,
):
    world_states = []

    state = initial_robot_state
    for t in timestamps:
        [right_sensor_pos, left_sensor_pos] = sensor_positions(
            state, sensor_radius_from_center, sensor_angle_from_center
        )

        right_sensor_dist = track_model(
            right_sensor_pos[0],
            right_sensor_pos[1],
            ellipse_x,
            ellipse_y,
            ellipse_radius_a,
            ellipse_radius_b,
            thickness,
        )

        left_sensor_dist = track_model(
            left_sensor_pos[0],
            left_sensor_pos[1],
            ellipse_x,
            ellipse_y,
            ellipse_radius_a,
            ellipse_radius_b,
            thickness,
        )

        [right_vel, left_vel] = control_model(
            right_sensor_dist, left_sensor_dist, v_max, True
        )

        control_vec = control_vector(right_vel, left_vel, robot_width)

        state = vehicle_model(state, control_vec, dt)
        world_state = np.concatenate(
            (
                [t],
                state,
                control_vec,
                right_sensor_pos,
                left_sensor_pos,
                [right_sensor_dist, left_sensor_dist, right_vel, left_vel],
            ),
            axis=0,
        )

        world_states.append(world_state)

    return np.array(world_states)


if __name__ == "__main__":
    import math

    DT = 0.05
    INITIAL_STATE = np.zeros(3)
    ROBOT_LENGTH = 0.075
    ROBOT_WIDTH = 0.05
    SENSOR_RADIUS_FROM_CENTER = math.sqrt((ROBOT_LENGTH/2) ** 2 + (ROBOT_WIDTH/2) ** 2)
    SENSOR_ANGLE_FROM_CENTER = math.atan2(ROBOT_WIDTH / 2, ROBOT_LENGTH / 2)
    ELLIPSE_X = 0
    ELLIPSE_Y = 0.075
    ELLIPSE_RADIUS_A = 0.2
    ELLIPSE_RADIUS_B = 0.075
    ELLIPSE_THICKNESS = 0.015

    WHEEL_RADIUS = 0.018 # m
    WHEEL_ANGULAR_VELOCITY = 7.645 # rad
    V_MAX = WHEEL_RADIUS * WHEEL_ANGULAR_VELOCITY

    Xs = simulate(
        np.arange(0, 240, DT),
        DT,
        INITIAL_STATE,
        ROBOT_WIDTH,
        SENSOR_RADIUS_FROM_CENTER,
        SENSOR_ANGLE_FROM_CENTER,
        ELLIPSE_X,
        ELLIPSE_Y,
        ELLIPSE_RADIUS_A,
        ELLIPSE_RADIUS_B,
        ELLIPSE_THICKNESS,
        V_MAX
    )

    import matplotlib.pyplot as plt
    from matplotlib.patches import Ellipse

    # Xs = simulate(np.arange(0, 240, DT), DT, INITIAL_STATE, d, r, th, x0, y0, a, b, T, v_max)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

    ax1.plot(Xs[:, 1], Xs[:, 2], label='Robot Path', color='blue')
    # ax1.plot(Xs[:, 6], Xs[:, 7], label='Right Sensor Path', color='green')
    # ax1.plot(Xs[:, 8], Xs[:, 9], label='Left Sensor Pat', color='green')


    ellipse = Ellipse(xy=(ELLIPSE_X, ELLIPSE_Y), width=ELLIPSE_RADIUS_A * 2, height=ELLIPSE_RADIUS_B * 2, 
                            edgecolor='r', fc='None', lw=2, label="Target Path")

    ax1.add_patch(ellipse)

    ax1.set_title("Robot Trajectory (Bang-Bang Control)")
    ax1.set_xlabel("X Position (m)")
    ax1.set_ylabel("Y Position (m)")
    ax1.axis('equal') 
    ax1.grid(True)
    ax1.legend()

    ax2.plot(Xs[:, 0], Xs[:, 3] % (2 * math.pi), color='red', label="Heading")
    ax2.set_title("Vehicle Heading vs. Time")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Angle (rad)")
    ax2.grid(True)
    ax2.legend()

    plt.tight_layout()
    plt.show()
