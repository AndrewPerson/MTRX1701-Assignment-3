import numpy as np
from sensors import sensor_positions
from track_model import track_model
from control_model import control_model
from vehicle_model import vehicle_model, control_vector


def simulate(
    timestamps,
    dt,
    initial_robot_state,
    robot_width,
    sensor_radius_from_center,
    sensor_angle_from_center,
    ellipse_x,
    ellipse_y,
    ellipse_radius_a,
    ellipse_radius_b,
    thickness,
    v_max,
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
            right_sensor_dist, left_sensor_dist, v_max
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
    DT = 0.05
    INITIAL_STATE = np.zeros(3)
    ROBOT_WIDTH = 0.025
    SENSOR_RADIUS_FROM_CENTER = 0.07905694150420949
    SENSOR_ANGLE_FROM_CENTER = 0.32175055439664224
    ELLIPSE_X = 0
    ELLIPSE_Y = 0.075
    ELLIPSE_RADIUS_A = 0.2
    ELLIPSE_RADIUS_B = 0.075
    ELLIPSE_THICKNESS = 0.015
    V_MAX = 0.00625

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

    plt.figure(figsize=(8, 6))
    plt.plot(Xs[:, 1], Xs[:, 2], label="Robot Trajectory (Bang-Bang)")

    # plt.plot(Xs[:, 0], Xs[:, 7])
    # plt.plot(Xs[:, 0], Xs[:, 8])

    plt.title("Line Following Robot Trajectory")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.grid(True)
    plt.legend()
    plt.axis("equal")

    ellipse = Ellipse(xy=(ELLIPSE_X, ELLIPSE_Y), width=ELLIPSE_RADIUS_A * 2, height=ELLIPSE_RADIUS_B * 2, 
                            edgecolor='r', fc='None', lw=2)

    plt.gca().add_patch(ellipse)

    plt.show()
