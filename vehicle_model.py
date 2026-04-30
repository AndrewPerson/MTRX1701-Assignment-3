import numpy as np


# function: derivatives
# inputs:
#   robot_state - robot state [x, y, heading]
#   control_vector - control inputs [velocity, angular velocity]
# returns:
#   vector of state derivatives [x velocity, y velocity, angular velocity]
def derivatives(robot_state, control_vector):
    heading = robot_state[2]
    vel = control_vector[0]

    x = vel * np.cos(heading)
    y = vel * np.sin(heading)

    angular_vel = control_vector[1]

    return np.array([x, y, angular_vel])


# function: vehicle_model
# inputs:
#   robot_state - robot state at current time [x, y, heading]
#   control_vector - control input [velocity, heading]
#   dt - time since last update (s)
# returns:
#   robot state dt seconds into the future [x, y, heading]
def vehicle_model(robot_state, control_vector, dt: float):
    deriv = derivatives(robot_state, control_vector)
    return robot_state + dt * deriv


# function: control_vector
# inputs:
#   right_vel - velocity of the right wheel (ms^-1)
#   left_vel - velcocity of the left wheel (ms^-1)
#   robot_width - width of robot (m)
# returns:
#   vector of control inputs [velocity, heading]
def control_vector(right_vel: float, left_vel: float, robot_width: float):
    v = (right_vel + left_vel) / 2
    heading = (right_vel - left_vel) / (2 * robot_width)

    return np.array([v, heading])


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    def plot_path(
        right_vel: float,
        left_vel: float,
        start_time: float = 0,
        end_time: float = 10,
        dt: float = 0.01,
        robot_width: float = 0.05,
    ):
        path: list[float] = []

        state = np.array([0.0, 0.0, 0.0])
        for _ in np.arange(start_time, end_time, dt):
            path.append(state)
            state = vehicle_model(
                state, control_vector(right_vel, left_vel, robot_width), dt
            )

        path = np.array(path)

        plt.plot(path[:, 0], path[:, 1])
        plt.xlabel("x (m)")
        plt.ylabel("y (m)")
        plt.axis("equal")

    plot_path(0.1, 0.1)

    plt.title("2c: Path with Equal Wheel Speeds")
    plt.show()
    plt.savefig("path_evolution_2c.png")

    plt.clf()

    v_max = 0.1
    plot_path(v_max, v_max * 0.5)

    plt.title("2d: Path with Full Speed vs Half Speed")
    plt.show()
    plt.savefig("assignment3_2d_plot.png")

    # In 2c:(equal wheel speed)when both the left and the right wheels were driven at the same velocity, the resulting path is a straight line.Mathematically, the vehicle model is calculated as psi_dot = (vr - vl) / (2 * d).So when the velocities are equal the numerator becomes 0 resulting in 0 turn rate.
    # In 2d: (Different wheel speed)When the wheels are with different speeds, (one with full speed and another with half the speed) the robot follows a circular path.In this case, vr is not equal to vl.
