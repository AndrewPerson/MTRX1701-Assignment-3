# function: ellipse_level_set_distance
# inputs:
#   x - x coord of point to test
#   y - y coord of point to test
#   ellipse_x - x position of centre of ellipse
#   ellipse_y - y position of centre of ellipse
#   a - major axis of ellipse
#   b - minor axis of ellipse
# return:
#   < 0 if point is inside ellipse
#   0 if point is on ellipse
#   > 0 if point is outside of ellipse
def ellipse_level_set_distance(x: float, y: float, ellipse_x: float, ellipse_y: float, a: float, b: float) -> float:
    return ((x - ellipse_x) / a) ** 2 + ((y - ellipse_y) / b) ** 2 - 1

# function: classify_ellipse_point
# inputs:
#   x - x coord of point to test
#   y - y coord of point to test
#   ellipse_x - x position of centre of ellipse
#   ellipse_y - y position of centre of ellipse
#   a - major axis of ellipse
#   b - minor axis of ellipse
# return:
#   < 0 if point is inside ellipse
#   0 if point is on ellipse
#   > 0 if point is outside of ellipse
def classify_ellipse_point(
    x: float, y: float, ellipse_x: float, ellipse_y: float, a: float, b: float, thickness: float
) -> float:
    dist_inner = ellipse_level_set_distance(
        x, y, ellipse_x, ellipse_y, a - thickness / 2, b - thickness / 2
    )
    dist_outer = ellipse_level_set_distance(
        x, y, ellipse_x, ellipse_y, a + thickness / 2, b + thickness / 2
    )

    if dist_inner > 0 and dist_outer < 0:
        return 0
    elif dist_outer > 0:
        return dist_outer
    else:
        return dist_inner


# function: track_model
# inputs:
#   x - x position of sensor
#   y - y position of sensor
#   ellipse_x - x position of centre of ellipse
#   ellipse_y - y position of centre of ellipse
#   a - major axis of ellipse
#   b - minor axis of ellipse
#   thickness - thickness of ellipse
# return:
#   < 0 if point is inside ellipse of thickness T
#   0 if point is on ellipse of thickness T
#   > 0 if point is outside of ellipse of thickness T
def track_model(
    x: float,
    y: float,
    ellipse_x: float,
    ellipse_y: float,
    a: float,
    b: float,
    thickness: float,
) -> float:
    return classify_ellipse_point(x, y, ellipse_x, ellipse_y, a, b, thickness)


if __name__ == "__main__":
    import numpy as np
    import matplotlib.pyplot as plt

    def points_on_rect(
        width: float,
        height: float,
        x: float,
        y: float,
        x_density: int | None = None,
        y_density: int | None = None,
    ):
        x_density = int(width * 10) if x_density is None else x_density
        y_density = int(height * 10) if y_density is None else y_density

        points = np.dstack(
            np.meshgrid(
                np.linspace(x - width / 2, x + width / 2, x_density),
                np.linspace(y - height / 2, y + height / 2, y_density),
            )
        ).reshape(
            -1, 2
        )  # From https://stackoverflow.com/a/11146645

        return points

    points = points_on_rect(0.5, 0.3, 0, 0.075, x_density=200, y_density=120)

    distances = list(map(lambda p: abs(classify_ellipse_point(p[0], p[1], 0, 0.075, 0.125, 0.075, 0)), points))
    min_distance = min(distances)
    max_distance = max(distances)

    colors = list(map(lambda d: (d + min_distance) / (min_distance + max_distance), distances))

    plt.scatter(points[:, 0], points[:, 1], c=colors)
    plt.axis("equal")
    plt.show()

    plt.clf()

    distances = list(map(lambda p: classify_ellipse_point(p[0], p[1], 0, 0.075, 0.125, 0.075, 0.015), points))
    colors = list(map(lambda d: 0 if d == 0 else 1, distances))

    plt.scatter(points[:, 0], points[:, 1], c=colors)
    plt.axis("equal")
    plt.show()
