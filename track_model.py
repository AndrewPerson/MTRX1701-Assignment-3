# function: classify_ellipse_point
# inputs:
#   x - x position of sensor
#   y - y position of sensor
#   ellipse_x - x position of centre of ellipse
#   ellipse_y - y position of centre of ellipse
#   a - major axis of ellipse
#   b - minor axis of ellipse
# return:
#   < 0 if point is inside ellipse
#   0 if point is on ellipse
#   > 0 if point is outside of ellipse
def classify_ellipse_point(
    x: float, y: float, ellipse_x: float, ellipse_y: float, a, b
):
    return ((x - ellipse_x) / a) ** 2 + ((y - ellipse_y) / b) ** 2 - 1


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
    dist_inner = classify_ellipse_point(
        x, y, ellipse_x, ellipse_y, a - thickness / 2, b - thickness / 2
    )
    dist_outer = classify_ellipse_point(
        x, y, ellipse_x, ellipse_y, a + thickness / 2, b + thickness / 2
    )

    if dist_inner > 0 and dist_outer < 0:
        return 0
    elif dist_outer > 0:
        return dist_outer
    else:
        return dist_inner
