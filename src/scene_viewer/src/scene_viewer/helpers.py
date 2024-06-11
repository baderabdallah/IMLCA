from scene_viewer.constants import *
import math

def get_y_coordinate_from_lane_number(number_of_lanes, lane_number, lane_width=LANE_WIDTH):
    """
    Transforms a lane number into a valid Y coordinate, assuming that the vehicle is located at the center of the lane
        @param scenario: scenario data
        @param lane_number: Lane_number starting from 0 to (NUMBER_OF_LANES - 1)
    """
    return ((number_of_lanes - lane_number - 0.5) * lane_width)

def calculate_angle(trajectory):
    """
    Calculates the angle of the ego car heading by taking the first two points of the trajectory in consideration.
        @param trajectory: trajectory data
    """
    if len(trajectory) < 2:
        return 0.0

    point0 = trajectory[0]
    point1 = trajectory[1]
    dx = point1.x - point0.x
    dy = point1.y - point0.y

    # print(f"(dx, dy): {dx}, {dy}")

    return math.degrees(math.atan2(dy, dx))
