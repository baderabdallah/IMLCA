"""
@file scenario_plottyer.py
@copyright Copyright (C) 2021 BMW Hackaton Team1
@brief This file contains functions to plot the content of an MLC message on a given axis.
"""

from matplotlib import transforms
from matplotlib.ticker import AutoMinorLocator, MultipleLocator
from plotter.constants import *
import math
import matplotlib.pyplot as plt
import os, rospkg


def get_y_coordinate_from_lane_number(scenario, lane_number):
    """
    Transforms a lane number into a valid Y coordinate, assuming that the vehicle is located at the center of the lane
        @param scenario: scenario data
        @param lane_number: Lane_number starting from 0 to (NUMBER_OF_LANES-1)
    """
    return ((scenario.number_of_lanes - lane_number - 0.5) * LANE_WIDTH)


def get_min_max_x(mlc_message):
    """
    Retrieves the first and last X coordinate of the road section which has be visualized.
    The aim is to retrieve [ego_x - LOOK_BEHIND, last_ego_trajectory_x + LOOK_AHEAD].
    The information
        @param mlc_message: the received MLC message
        @return: the minimum and maximum relevant X coordinates in the current scenario.
    """

    min_x = -LOOK_BEHIND
    max_x = LOOK_AHEAD

    if mlc_message.ego_vehicle_trajectory.trajectory:
        # Ego pos
        ego_x = mlc_message.ego_vehicle_trajectory.trajectory[0].x
        min_x = ego_x - LOOK_BEHIND
        max_x = ego_x + LOOK_AHEAD

    elif mlc_message.scenario_data.vehicles_information:
        xs = []
        for vehicle in mlc_message.scenario_data.vehicles_information:
            xs.append(vehicle.pos_x)
        min_vehicles_x = min(xs)
        max_vehicles_x = max(xs)

        min_x = min_vehicles_x - LOOK_BEHIND
        max_x = max_vehicles_x + LOOK_BEHIND

    return (min_x, max_x)


def plot_lanes(number_of_lanes, min_x, max_x, ax: plt.Axes):
    """
    Plots the lanes for a given scenario.
        @param number_of_lanes: the number of lanes
        @param min_x: the first X coordinate of the road section to be visualized
        @param max_x: the last X coordinate of the road section to be visualized
        @param ax: axis to plot to
    """
    for i in range(number_of_lanes + 1):
        lane_xs = [min_x, max_x]
        lane_ys = [i * LANE_WIDTH, i * LANE_WIDTH]
        ax.plot(lane_xs, lane_ys, color=LANE_COLOR,
                linestyle=LANE_STYLE, linewidth=LANE_LINE_WIDTH, zorder=3)


def plot_car(ax, x, y, heading, is_ego, ):
    """
    Plots a single car to an axis.
        @param x, y: car coordinates
        @param heading: yaw (in degrees)
        @param color: color to use
        @param ax: axis to plot to
    """

    rospack = rospkg.RosPack()
    node_path = rospack.get_path("visualizer")
    if is_ego:
        image = plt.imread(os.path.join(node_path, "src/plotter/img/red_car.png"))

    else:
        image = plt.imread(os.path.join(node_path, "src/plotter/img/blue_car.png"))

    # rotation animation
    tr = transforms.Affine2D().translate(-x, -y).rotate_deg(heading).translate(x, y)

    # NOTE: axis limits need to be set after this line
    ax.imshow(image, extent=[x - CAR_LENGTH / 2, x + CAR_LENGTH / 2, y - CAR_WIDTH / 2, y + CAR_WIDTH / 2], transform=tr + ax.transData, zorder=10)


def plot_vehicles(scenario, min_x, max_x, ax):
    """
    Plots vehicles in a given scenario.
        @param scenario: scenario data
        @param min_x: the first X coordinate of the road section to be visualized
        @param max_x: the last X coordinate of the road section to be visualized
        @param ax: axis to plot to
    """
    # Collect car position data
    xs = []
    ys = []
    for vehicle in scenario.vehicles_information:
        if vehicle.pos_x > (min_x + CAR_LENGTH/2) and vehicle.pos_x < (max_x - CAR_LENGTH/2):
            xs.append(vehicle.pos_x)
            ys.append(get_y_coordinate_from_lane_number(
                scenario, vehicle.lane_number))

    # Draw the cars
    for car_id, (x, y) in enumerate(zip(xs, ys)):
        plot_car(ax, x, y, 0.0, False)


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

    print(f"(dx, dy): {dx}, {dy}")
    return math.degrees(math.atan2(dy, dx))


def plot_trajectory(trajectory, ax):
    """
    Plots the ego car with the given trajectory.
        @param trajectory: trajectory data
        @param ax: axis to plot to
    """
    xs = []
    ys = []
    for pose in trajectory:
        xs.append(pose.x)
        ys.append(pose.y)

    ax.plot(xs, ys, color=TRAJECTORY_COLOR,
            linestyle=TRAJECTORY_STYLE, linewidth=TRAJECTORY_WIDTH)


def plot_ego(mlc_message, ax):
    """
    Plots the ego car with the given trajectory.
        @param mlc_message: MLC message
        @param ax: axis to plot to
    """
    trajectory = mlc_message.ego_vehicle_trajectory.trajectory
    if not trajectory:
        header = mlc_message.scenario_data.header
        timestamp = header.stamp
        print(
            f"* Message #{header.seq} contains an empty trajectory (at {timestamp.secs}.{timestamp.nsecs} seconds)")
        return

    plot_trajectory(trajectory, ax)

    ego_x = trajectory[0].x
    ego_y = trajectory[0].y
    heading = calculate_angle(trajectory)
    print("heading:", heading)
    plot_car(ax, ego_x, ego_y, heading, True)


def initialize_axis(ax, scenario,):
    """
    Initialize the axis to draw on: removes any previous content and sets the scale limits.
        @param ax: axis to plot to
        @param mlc_message: MLC message
        @param min_x: the first X coordinate of the road section to be visualized
        @param max_x: the last X coordinate of the road section to be visualized
    """
    ax.cla()

    timestamp = scenario.header.stamp
    ax.set_title(
        f"Frame #{scenario.header.seq}, time: {timestamp.secs}.{timestamp.nsecs}")

    ax.xaxis.set_major_locator(MultipleLocator(MAJOR_X_TICKS))
    ax.yaxis.set_major_locator(MultipleLocator(MAJOR_Y_TICKS))
    ax.xaxis.set_minor_locator(AutoMinorLocator(MINOR_X_TICKS))
    ax.yaxis.set_minor_locator(AutoMinorLocator(MINOR_Y_TICKS))
    ax.grid(True, color="grey")

def reset_axis_size(ax, scenario, min_x, max_x):
    ax.set_xlim(min_x, max_x)
    ax.set_ylim(-0.5 * LANE_WIDTH, (scenario.number_of_lanes + 0.5)*LANE_WIDTH)
    ax.axis("equal")


# Main entry point
def plot_mlc(mlc_message, ax):
    """
    Plots the content of a MultiLaneChange message.
        @param mlc_message: MLC message
        @param ax: axis to plot to
    """
    (min_x, max_x) = get_min_max_x(mlc_message)
    initialize_axis(ax, mlc_message.scenario_data)

    # Add lanes and agents
    plot_lanes(mlc_message.scenario_data.number_of_lanes, min_x, max_x, ax)
    plot_vehicles(mlc_message.scenario_data, min_x, max_x, ax)
    plot_ego(mlc_message, ax)

    # Ensure plot ratio and avoid flickering
    reset_axis_size(ax, mlc_message.scenario_data, min_x, max_x)
