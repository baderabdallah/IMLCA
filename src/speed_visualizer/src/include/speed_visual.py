import numpy as np
import matplotlib.pyplot as plt

from include.constants import *


class SpeedVisual:
    def __init__(self):
        self._fig, self._ax = plt.subplots(
            1,
            1,
            figsize=(FIGURE_WIDTH, FIGURE_HEIGHT),
            subplot_kw={"projection": "polar"},
        )
        self._fig.suptitle(FIGURE_TITLE)

        self.reset_axis()

    def reset_axis(
        self,
    ):
        self._ax.set_aspect("auto")
        self._ax.grid(False)
        self._ax.set_yticklabels([])
        self._ax.set_xticklabels([])
        self._ax.set_theta_zero_location("W")
        self._ax.set_theta_direction(-1)

    def draw_speedometer_visual(self, current_speed):
        self._ax.cla()
        ranges = [
            (0, int(MAX_SPEED / 3), "green"),
            (int(MAX_SPEED / 3), int(2 * MAX_SPEED / 3), "yellow"),
            (int(2 * MAX_SPEED / 3), MAX_SPEED, "red"),
        ]

        for start, end, color in ranges:
            angles = np.linspace(np.deg2rad(start), np.deg2rad(end), 100)
            self._ax.fill_between(angles, 0, 10, color=color, alpha=0.6)

        angles = np.linspace(0, np.pi, 100)
        self._ax.plot(angles, [10] * 100, color="black")

        for angle in np.linspace(0, np.pi, 10):
            self._ax.plot([angle, angle], [9, 10], color="black")
            self._ax.text(
                angle, 11, f"{int(MAX_SPEED * angle / np.pi)}", ha="center", va="center"
            )

        # Convert speed to angle
        angle = current_speed * np.pi / MAX_SPEED
        self._ax.plot([0, angle], [0, 8], color="black", lw=3)

        # Create text visual
        self._ax.text(
            -np.pi / 2,
            3,
            f"{current_speed} km/h",
            ha="center",
            va="center",
            fontsize=16,
            style="italic",
        )

        self.reset_axis()
