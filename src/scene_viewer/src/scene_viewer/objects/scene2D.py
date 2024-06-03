from matplotlib.ticker import AutoMinorLocator, MultipleLocator
from scene_viewer.constants import *
import matplotlib.pyplot as plt

from scene_viewer.objects.i_scene_2D_object import Scene2DObject

class Scene2D:
    def __init__(
        self,
        ax: plt.Axes,
        min_x: float, 
        max_x: float,
        number_of_lanes: float,
        title: str = ""
    ) -> None:

        self.ax = ax
        self.min_x = min_x
        self.max_x = max_x
        self.number_of_lanes = number_of_lanes
        
        self.objects = []
        
        self.__initialize_axis(title)
        self.__add_lanes()
        
    def __initialize_axis(self, title,):
        """
        Initialize the axis to draw on: removes any previous content and sets the scale limits.
            @param ax: axis to plot to
            @param title: plot_title
        """
        self.ax.cla()

        self.ax.set_title(title)

        self.ax.xaxis.set_major_locator(MultipleLocator(MAJOR_X_TICKS))
        self.ax.yaxis.set_major_locator(MultipleLocator(MAJOR_Y_TICKS))
        self.ax.xaxis.set_minor_locator(AutoMinorLocator(MINOR_X_TICKS))
        self.ax.yaxis.set_minor_locator(AutoMinorLocator(MINOR_Y_TICKS))
        
        self.ax.grid(True, color="grey")
        
    def __reset_axis_size(self):
        
        self.ax.set_xlim(self.min_x, self.max_x)
        self.ax.set_ylim(-0.5 * LANE_WIDTH, (self.number_of_lanes + 0.5)*LANE_WIDTH)
        self.ax.axis("equal")
    
    def __add_lanes(self):
        
        for i in range(self.number_of_lanes + 1):
            lane_xs = [self.min_x, self.max_x]
            lane_ys = [i * LANE_WIDTH, i * LANE_WIDTH]
        
            self.ax.plot(
                lane_xs, 
                lane_ys, 
                color=LANE_COLOR,
                linestyle=LANE_STYLE, 
                linewidth=LANE_LINE_WIDTH, 
                zorder=3
            )
        
    def add(self, *new_objects: Scene2DObject):
        for obj in new_objects:
            self.objects.append(obj)
            obj.add_scene_info(self)
        
    def plot(self):
        for obj in self.objects:
            obj.plot()
            
        self.__reset_axis_size()
        