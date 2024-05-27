from plotter.constants import *

from plotter.objects.i_scene_2D_object import Scene2DObject

class Trajectory(Scene2DObject):
    
    def __init__(self, trajectory):
        
        super().__init__()
        
        self.xs = [pose.x for pose in trajectory]
        self.ys = [pose.y for pose in trajectory]
    
    def plot(self):

        if len(self.xs) and len(self.ys):
            self.scene.ax.plot(
                self.xs, 
                self.ys, 
                color=TRAJECTORY_COLOR,
                linestyle=TRAJECTORY_STYLE, 
                linewidth=TRAJECTORY_WIDTH
            )