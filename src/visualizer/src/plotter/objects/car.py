from matplotlib import transforms
from plotter.constants import *
import matplotlib.pyplot as plt
import os, rospkg

from plotter.objects.i_scene_2D_object import Scene2DObject

TRAFFIC_CAR_IMAGE_PATH = "src/plotter/img/red_car.png"
EGO_CAR_IMAGE_PATH = "src/plotter/img/blue_car.png"

class Car(Scene2DObject):
    
    def __init__(
        self, 
        x: float,
        y: float,
        length: float,
        width: float,
        heading_angle_deg: float = 0,
        is_ego: bool = False,
    ) -> None:
        
        super().__init__()
        
        # Position 
        self.x = x
        self.y = y
        
        # Geometry 
        self.length = length
        self.width = width
        
        # Orientation
        self.heading_angle_deg = heading_angle_deg
        
        self.image = self.__get_image_path(is_ego)
        
    def __get_image_path(self, is_ego):
        
        rospack = rospkg.RosPack()
        node_path = rospack.get_path("visualizer")
        
        if is_ego:
            return plt.imread(os.path.join(node_path, EGO_CAR_IMAGE_PATH))
        
        return plt.imread(os.path.join(node_path, TRAFFIC_CAR_IMAGE_PATH))
        
    
    def plot(self):
        
        if self.x > (self.scene.min_x + CAR_LENGTH/2) and self.x < (self.scene.max_x - CAR_LENGTH/2):
            # rotation animation
            tr = transforms.Affine2D().translate(-self.x, -self.y).rotate_deg(self.heading_angle_deg).translate(self.x, self.y)

            # NOTE: axis limits need to be set after this line
            self.scene.ax.imshow(
                self.image, 
                extent=[
                    self.x - CAR_LENGTH / 2, 
                    self.x + CAR_LENGTH / 2, 
                    self.y - CAR_WIDTH / 2, 
                    self.y + CAR_WIDTH / 2], 
                transform=tr + self.scene.ax.transData, 
                zorder=10
            )