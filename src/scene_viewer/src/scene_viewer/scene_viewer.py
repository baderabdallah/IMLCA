import os, rospkg

from direct.gui.OnscreenText import OnscreenText
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from lane_msgs.msg import Mlc
from math import pi, sin, cos
from panda3d.core import AmbientLight, DirectionalLight, Spotlight
from panda3d.core import AntialiasAttrib, CullFaceAttrib, TransparencyAttrib, LightRampAttrib
from panda3d.core import GeomNode, TextNode, NodePath, Geom, GeomLines, GeomPoints, GeomTriangles, GeomVertexData, GeomVertexFormat, GeomVertexWriter
from panda3d.core import loadPrcFileData
from panda3d.core import Material, Texture
from panda3d.core import PNMImage, Fog
from panda3d.core import RenderState
from panda3d.core import Vec2, Vec3, Vec4, Quat, Mat4, BitMask32
from panda3d.core import WindowProperties
from scene_viewer.constants import *

from scene_viewer.objects.car import Car
from scene_viewer.objects.trajectory import Trajectory
from scene_viewer.objects.scene2D import Scene2D
from scene_viewer.helpers import *


#from panda3d_viewer.geometry import *


# class SceneViewer(Viewer):
#     def __init__(self):
#         self.node_base_path = rospkg.RosPack().get_path("scene_viewer")

#         config = ViewerConfig()
#         config.set_window_size(1600, 900)
#         config.enable_antialiasing(True, multisamples=4)

#         Viewer.__init__(self, window_type='onscreen', window_title=FIGURE_TITLE, config=config)

#         self.append_group('root')
#         self.reset_camera(pos=(0, 0, 15), look_at=(0, 0, 0))

loadPrcFileData("", "load-file-type p3assimp")

class SceneViewer(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
        
        properties = WindowProperties()
        properties.setSize(1600, 900)
        properties.setTitle(FIGURE_TITLE)
        
        self.win.requestProperties(properties)
        self._node_base_path = rospkg.RosPack().get_path("scene_viewer")
        self._textures_base_path = os.path.join(self._node_base_path, "src/scene_viewer/textures/")
        self._models_base_path = os.path.join(self._node_base_path, "src/scene_viewer/models/")

        self.disableMouse()

        self.__load_textures()
        self.camera.setPos(0, 0, 100)
        self.camera.lookAt((0, 0, 0), (0, 0, 1))
        self._scene_root = self.render.attachNewNode('SceneRoot')
        self._road = self._scene_root.attachNewNode('Road')

        self._van = self.loader.loadModel(os.path.join(self._models_base_path, "Van.glb"))
        self._van.reparentTo(self._scene_root)
        self._van.setPos(Vec3(0, 0, 0))
        # self._van.setP(90)
        self._van.setH(90)
        # self._van.setScale(Vec3(0.07, 0.07, 0.07))
        self._van.setScale(Vec3(2, 2, 2))
        



        self.taskMgr.add(self.__spinCameraTask, "SpinCameraTask")

    def __spinCameraTask(self, task):
        angleDegrees = task.time * 6.0
        angleRadians = angleDegrees * (pi / 180.0)
        self.camera.setPos(20 * sin(angleRadians), -20 * cos(angleRadians), 3)
        self.camera.setHpr(angleDegrees, 0, 0)
        return Task.cont
    
    def __create_plane(self, name, size=(1, 1), position=(0, 0, 0), rotation=None, texture_repeat=(1, 1)):
        """Create a plane node with at a given local position, with the supplied size and rotation.

            Arguments:
                name {str} -- node name
                size {Vec2} -- plane size
                position {Vec3} -- plane local position
                rotation {Quat} -- plane rotation

        """

        geom_node = GeomNode(name)
        geom_node.addGeom(self.__make_plane((1.0, 1.0), texture_repeat))
        node = NodePath(geom_node)
        node.setScale(Vec3(size[0], size[1], 1.0))
        node.setPos(Vec3(*position))

        if rotation is not None:
            node.setQuat(Quat(*rotation))

        return node

    def __create_road(self, msg):
        self._road.removeNode()
        self._road = self._scene_root.attachNewNode('Road')
        self._center_lane_y_positions = []
        
        road_chunk_size = Vec2(100, 5)
        #road_chunk_offset = (msg.scenario_data.number_of_lanes / 2) * road_chunk_size[1] - road_chunk_size[1] / 2
        road_chunk_offset = 0.5 * road_chunk_size[1] * (msg.scenario_data.number_of_lanes - 1)

        if msg.scenario_data.number_of_lanes == 1:
            road_chunk = self.__create_plane("single_lane_road", road_chunk_size, (0, 0, 0), None, (20, 1))
            road_chunk.setTexture(self.single_lane_road_chunk_texture, 1)
            road_chunk.reparentTo(self._road)
            self._center_lane_y_positions.append(0)

        elif msg.scenario_data.number_of_lanes > 1:
            upper_lane_road_chunk = self.__create_plane("upper_lane_road_chunk", road_chunk_size, Vec3(0, road_chunk_offset, 0), None, (20, 1))
            upper_lane_road_chunk.setTexture(self.upper_lane_road_chunk_texture, 1)
            upper_lane_road_chunk.reparentTo(self._road)
            self._center_lane_y_positions.append(road_chunk_offset)
            road_chunk_offset -= road_chunk_size[1]

            for i in range(msg.scenario_data.number_of_lanes - 2):
                middle_lane_road_chunk = self.__create_plane('middle_lane_chunk_' + str(i), road_chunk_size, Vec3(0, road_chunk_offset, 0), None, (20, 1))
                middle_lane_road_chunk.setTexture(self.middle_lane_road_chunk_texture, 1)
                middle_lane_road_chunk.reparentTo(self._road)
                self._center_lane_y_positions.append(road_chunk_offset)
                road_chunk_offset -= road_chunk_size[1]

            lower_lane_road_chunk = self.__create_plane("lower_lane_road_chunk", road_chunk_size, Vec3(0, road_chunk_offset, 0), None, (20, 1))
            lower_lane_road_chunk.setTexture(self.lower_lane_road_chunk_texture, 1)
            lower_lane_road_chunk.reparentTo(self._road)
            self._center_lane_y_positions.append(road_chunk_offset)
            # road_chunk_offset -= road_chunk_size[1]

    def __display_cars(self, msg):
        """ TODO """

    def __load_2d_texture(self, texture_file_name, texture_wrap_mode=(Texture.WM_repeat, Texture.WM_repeat), texture_filter=(Texture.FT_linear, Texture.FT_linear)):
        texture = self.loader.loadTexture(os.path.join(self._textures_base_path, texture_file_name))
        texture.setWrapU(texture_wrap_mode[0])
        texture.setWrapV(texture_wrap_mode[1])
        texture.setMagfilter(texture_filter[0])
        texture.setMinfilter(texture_filter[1])
        return texture

    def __load_textures(self):
        self.upper_lane_road_chunk_texture = self.__load_2d_texture("UpperLaneChunk.png", (Texture.WM_mirror, Texture.WM_mirror))
        self.middle_lane_road_chunk_texture = self.__load_2d_texture("MiddleLaneChunk.png", (Texture.WM_mirror, Texture.WM_mirror))
        self.lower_lane_road_chunk_texture = self.__load_2d_texture("LowerLaneChunk.png", (Texture.WM_mirror, Texture.WM_mirror))
        self.single_lane_road_chunk_texture = self.__load_2d_texture("SingleLaneChunk.png", (Texture.WM_mirror, Texture.WM_mirror))

    def update(self, msg):
        if hasattr(self, 'latest_msg'):
            if self.latest_msg.scenario_data.number_of_lanes != msg.scenario_data.number_of_lanes:
                self.__create_road(msg)
        else:
            self.__create_road(msg)
        
        self.__display_cars(msg)

        self.latest_msg = msg
        #plot_mlc ...
        """ TODO """

    def __make_plane(self, size=(1.0, 1.0), texture_repeat=(1, 1)):
        """Make a plane geometry.

        Arguments:
            size {tuple} -- plane size x,y

        Returns:
            Geom -- p3d geometry
        """
        vformat = GeomVertexFormat.get_v3n3c4t2()
        vdata = GeomVertexData('vdata', vformat, Geom.UHStatic)
        vdata.uncleanSetNumRows(4)

        vertex = GeomVertexWriter(vdata, 'vertex')
        normal = GeomVertexWriter(vdata, 'normal')
        color = GeomVertexWriter(vdata, 'color')
        tcoord = GeomVertexWriter(vdata, 'texcoord')

        quad = ((0, 0), (1, 0), (0, 1), (1, 1))
        
        for u, v in quad:
            vertex.addData3((u - 0.5) * size[0], (v - 0.5) * size[1], 0)
            normal.addData3(0, 0, 1)
            color.addData4(1, 1, 1, 1)
            tcoord.addData2(u * texture_repeat[0], v * texture_repeat[1])

        prim = GeomTriangles(Geom.UHStatic)
        prim.addVertices(0, 1, 2)
        prim.addVertices(2, 1, 3)

        geom = Geom(vdata)
        geom.addPrimitive(prim)

        return geom

###########################################################

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

def __make_title(scenario):
    timestamp = scenario.header.stamp
    return f"Frame #{scenario.header.seq}, time: {timestamp.secs}.{timestamp.nsecs}"

def __get_trajectory(mlc_message):
    trajectory = mlc_message.ego_vehicle_trajectory.trajectory
    
    if not trajectory:
        header = mlc_message.scenario_data.header
        timestamp = header.stamp
        print(
            f"* Message #{header.seq} contains an empty trajectory (at {timestamp.secs}.{timestamp.nsecs} seconds)")
        return []
    
    return trajectory
    
def build_ego_from_trajectory_data(trajectory):
    ego_x = trajectory[0].x
    ego_y = trajectory[0].y
    
    heading = calculate_angle(trajectory)
    print("heading:", heading)
    
    return Car(
        x=ego_x,
        y=ego_y,
        length=CAR_LENGTH,
        width=CAR_WIDTH,
        heading_angle_deg=heading,
        is_ego=True,
    )
    
def build_traffic_agents(vehicles_information, lane_numbers, scene):
     # Collect car position data
    xs = [vehicle.pos_x for vehicle in vehicles_information]
    ys = [get_y_coordinate_from_lane_number(lane_numbers, vehicle.lane_number) 
            for vehicle in vehicles_information]
            
    # Create cars objects
    return [
        Car(
            x=x,
            y=y,
            length=CAR_LENGTH,
            width=CAR_WIDTH,
            heading_angle_deg=0,
            is_ego=False,
        )
        for x, y in zip(xs, ys)
        ]
    

def plot_mlc(mlc_message, ax):

    (min_x, max_x) = get_min_max_x(mlc_message)
    
    scene = Scene2D(
        ax=ax,
        min_x=min_x,
        max_x=max_x,
        number_of_lanes=mlc_message.scenario_data.number_of_lanes,
        title=__make_title(mlc_message.scenario_data),
    )
    
    trajectory_data = __get_trajectory(mlc_message=mlc_message)    
    
    if len(trajectory_data):
        
        # Add trajectory to the scene
        trajectory = Trajectory(
            trajectory_data
        )
        scene.add(trajectory)
    
        # Add ego vehicle to the scene
        ego = build_ego_from_trajectory_data(trajectory_data)
        scene.add(ego)
    
    # Add other vehicles to the scene
    vehicles_list = build_traffic_agents(
            mlc_message.scenario_data.vehicles_information, 
            mlc_message.scenario_data.number_of_lanes,
            scene,
        )
    
    #for vehicle in vehicles_list:
    scene.add(*vehicles_list)
    
    # Draw the scene
    scene.plot()