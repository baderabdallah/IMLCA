import json
import os, rospkg
import random

from direct.gui.OnscreenText import OnscreenText
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
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
from scene_viewer.helpers import *
from scene_viewer.vehicle_model_info import *

from scene_viewer.objects.car import Car
from scene_viewer.objects.trajectory import Trajectory
from scene_viewer.objects.scene2D import Scene2D


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

        self.camera.setPos(0, 0, 50)
        self.camera.lookAt((0, 0, 0), (0, 0, 1))
        self._scene_root = self.render.attachNewNode('SceneRoot')
        self._road = self._scene_root.attachNewNode('Road')
        self._cars = self._scene_root.attachNewNode('Cars')
        self._on_screen_cars = dict()
        self._on_screen_ego = None
        self._rebuild_road = True

        self.__load_3d_models()
        self.__load_textures()

        self.taskMgr.add(self.__update_scene, "Update Scene Task")

    #     self.taskMgr.add(self.__spinCameraTask, "SpinCameraTask")

    # def __spinCameraTask(self, task):
    #     angleDegrees = task.time * 6.0
    #     angleRadians = angleDegrees * (pi / 180.0)
    #     self.camera.setPos(20 * sin(angleRadians), -20 * cos(angleRadians), 3)
    #     self.camera.setHpr(angleDegrees, 0, 0)
    #     return Task.cont

    def __update_scene(self, task):
        if hasattr(self, 'latest_msg'):
            if self._rebuild_road:
                self._rebuild_road = False
                self.__create_road(self.latest_msg)
            else:
                self.__adjust_road(self.latest_msg)

            self.__display_and_update_cars(self.latest_msg)
            
        return Task.cont
    
    def __move_camera(self, height=50, target=Vec3()):
        self.camera.setPos(target.x, self._road_center_y_position, height)
        self.camera.lookAt((target.x, self._road_center_y_position, 0), (0, 0, 1))

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
        self._road_chunks = []
        self._road.removeNode()
        self._road = self._scene_root.attachNewNode('Road')
        self._road_chunks.append(self._road.attachNewNode('RoadChunk_0'))
        self._lane_center_y_positions = []
        self._road_center_y_position = 0
        self._current_road_chunk_index = 1
        
        self._road_chunk_size = Vec2(500, LANE_WIDTH)
        self._half_road_chunk_size = self._road_chunk_size * 0.5
        plane_texture_repeat = Vec2(self._road_chunk_size.x / 5.0, 1)
        
        if msg.ego_vehicle_trajectory.trajectory:
            x_coord = msg.ego_vehicle_trajectory.trajectory[0].x
        else:
            x_coord = 0

        if msg.scenario_data.number_of_lanes == 1:
            y_coord = get_y_coordinate_from_lane_number(1, 0, self._road_chunk_size.y)
            road_chunk = self.__create_plane("single_lane_road", self._road_chunk_size, (0, y_coord, 0), None, plane_texture_repeat)
            road_chunk.setTexture(self.single_lane_road_chunk_texture, 1)
            road_chunk.reparentTo(self._road_chunks[0])
            self._lane_center_y_positions.append(y_coord)
            self._road_center_y_position = y_coord

        elif msg.scenario_data.number_of_lanes > 1:
            y_coord = get_y_coordinate_from_lane_number(msg.scenario_data.number_of_lanes, 0, self._road_chunk_size.y)
            self._road_center_y_position = y_coord - self._road_chunk_size.y * 0.5
            upper_lane_road_chunk = self.__create_plane("upper_lane_road_chunk", self._road_chunk_size, Vec3(0, y_coord, 0), None, plane_texture_repeat)
            upper_lane_road_chunk.setTexture(self.upper_lane_road_chunk_texture, 1)
            upper_lane_road_chunk.reparentTo(self._road_chunks[0])
            self._lane_center_y_positions.append(y_coord)

            for i in range(msg.scenario_data.number_of_lanes - 2):
                y_coord = get_y_coordinate_from_lane_number(msg.scenario_data.number_of_lanes, i + 1, self._road_chunk_size.y)
                middle_lane_road_chunk = self.__create_plane('middle_lane_chunk_' + str(i), self._road_chunk_size, Vec3(0, y_coord, 0), None, plane_texture_repeat)
                middle_lane_road_chunk.setTexture(self.middle_lane_road_chunk_texture, 1)
                middle_lane_road_chunk.reparentTo(self._road_chunks[0])
                self._lane_center_y_positions.append(y_coord)

            y_coord = get_y_coordinate_from_lane_number(msg.scenario_data.number_of_lanes, msg.scenario_data.number_of_lanes - 1, self._road_chunk_size.y)
            self._road_center_y_position = (self._road_center_y_position + (y_coord + self._road_chunk_size.y * 0.5)) * 0.5
            lower_lane_road_chunk = self.__create_plane("lower_lane_road_chunk", self._road_chunk_size, Vec3(0, y_coord, 0), None, plane_texture_repeat)
            lower_lane_road_chunk.setTexture(self.lower_lane_road_chunk_texture, 1)
            lower_lane_road_chunk.reparentTo(self._road_chunks[0])
            self._lane_center_y_positions.append(y_coord)
        
        self._road_chunks[0].setPos(Vec3(x_coord - self._road_chunk_size.x, 0, 0))

        for chunk_index in range(1, 3):
            prev_pos = self._road_chunks[chunk_index - 1].getPos()
            self._road_chunks.append(self._road_chunks[0].copyTo(self._road))
            self._road_chunks[chunk_index].setName("RoadChunk_" + str(chunk_index))
            self._road_chunks[chunk_index].setPos(Vec3(prev_pos.x + self._road_chunk_size.x, prev_pos.y, prev_pos.z))

    def __adjust_road(self, msg):
        if msg.ego_vehicle_trajectory.trajectory and msg.scenario_data.number_of_lanes > 0:
            x_coord = msg.ego_vehicle_trajectory.trajectory[0].x
            road_chunk_limit_x = self._road_chunks[self._current_road_chunk_index].getPos().x + self._half_road_chunk_size.x

            if x_coord >= road_chunk_limit_x:
                next_road_chunk_index = self.__get_next_road_chunk_index()
                previous_road_chunk_index = self.__get_previous_road_chunk_index()
                new_road_chunk_pos = self._road_chunks[next_road_chunk_index].getPos()
                new_road_chunk_pos.x += self._road_chunk_size.x
                self._road_chunks[previous_road_chunk_index].setPos(new_road_chunk_pos)
                self._current_road_chunk_index = next_road_chunk_index

    def __get_next_road_chunk_index(self):
        return (self._current_road_chunk_index + 1) % len(self._road_chunks)

    def __get_previous_road_chunk_index(self):
        i = self._current_road_chunk_index - 1

        if i < 0:
            i += len(self._road_chunks)

        return i

    def __display_and_update_cars(self, msg):
        if not msg.scenario_data.vehicles_information:
            self._cars.removeNode()
            self._cars = self._scene_root.attachNewNode('Cars')
            self._on_screen_cars.clear()

        received_car_ids = dict()
        cars_to_remove = []

        if not self._on_screen_ego:
            self._on_screen_ego = self._ego.copyTo(self._cars)

        ego_trajectory = self.__get_trajectory(msg)

        if ego_trajectory:
            ego_position = Vec3(ego_trajectory[0].x, ego_trajectory[0].y, 0)
            ego_rotation_yaw = calculate_angle(ego_trajectory)
            self._on_screen_ego.reparentTo(self._cars)
            self._on_screen_ego.setPos(ego_position)
            self._on_screen_ego.setH(self._ego.getH() + ego_rotation_yaw)
            self.__move_camera(60, ego_position)
        else:
            self._on_screen_ego.detachNode()

        for v in msg.scenario_data.vehicles_information:
            received_car_ids[v.id] = v.id
            car = self._on_screen_cars.get(v.id)

            if not car:
                car = self._vehicle_pool[random.randrange(0, len(self._vehicle_pool))].copyTo(self._cars)
                self._on_screen_cars[v.id] = car

            car.setPos(Vec3(v.pos_x, self._lane_center_y_positions[v.lane_number], 0))
        
        for k in self._on_screen_cars.keys():
            if not received_car_ids.get(k):
                self._on_screen_cars.get(k).removeNode()
                cars_to_remove.append(k)

        for c in cars_to_remove:
            self._on_screen_cars.pop(c)

        # uint32 lane_number
        # uint32 id
        # float64 pos_x
        # float64 velocity_x
        # print("ON SCREEN CARS = " + str(len(self._on_screen_cars)) + ", INFO = " + str(len(msg.scenario_data.vehicles_information)))

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

    def __load_3d_model(self, model_info):
        model_node = self.loader.loadModel(os.path.join(self._models_base_path, model_info.fileName))
        model_node.setPos(Vec3(0, 0, 0))
        model_node.setR(model_info.rotation.roll)
        model_node.setP(model_info.rotation.pitch)
        model_node.setH(model_info.rotation.yaw)
        model_node.setScale(model_info.scale)
        return model_node

    def __load_3d_models(self):
        ego, vehicles = self.__load_vehicles_info()
        
        self._ego = self.__load_3d_model(ego)
        self._vehicle_pool = []

        for v in vehicles:
            self._vehicle_pool.append(self.__load_3d_model(v))

    def __load_vehicles_info(self):
        with open(os.path.join(self._models_base_path, "vehicles.json")) as json_file:
            vehicle_json_data = json.load(json_file)

            ego = VehicleModelInfo()
            vehicles = []

            ego_json = vehicle_json_data["Ego"]
            ego.fileName = ego_json["fileName"]
            ego.rotation.roll = float(ego_json["rotation"]["roll"])
            ego.rotation.pitch = float(ego_json["rotation"]["pitch"])
            ego.rotation.yaw = float(ego_json["rotation"]["yaw"])
            ego.scale = Vec3(float(ego_json["scale"]["x"]), float(ego_json["scale"]["y"]), float(ego_json["scale"]["z"]))

            for vehicle_json in vehicle_json_data["Vehicles"]:
                v = VehicleModelInfo()
                v.fileName = vehicle_json["fileName"]
                v.rotation.roll = float(vehicle_json["rotation"]["roll"])
                v.rotation.pitch = float(vehicle_json["rotation"]["pitch"])
                v.rotation.yaw = float(vehicle_json["rotation"]["yaw"])
                v.scale = Vec3(float(vehicle_json["scale"]["x"]), float(vehicle_json["scale"]["y"]), float(vehicle_json["scale"]["z"]))
                vehicles.append(v)
            
            return [ego, vehicles]
    
    def update(self, msg):
        if hasattr(self, 'latest_msg'):
            self._rebuild_road = (self.latest_msg.scenario_data.number_of_lanes != msg.scenario_data.number_of_lanes)
        
        self.latest_msg = msg

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

    def __get_trajectory(self, mlc_message):
        trajectory = mlc_message.ego_vehicle_trajectory.trajectory
        
        if not trajectory:
            header = mlc_message.scenario_data.header
            timestamp = header.stamp
            print(
                f"* Message #{header.seq} contains an empty trajectory (at {timestamp.secs}.{timestamp.nsecs} seconds)")
            return []
        
        return trajectory

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