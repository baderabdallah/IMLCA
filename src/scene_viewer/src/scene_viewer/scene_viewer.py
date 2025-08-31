import json
import os
import random
import rospkg
import rospy

from direct.directtools.DirectGeometry import LineNodePath
from direct.gui.DirectGui import *
from direct.gui.OnscreenText import OnscreenText
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from math import pi, sin, cos
from panda3d.core import AmbientLight, DirectionalLight, Spotlight
from panda3d.core import (
    AntialiasAttrib,
    CullFaceAttrib,
    TransparencyAttrib,
    LightRampAttrib,
)
from panda3d.core import (
    GeomNode,
    TextNode,
    NodePath,
    Geom,
    GeomLines,
    GeomPoints,
    GeomTriangles,
    GeomVertexData,
    GeomVertexFormat,
    GeomVertexWriter,
)
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

# Ensure Panda3D uses an X11-capable display in the container. Do this
# before creating ShowBase so the window opens over XQuartz on macOS.
# NOTE: This is critical for Dockerized GUI apps on macOS.
# The `load-display` setting tells Panda3D which graphics backend to use.
# We prioritize `p3tinydisplay` (software renderer) as it is more robust
# in environments without hardware GL, like Docker. `pandagl` is the
# hardware-accelerated fallback.
os.environ.setdefault("DISPLAY", ":0")
loadPrcFileData(
    "",
    "load-display p3tinydisplay\n"
    "aux-display pandagl\n"
    "win-origin -2 -2\n"  # Hack to prevent window manager decoration issues
)

ego_screen_position_offset = -27
camera_position_z = 70


class SceneViewer(ShowBase):
    def __init__(self):
        try:
            ShowBase.__init__(self)
        except Exception as e:
            rospy.logerr(f"[Scene Viewer] Panda3D ShowBase failed to initialize: {e}")
            rospy.logerr(
                "[Scene Viewer] This is a critical error, meaning the application could not create a graphics window."
            )
            rospy.logerr(
                "[Scene Viewer] Common causes and solutions for macOS Docker users:\n"
                "1. XQuartz Not Running: Ensure XQuartz is installed and running on your Mac.\n"
                "2. 'Allow connections from network clients' is NOT checked in XQuartz > Settings > Security. Please uncheck it.\n"
                "3. IP Address Mismatch: The script automatically detects your IP, but it might be wrong. Verify with `ifconfig | grep 'inet '`.\n"
                "4. Firewall Issues: A firewall might be blocking the connection to the X server."
            )
            self._test_x11_connection()
            raise

    def _test_x11_connection(self):
        """Attempt to create a simple tkinter window to test the X11 connection."""
        rospy.loginfo("[Scene Viewer] Attempting to open a simple test window with tkinter...")
        try:
            import tkinter as tk

            root = tk.Tk()
            root.title("X11 Test")
            tk.Label(root, text="If you see this, X11 forwarding is working.").pack()
            root.after(3000, root.destroy)  # Close after 3 seconds
            root.mainloop()
            rospy.loginfo(
                "[Scene Viewer] tkinter test window opened and closed successfully. The X11 connection seems OK."
            )
        except Exception as tk_e:
            rospy.logerr(f"[Scene Viewer] tkinter test failed: {tk_e}")
            rospy.logerr(
                "[Scene Viewer] The basic X11 connection test failed. This confirms a problem with the DISPLAY setup or XQuartz."
            )

        properties = WindowProperties()
        properties.setSize(WINDOW_WIDTH, WINDOW_HEIGHT)
        properties.setTitle(FIGURE_TITLE)

        self.win.requestProperties(properties)
        self._node_base_path = rospkg.RosPack().get_path("scene_viewer")
        self._textures_base_path = os.path.join(
            self._node_base_path, "src/scene_viewer/textures/"
        )
        self._models_base_path = os.path.join(
            self._node_base_path, "src/scene_viewer/models/"
        )

        self.disableMouse()

        self.camera.setPos(0, 0, 50)
        self.camera.lookAt((0, 0, 0), (0, 0, 1))
        self._scene_root = self.render.attachNewNode("SceneRoot")
        self._road = self._scene_root.attachNewNode("Road")
        self._cars = self._scene_root.attachNewNode("Cars")
        self._on_screen_cars = dict()
        self._on_screen_ego = None
        self._on_screen_ego_trajectory = LineNodePath(
            parent=self._cars,
            name="Ego Trajectory",
            thickness=1.0,
            colorVec=Vec4(1, 0, 0, 1),
        )
        self._rebuild_road = True
        self._initialize_gui = True

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
        if self._initialize_gui and self.__can_initialize_gui():
            self._initialize_gui = False
            self.__init_gui()

        if hasattr(self, "latest_msg"):
            # Ensure road is initialized before adjusting
            if self._rebuild_road or not hasattr(self, "_road_chunks"):
                self._rebuild_road = False
                self.__create_road(self.latest_msg)
            else:
                self.__adjust_road(self.latest_msg)

            self.__display_and_update_cars(self.latest_msg)

            if not self._initialize_gui:
                self.__update_gui(self.latest_msg)
                if hasattr(self, "kpi_msg"):
                    self.__update_gui_kpi(self.kpi_msg)

        return Task.cont

    def __move_camera(self, height=50, target=Vec3()):
        self.camera.setPos(target.x - ego_screen_position_offset, self._road_center_y_position, height)
        self.camera.lookAt((target.x - ego_screen_position_offset, self._road_center_y_position, 0), (0, 0, 1))

    def __create_plane(
        self,
        name,
        size=(1, 1),
        position=(0, 0, 0),
        rotation=None,
        texture_repeat=(1, 1),
    ):
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
        self._road = self._scene_root.attachNewNode("Road")
        self._road_chunks.append(self._road.attachNewNode("RoadChunk_0"))
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
            # Draw single lane
            y_coord = get_y_coordinate_from_lane_number(1, 0, self._road_chunk_size.y)
            road_chunk = self.__create_plane(
                "single_lane_road",
                self._road_chunk_size,
                (0, y_coord, 0),
                None,
                plane_texture_repeat,
            )
            road_chunk.setTexture(self.single_lane_road_chunk_texture, 1)
            road_chunk.reparentTo(self._road_chunks[0])
            self._lane_center_y_positions.append(y_coord)
            self._road_center_y_position = y_coord
            ######################################################################################

        elif msg.scenario_data.number_of_lanes > 1:
            # Draw upper lane
            y_coord = get_y_coordinate_from_lane_number(
                msg.scenario_data.number_of_lanes, 0, self._road_chunk_size.y
            )
            self._road_center_y_position = y_coord - self._road_chunk_size.y * 0.5
            upper_lane_road_chunk = self.__create_plane(
                "upper_lane_road_chunk",
                self._road_chunk_size,
                Vec3(0, y_coord, 0),
                None,
                plane_texture_repeat,
            )
            upper_lane_road_chunk.setTexture(self.upper_lane_road_chunk_texture, 1)
            upper_lane_road_chunk.reparentTo(self._road_chunks[0])
            self._lane_center_y_positions.append(y_coord)
            ######################################################################################

            # Draw upper lane border
            # y_coord = y_coord + self._road_chunk_size.y / 2 + (self._road_chunk_size.y / 5) / 2
            y_coord += self._road_chunk_size.y * 0.6
            upper_lane_road_border_size = Vec2(
                self._road_chunk_size.x, self._road_chunk_size.y * 0.2
            )
            upper_lane_road_border = self.__create_plane(
                "upper_lane_road_border",
                upper_lane_road_border_size,
                Vec3(0, y_coord, 0),
                None,
                plane_texture_repeat,
            )
            upper_lane_road_border.setTexture(self.upper_lane_road_border_texture, 1)
            upper_lane_road_border.reparentTo(self._road_chunks[0])
            ######################################################################################

            # Draw middle lanes
            y_coords = []
            middle_lane_count = msg.scenario_data.number_of_lanes - 2

            for i in range(middle_lane_count):
                y_coord = get_y_coordinate_from_lane_number(
                    msg.scenario_data.number_of_lanes, i + 1, self._road_chunk_size.y
                )
                self._lane_center_y_positions.append(y_coord)
                y_coords.append(y_coord)

            center_y_coord = (y_coords[0] + y_coords[middle_lane_count - 1]) * 0.5
            middle_lane_road_size = Vec2(
                self._road_chunk_size.x, self._road_chunk_size.y * middle_lane_count
            )
            middle_lane_road_texture_repeat = Vec2(
                plane_texture_repeat.x, middle_lane_count
            )
            middle_lane_road_chunk = self.__create_plane(
                "middle_lane_chunk",
                middle_lane_road_size,
                Vec3(0, center_y_coord, 0),
                None,
                middle_lane_road_texture_repeat,
            )
            middle_lane_road_chunk.setTexture(self.middle_lane_road_chunk_texture, 1)
            middle_lane_road_chunk.reparentTo(self._road_chunks[0])
            ######################################################################################

            # Draw lower lane
            y_coord = get_y_coordinate_from_lane_number(
                msg.scenario_data.number_of_lanes,
                msg.scenario_data.number_of_lanes - 1,
                self._road_chunk_size.y,
            )
            self._road_center_y_position = (
                self._road_center_y_position + (y_coord + self._road_chunk_size.y * 0.5)
            ) * 0.5
            lower_lane_road_chunk = self.__create_plane(
                "lower_lane_road_chunk",
                self._road_chunk_size,
                Vec3(0, y_coord, 0),
                None,
                plane_texture_repeat,
            )
            lower_lane_road_chunk.setTexture(self.lower_lane_road_chunk_texture, 1)
            lower_lane_road_chunk.reparentTo(self._road_chunks[0])
            self._lane_center_y_positions.append(y_coord)
            ######################################################################################

            # Draw lower lane border
            # y_coord = y_coord - self._road_chunk_size.y / 2 - (self._road_chunk_size.y / 5) / 2
            y_coord -= self._road_chunk_size.y * 0.6
            lower_lane_road_border_size = Vec2(
                self._road_chunk_size.x, self._road_chunk_size.y * 0.2
            )
            lower_lane_road_border = self.__create_plane(
                "lower_lane_road_border",
                lower_lane_road_border_size,
                Vec3(0, y_coord, 0),
                None,
                plane_texture_repeat,
            )
            lower_lane_road_border.setTexture(self.lower_lane_road_border_texture, 1)
            lower_lane_road_border.reparentTo(self._road_chunks[0])
            ######################################################################################

        self._road_chunks[0].setPos(Vec3(x_coord - self._road_chunk_size.x, 0, 0))

        for chunk_index in range(1, 3):
            prev_pos = self._road_chunks[chunk_index - 1].getPos()
            self._road_chunks.append(self._road_chunks[0].copyTo(self._road))
            self._road_chunks[chunk_index].setName("RoadChunk_" + str(chunk_index))
            self._road_chunks[chunk_index].setPos(
                Vec3(prev_pos.x + self._road_chunk_size.x, prev_pos.y, prev_pos.z)
            )

    def __adjust_road(self, msg):
        if (
            msg.ego_vehicle_trajectory.trajectory
            and msg.scenario_data.number_of_lanes > 0
        ):
            x_coord = msg.ego_vehicle_trajectory.trajectory[0].x
            road_chunk_limit_x = (
                self._road_chunks[self._current_road_chunk_index].getPos().x
                + self._half_road_chunk_size.x
            )

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
            self._cars = self._scene_root.attachNewNode("Cars")
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
            self.__move_camera(camera_position_z, ego_position)
            self.__show_ego_trajectory(ego_trajectory, 2)
        else:
            self._on_screen_ego.detachNode()
            self._on_screen_ego_trajectory.reset()

        for v in msg.scenario_data.vehicles_information:
            received_car_ids[v.id] = v.id
            car = self._on_screen_cars.get(v.id)

            if not car:
                car = self._vehicle_pool[
                    random.randrange(0, len(self._vehicle_pool))
                ].copyTo(self._cars)
                self._on_screen_cars[v.id] = car

            car.setPos(Vec3(v.pos_x, self._lane_center_y_positions[v.lane_number], 0))

        for k in self._on_screen_cars.keys():
            if not received_car_ids.get(k):
                car_to_remove = self._on_screen_cars.get(k)
                if car_to_remove:
                    car_to_remove.removeNode()
                cars_to_remove.append(k)

        for c in cars_to_remove:
            self._on_screen_cars.pop(c, None)

        # uint32 lane_number
        # uint32 id
        # float64 pos_x
        # float64 velocity_x
        # print("ON SCREEN CARS = " + str(len(self._on_screen_cars)) + ", INFO = " + str(len(msg.scenario_data.vehicles_information)))

    def __show_ego_trajectory(self, trajectory, height):
        if trajectory:
            """"""
            points = []

            for p in trajectory:
                points.append((p.x, p.y, height))

            self._on_screen_ego_trajectory.reset()
            self._on_screen_ego_trajectory.drawLines([(points)])
            self._on_screen_ego_trajectory.create()

    def __load_2d_texture(
        self,
        texture_file_name,
        texture_wrap_mode=(Texture.WM_repeat, Texture.WM_repeat),
        texture_filter=(Texture.FT_linear, Texture.FT_linear),
    ):
        texture = self.loader.loadTexture(
            os.path.join(self._textures_base_path, texture_file_name)
        )
        texture.setWrapU(texture_wrap_mode[0])
        texture.setWrapV(texture_wrap_mode[1])
        texture.setMagfilter(texture_filter[0])
        texture.setMinfilter(texture_filter[1])
        return texture

    def __load_textures(self):
        self.upper_lane_road_border_texture = self.__load_2d_texture(
            "UpperLaneBorder.png", (Texture.WM_mirror, Texture.WM_mirror)
        )
        self.upper_lane_road_chunk_texture = self.__load_2d_texture(
            "UpperLaneChunk.png", (Texture.WM_mirror, Texture.WM_mirror)
        )
        self.middle_lane_road_chunk_texture = self.__load_2d_texture(
            "MiddleLaneChunk.png", (Texture.WM_mirror, Texture.WM_mirror)
        )
        self.lower_lane_road_chunk_texture = self.__load_2d_texture(
            "LowerLaneChunk.png", (Texture.WM_mirror, Texture.WM_mirror)
        )
        self.lower_lane_road_border_texture = self.__load_2d_texture(
            "LowerLaneBorder.png", (Texture.WM_mirror, Texture.WM_mirror)
        )
        self.single_lane_road_chunk_texture = self.__load_2d_texture(
            "SingleLaneChunk.png", (Texture.WM_mirror, Texture.WM_mirror)
        )

    def __load_3d_model(self, model_info):
        model_node = self.loader.loadModel(
            os.path.join(self._models_base_path, model_info.fileName)
        )
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
            ego.scale = Vec3(
                float(ego_json["scale"]["x"]),
                float(ego_json["scale"]["y"]),
                float(ego_json["scale"]["z"]),
            )

            for vehicle_json in vehicle_json_data["Vehicles"]:
                v = VehicleModelInfo()
                v.fileName = vehicle_json["fileName"]
                v.rotation.roll = float(vehicle_json["rotation"]["roll"])
                v.rotation.pitch = float(vehicle_json["rotation"]["pitch"])
                v.rotation.yaw = float(vehicle_json["rotation"]["yaw"])
                v.scale = Vec3(
                    float(vehicle_json["scale"]["x"]),
                    float(vehicle_json["scale"]["y"]),
                    float(vehicle_json["scale"]["z"]),
                )
                vehicles.append(v)

            return [ego, vehicles]

    def __can_initialize_gui(self):
        win_size = self.win.getProperties().getSize()
        return win_size.x == WINDOW_WIDTH and win_size.y == WINDOW_HEIGHT

    def __init_gui(self):
        
        z_offset = 0.025
        frame_size = Vec2(
            (self.a2dRight - self.a2dLeft) * 0.97, (self.a2dTop - self.a2dBottom) * 0.1
        )
        self.info_frame = DirectFrame(
            parent=self.aspect2d,
            frameColor=(1, 1, 1, 0.3),
            frameSize=(
                -frame_size.x * 0.5,
                frame_size.x * 0.5,
                -frame_size.y * 0.5,
                frame_size.y * 0.5,
            ),
            pos=(0, 0, self.a2dBottom + frame_size.y * 0.5 + z_offset + 0.15),
        )
        pos_z = self.info_frame.getHeight() * self.info_frame.getScale().z * 0.5

        self.speed_info = DirectFrame(
            parent=self.info_frame,
            text="Speed: 130 km/h",
            frameColor=(1, 1, 1, 0),
            pos=(0, 0, 0),
            scale=0.08,
        )
        pos_x = z_offset + 0.5 * (
            -self.info_frame.getWidth() * self.info_frame.getScale().x
            + self.speed_info.getWidth() * self.speed_info.getScale().x
        )
        pos_z -= 0.05 + self.speed_info.getHeight() * self.speed_info.getScale().z * 0.5
        self.speed_info.setPos(Vec3(pos_x, 0, pos_z))

        self.target_lane_info = DirectFrame(
            parent=self.info_frame,
            text="Target lane: 3",
            frameColor=(1, 1, 1, 0),
            pos=(0, 0, 0),
            scale=0.08,
        )
        pos_x = 1.3 + z_offset + 0.5 * (
            -self.info_frame.getWidth() * self.info_frame.getScale().x
            + self.target_lane_info.getWidth() * self.target_lane_info.getScale().x
        )
        self.target_lane_info.setPos(Vec3(pos_x, 0, pos_z))

        self.lane_change_status_info = DirectFrame(
            parent=self.info_frame,
            text="Lane change status: In progress",
            frameColor=(1, 1, 1, 0),
            pos=(0, 0, 0),
            scale=0.08,
        )
        pos_x = (
            self.info_frame.getPos().x
            + z_offset
            + 0.5
            + 0.5
            * (
                self.lane_change_status_info.getWidth()
                * self.lane_change_status_info.getScale().x
            )
        )

        self.lane_change_status_info.setPos(Vec3(pos_x, 0, pos_z))

    def __update_gui(self, msg):
        """TODO"""
        speed = msg.ego_info.velocity_x
        self.speed_info["text"] = f"Speed: {speed} km/h"

    def __update_gui_kpi(self, kpi_msg):
        """TODO"""
        number_lane_change = kpi_msg.lane_changes        
        self.target_lane_info["text"] = f"Lane changes: {number_lane_change}"

        avg_speed = kpi_msg.overall_avg_velocity
        self.lane_change_status_info["text"] = f"Scenario avg speed: {avg_speed:.2f}"
        
    def update(self, msg, kpi_msg):
        if hasattr(self, "latest_msg"):
            self._rebuild_road = (
                self.latest_msg.scenario_data.number_of_lanes
                != msg.scenario_data.number_of_lanes
            )

        self.latest_msg = msg
        self.kpi_msg = kpi_msg

    def __make_plane(self, size=(1.0, 1.0), texture_repeat=(1, 1)):
        """Make a plane geometry.

        Arguments:
            size {tuple} -- plane size x,y

        Returns:
            Geom -- p3d geometry
        """
        vformat = GeomVertexFormat.get_v3n3c4t2()
        vdata = GeomVertexData("vdata", vformat, Geom.UHStatic)
        vdata.uncleanSetNumRows(4)

        vertex = GeomVertexWriter(vdata, "vertex")
        normal = GeomVertexWriter(vdata, "normal")
        color = GeomVertexWriter(vdata, "color")
        tcoord = GeomVertexWriter(vdata, "texcoord")

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
                f"* Message #{header.seq} contains an empty trajectory (at {timestamp.secs}.{timestamp.nsecs} seconds)"
            )
            return []

        return trajectory


###########################################################


def __make_title(scenario):
    timestamp = scenario.header.stamp
    return f"Frame #{scenario.header.seq}, time: {timestamp.secs}.{timestamp.nsecs}"
