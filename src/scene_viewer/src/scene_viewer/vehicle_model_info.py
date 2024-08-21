from panda3d.core import Vec3


class Rotation:
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0


class VehicleModelInfo:
    def __init__(self):
        self.fileName = ""
        self.rotation = Rotation()
        self.scale = Vec3(1, 1, 1)
