from abc import ABC, abstractmethod


class Scene2DObject(ABC):
    def __init__(
        self,
    ):
        self.scene = None

    @abstractmethod
    def plot(
        self,
    ):
        raise NotImplementedError("Derived class must implement 'plot' function")

    def add_scene_info(self, scene):
        self.scene = scene
