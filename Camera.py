#Camera.py

from Objects import Object

class Camera(Object):
    def __init__(self, center=[0, 0, 0]):
        super().__init__(center)
        self.rotation = [0, 0]
        self.rotationLock = True
