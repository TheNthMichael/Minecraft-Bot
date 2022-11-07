"""
Specify the events that can be sent over the queue to the visualizer.
"""
import utility

class MSGAddBlock(object):
    def __init__(self, position: utility.Vector3, btype: str):
        self.position = position
        self.btype = btype

class MSGDeleteBlock(object):
    def __init__(self, position: utility.Vector3):
        self.position = position

class MSGPlayerState(object):
    def __init__(self, position: utility.Vector3, orientation: utility.Vector3) -> None:
        self.position = position
        self.orientation = orientation


class MSGPathfindTask(object):
    def __init__(self, position: utility.Vector3) -> None:
        self.position = position

class MSGLookAt(object):
    def __init__(self, position: utility.Vector3) -> None:
        self.position = position