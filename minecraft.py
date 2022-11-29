"""
Module offers api for controlling the player through computer vision and simulating keypresses.
"""
from math import asin, atan2, floor, pi
from actions import *
from map import QueryMap
from pathfinder import Pathfinder
from ctypes import windll, Structure, c_long, byref

from utility import *
import mss

class POINT(Structure):
    _fields_ = [("x", c_long), ("y", c_long)]

def queryMousePosition():
    pt = POINT()
    windll.user32.GetCursorPos(byref(pt))
    return { "x": pt.x, "y": pt.y}

class MinecraftPlayer:
    def __init__(self, bb_coords, bb_rotation, bb_block_coords, bb_block_type, shared_map, shared_lock) -> None:
        self.bb_coords = bb_coords
        self.bb_rotation = bb_rotation
        self.bb_block_coords = bb_block_coords
        self.bb_block_type = bb_block_type

        self.coordinate_regex = re.compile(r'([+-]?\d+\.\d+)/([+-]?\d+\.\d+)/([+-]?\d+\.\d+)')
        self.rotation_regex = re.compile(r'[a-zA-Z]+\([a-zA-Z]+\)\(([+-]?\d+\.\d+)/([+-]?\d+\.\d+)\)')
        self.target_block_position_regex = re.compile(r'([+-]?\d+),([+-]?\d+),([+-]?\d+)')
        self.target_block_type_regex = re.compile(r'(.+)')

        self.position = Vector3(0, 0, 0)
        self.eye_height_offset = Vector3(0, 0.6, 0)
        self.prev_position = None
        self.rotation = Vector2(0, 0)
        self.prev_rotation = None
        self.target_position = Vector3(0,0,0)
        self.prev_target_position = None
        self.target_type = ""
        self.prev_target_type = None

        self.time = 0
        self.prev_time = None

        self.max_speed_tolerance = 10
        self.max_rotation_tolerance = 720
        self.max_target_tolerance = 100

        self.current_position_image = None
        self.current_rotation_image = None
        self.current_block_position_image = None
        self.current_block_type_image = None

        self.map = Map(shared_map=shared_map, shared_lock=shared_lock)
        self.qmap = QueryMap(self.map, None)
        self.qmap.record_edge_cost_changes()

        self.action_queue = []

        self.rotation_action_queue = []

        self.movement_action_queue = []

        self.screen = mss.mss()

    # Create a set of functions that give an idea of the players state.
    @property
    def position_at_eye(self):
        return self.position.add(self.eye_height_offset)

    @property
    def forward(self):
        return Vector3(
            cos(radians(self.pitch_normal)) * sin(radians(self.yaw_normal)), # right
            sin(radians(self.pitch_normal)),    # up
            cos(radians(self.pitch_normal)) * cos(radians(self.yaw_normal)) # forward
        )

    @property
    def forward_position(self):
        return self.forward.add(self.position_at_eye)
    
    @property
    def yaw_normal(self):
        return minecraft_yaw_to_normal(self.rotation.x)

    @property
    def pitch_normal(self):
        return minecraft_pitch_to_normal(self.rotation.y)

    def bearing2vector(self, yaw, pitch, is_radians=False):
        if not is_radians:
            pitch = radians(pitch)
            yaw = radians(yaw)
        return Vector3(
            cos(pitch) * sin(yaw), # right
            sin(pitch), # up
            cos(pitch) * cos(yaw) # forward
        )

    def look_at(self, position: Vector3):
        """
        Looks at the given position. Note, this is eye to exact position.
        If you pass in the top of a block, it will look exactly at the top of the block.
        Generally, you will want to pass in the mid-point of the block you want to look at.
        @param position (Vector3) - A vector containing the position to look at.
        @returns A Vector2 containing the desired yaw and pitch difference between the current direction and the desired direction.
        """
        # Just as shrimple as that.
        fr = self.position_at_eye
        target = position.subtract(fr)
        mag = target.magnitude()
        target = target.scalar_div(mag)

        pitch = asin(target.y) * (180.0 / pi)
        yaw = atan2(target.x, target.z) * (180.0 / pi)
        return Vector2(yaw_to_minecraft_yaw(yaw), pitch_to_minecraft_pitch(pitch))

    def add_action(self, event):
        """
        Adds the action to the event queue.
        """
        self.action_queue.append(event)

    def _insert_events(self, events, position):
        """
        Adds the events at the given position. ([p-1, <events>, p+1])
        """
        self.action_queue = self.action_queue[:position] + events + self.action_queue[position:]

    def serve_action(self):
        """
        Get an action from the top of the queue and execute it.
        Removes that action, if it is complete.
        """
        if len(self.action_queue) <= 0:
            return
        action = self.action_queue[0]
        finished, new_events = action.act(self)
        if finished:
            self.action_queue.pop(0)
        if len(new_events) != 0:
            self._insert_events(new_events, 1)
    
    def update(self, api):
        """
        Grab all new text, update time, then update pos, rot, block, etc...
        """
        # Position
        api.SetVariable("tessedit_char_whitelist", "./-0123456789")
        #im_position = ImageGrab.grab(bbox=self.bb_coords)
        mon = {"top": self.bb_coords[1], "left": self.bb_coords[0], "width": self.bb_coords[2] - self.bb_coords[0], "height": self.bb_coords[3] - self.bb_coords[1]}
        im_position = self.screen.grab(mon)
        pos_text, self.current_position_image = image_to_text(api, im_position)

        # Rotation
        api.SetVariable("tessedit_char_whitelist", "./-()" + string.digits + string.ascii_letters.replace('S', ""))
        #im_rotation = ImageGrab.grab(bbox=self.bb_rotation)

        mon = {"top": self.bb_rotation[1], "left": self.bb_rotation[0], "width": self.bb_rotation[2] - self.bb_rotation[0], "height": self.bb_rotation[3] - self.bb_rotation[1]}
        im_rotation = self.screen.grab(mon)
        rot_text, self.current_rotation_image = image_to_text(api, im_rotation)


        # Block Look Position
        api.SetVariable("tessedit_char_whitelist", "-," + string.digits + string.ascii_letters.replace('S', ""))
        #im_block_position = ImageGrab.grab(bbox=self.bb_block_coords)

        mon = {"top": self.bb_block_coords[1], "left": self.bb_block_coords[0], "width": self.bb_block_coords[2] - self.bb_block_coords[0], "height": self.bb_block_coords[3] - self.bb_block_coords[1]}
        im_block_position = self.screen.grab(mon)
        block_position_text, self.current_block_position_image = image_to_text(api, im_block_position, crop_to_activity=True, crop_extra=160)
        block_position_text = block_position_text.replace(' ',  '')

        # Block Look Type
        api.SetVariable("tessedit_char_whitelist", "_" + string.ascii_lowercase)
        #im_block_type = ImageGrab.grab(bbox=self.bb_block_type)

        mon = {"top": self.bb_block_type[1], "left": self.bb_block_type[0], "width": self.bb_block_type[2] - self.bb_block_type[0], "height": self.bb_block_type[3] - self.bb_block_type[1]}
        im_block_type = self.screen.grab(mon)
        block_type_text, self.current_block_type_image = image_to_text(api, im_block_type, crop_to_activity=True, crop_extra=97)

        #block_type_text = block_type_text.replace(' ', '')


        self.time = time.time()
        if self.prev_time is None:
                self.prev_time = self.time
        coord = self.update_coords(pos_text)
        rot = self.update_rotation(rot_text)
        block_pos = self.update_block_position(block_position_text)
        block_type = self.update_block_type(block_type_text)

        self.prev_time = self.time

        if block_pos and block_type:
            # print(f"Adding block to map {self.target_type}: {self.target_position}")
            tpos = self.target_position.copy()
            self.map.add_block(self.target_type, tpos)
            element = self.qmap.get(tpos)
            if element is None:
                self.qmap.add(MapNode(self.target_type, tpos, []))
            else:
                self.qmap.update_type(tpos, self.target_type, False)
                self.map.shared_map[tpos].update_type = True

            raycast = Bresenham3D(self.position_at_eye, self.forward.scalar_mult(20).add(self.position_at_eye))

            current_node_ordering = self.target_position.subtract(self.position_at_eye).magnitude()
            #print("-------------")
            for block in raycast:
                ordering = block.subtract(self.position_at_eye).magnitude()
                node_at_block = self.qmap.get(block)

                if ordering < current_node_ordering:
                    #print(block)
                    # if we have a block here and it should have been detected instead of the block we actually saw
                    if node_at_block is not None and node_at_block.block_type != "air":
                        print(f"block removed -> {block}")
                        self.qmap.update_type(block, "air", only_if_uncertain=False)
                        self.map.shared_map[block].update_type = True
                    elif node_at_block is None:
                        self.qmap.add(MapNode("air", block, []))

        if coord and rot:
            # Go through actions
            self.serve_action()

        return coord and rot

    def update_coords(self, pos_text):
        
        match = self.coordinate_regex.match(pos_text)

        if match:
            x, y, z = match.groups()
            self.position.reassign(float(x), float(y), float(z))

            if self.prev_position is None:
                self.prev_position = Vector3(self.position.x, self.position.y, self.position.z)

            dt = self.time - self.prev_time
            if dt == 0:
                dt = 0.00001
            dx = (self.position.x - self.prev_position.x) / dt
            dy = (self.position.y - self.prev_position.y) / dt
            dz = (self.position.z - self.prev_position.z) / dt

            speed = Vector3(dx, dy, dz)
            
            self.prev_position.reassign(self.position.x, self.position.y, self.position.z)

            if speed.magnitude() > self.max_speed_tolerance:
                pass
                #print("Coord error, invalid speed")
            #print(f"\tCoords - X: {self.position.x}, Y: {self.position.y}, Z: {self.position.z}", f"\t\tSpeed - dx: {dx}, dy: {dy}, dz: {dz}, dt: {dt}")
        else:
            # print(f"[Error] Could not parse coordinates this frame: {pos_text}")
            return False
        return True

    def update_rotation(self, rot_text):
        match = self.rotation_regex.match(rot_text)

        if match:
            x, y = match.groups()
            self.rotation.reassign(float(x), float(y))

            if self.prev_rotation is None:
                self.prev_rotation = Vector2(self.rotation.x, self.rotation.y)

            dt = self.time - self.prev_time
            if dt == 0:
                dt = 0.00001
            dx = (self.rotation.x - self.prev_rotation.x) / dt
            dy = (self.rotation.y - self.prev_rotation.y) / dt

            speed = Vector2(dx, dy)
            
            self.prev_rotation.reassign(self.rotation.x, self.rotation.y)

            #if speed.magnitude() > self.max_rotation_tolerance:
                #print("Coord error, invalid rotation")
            #print(f"\tRotation - X: {self.rotation.x}, Y: {self.rotation.y}", f"\t\tSpeed - dx: {dx}, dy: {dy}, dt: {dt}")
        else:
            #print(f"[Error] Could not parse rotation this frame: {rot_text}")
            return False
        return True

    def update_block_position(self, block_position_text):
        match = self.target_block_position_regex.match(block_position_text)

        if match:
            x, y, z = match.groups()
            self.target_position.reassign(float(x), float(y), float(z))

            if self.prev_target_position is None:
                self.prev_target_position = Vector3(self.target_position.x, self.target_position.y, self.target_position.z)

            dt = self.time - self.prev_time
            if dt == 0:
                dt = 0.00001
            dx = (self.target_position.x - self.prev_target_position.x) / dt
            dy = (self.target_position.y - self.prev_target_position.y) / dt
            dz = (self.target_position.z - self.prev_target_position.z) / dt

            speed = Vector3(dx, dy, dz)
            
            self.prev_target_position.reassign(self.target_position.x, self.target_position.y, self.target_position.z)

            #if speed.magnitude() > self.max_target_tolerance:
                #print("Coord error, invalid speed")
            #print(f"\tTargetCoords - X: {self.target_position.x}, Y: {self.target_position.y}, Z: {self.target_position.z}", f"\t\tSpeed - dx: {dx}, dy: {dy}, dz: {dz}, dt: {dt}")
        else:
            # print(f"[Error] Could not parse target coordinates this frame: {block_position_text}")
            return False
        return True

    def update_block_type(self, block_type_text):
        match = self.target_block_type_regex.match(block_type_text)

        if match:
            self.prev_target_type = self.target_type
            self.target_type = match.group(0)
            #print(f"\tTargetType - {self.target_type}")
        else:
            # print(f"[Error] Could not parse target type this frame: {block_type_text}")
            return False
        return True