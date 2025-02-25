"""
Module handles all the utility functions and classes for minecraft.py
"""

from cmath import isclose, nan
from math import ceil, floor, cos, sin, radians
from msilib.schema import Error
import cv2
import ctypes
import numpy as np
import re
import time
import string
import pydirectinput
pydirectinput.PAUSE = 0
import cython

from PIL import ImageGrab, Image

def get_middle_top_of_block(block):
    return block.add(Vector3(0.5, 0, 0.5))

def get_middle_of_block(block):
    return block.add(Vector3(0.5, 0, 0.5))

# Some math utilities
def clamp(x, lower, upper):
    if x < lower:
        return lower
    elif x > upper:
        return upper
    else:
        return x

def sign(x):
    if x < 0:
        return -1
    else:
        return 1

def linmap(x, in_min, in_max, out_min, out_max):
    """
    Transforms the value x from the input range to the output range.
    """
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def minecraft_yaw_to_normal(x):
    """
    returns the yaw or horizontal rotation from 0 to 360, positive ccw.
    if (x <= 0):
        return abs(x)
    else:
        return 360 - x
    """
    # 0, -179.9999 -> 0, 179.9999
    # 179.9999, 0 -> 18.00001, 360
    if x < 180:
        return -1 * x
    elif x > 180:
        return 360 - x
    else:
        return 180 # not possible

def yaw_to_minecraft_yaw(x):
    """
    returns the yaw or horizontal rotation from (0, -179) ccw and (0, 179) cw. (disgusting)
    if (x < 180):
        return -1 * x
    else:
        return 360 - x
    """
    if x < 180:
        return -1 * x
    elif x > 180:
        return -x + 360
    else:
        return 179.9999

def minecraft_pitch_to_normal(x):
    """
    returns the pitch or vertical rotation from 0 to 180, negative ccw, 90 is forward.
    """
    return -x    

def pitch_to_minecraft_pitch(x):
    """
    returns the pitch or vertical rotation from (-90, 90) -90 being up, 0 being forward, and 90 being down (why?)
    """
    return -x

def get_neighboring_blocks(block_position):
    """
    Returns an array of BlockRotation elements.
    """
    return [
                # Forward
                BlockRotation(Vector2(0,0), block_position.add(Vector3(0, 1, 1))),
                BlockRotation(Vector2(0,60), block_position.add(Vector3(0, 0, 1))),
                BlockRotation(Vector2(0,60), block_position.add(Vector3(0, -1, 1))),
                BlockRotation(Vector2(0,90), block_position.add(Vector3(0, -1, 0))),
                # Left
                BlockRotation(Vector2(-90, 0), block_position.add(Vector3(1, 1, 0))),
                BlockRotation(Vector2(-90, 60), block_position.add(Vector3(1, 0, 0))),
                BlockRotation(Vector2(-90, 60), block_position.add(Vector3(1, -1, 0))),
                # Backwards
                BlockRotation(Vector2(180, 0), block_position.add(Vector3(0, 1, -1))),
                BlockRotation(Vector2(180, 60), block_position.add(Vector3(0, 0, -1))),
                BlockRotation(Vector2(180, 60), block_position.add(Vector3(0, -1, -1))),
                # Right
                BlockRotation(Vector2(90, 0), block_position.add(Vector3(-1, 1, 0))),
                BlockRotation(Vector2(90, 60), block_position.add(Vector3(-1, 0, 0))),
                BlockRotation(Vector2(90, 60), block_position.add(Vector3(-1, -1, 0))),
            ]

def get_neighboring_blocks_dict(block_position):
    return {
        "forward": {
            "start": BlockRotation(Vector2(0,60), block_position.add(Vector3(0, -1, 1))),
            "rest": [
                BlockRotation(Vector2(0,0), block_position.add(Vector3(0, 1, 1))),
                BlockRotation(Vector2(0,60), block_position.add(Vector3(0, 0, 1))),
            ],
        },
        "left": {
            "start": BlockRotation(Vector2(-90, 60), block_position.add(Vector3(1, -1, 0))),
            "rest": [
                BlockRotation(Vector2(-90, 0), block_position.add(Vector3(1, 1, 0))),
                BlockRotation(Vector2(-90, 60), block_position.add(Vector3(1, 0, 0))),
            ],
        },
            
        "backwards": {
            "start": BlockRotation(Vector2(180, 60), block_position.add(Vector3(0, -1, -1))),
            "rest": [
                BlockRotation(Vector2(180, 0), block_position.add(Vector3(0, 1, -1))),
                BlockRotation(Vector2(180, 60), block_position.add(Vector3(0, 0, -1))),
            ],
        }, 
        "right": {
            "start": BlockRotation(Vector2(90, 60), block_position.add(Vector3(-1, -1, 0))),
            "rest": [
                BlockRotation(Vector2(90, 0), block_position.add(Vector3(-1, 1, 0))),
                BlockRotation(Vector2(90, 60), block_position.add(Vector3(-1, 0, 0))),
            ],
        },
    }

def v_filter_gray(rgb):
    # Expect len 3
    r, g, b = rgb

    r = (r + 150) / 2
    g = (g + 150) / 2
    b = (b + 150) / 2

    mean = (r + g + b) / 3
    diffr = abs(mean - r)
    diffg = abs(mean - g)
    diffb = abs(mean - b)

    maxdev = 2

    if (diffr + diffg + diffb) > maxdev:
        return 0
    return rgb

def filter_gray(na):
    mean = np.mean(na, axis=2, keepdims=True)

    res = (na[..., 2] / na[..., 0]) * 100/l
    return res.astype(np.uint8)

def process_image(im, crop_to_activity=False, crop_extra=0):
    """
    Converts the image to a numpy array, then applies preprocessing.
    """
    im_arr = np.array(im)

    kernel = np.ones((1,1), np.uint8)
    im_arr = cv2.dilate(im_arr, kernel, iterations=2)
        
    height, width, depth = im_arr.shape

    hsv = cv2.cvtColor(im_arr, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, np.array([0,0,0]), np.array([179,2,255]))

    im_arr = cv2.bitwise_and(im_arr, im_arr, mask = mask) 

    #im_arr = np.apply_along_axis(v_filter_gray, 2, im_arr)


    """for i in range(height):
        for j in range(width):
            r, g, b = im_arr[i][j]

            r = (r + 150) / 2
            g = (g + 150) / 2
            b = (b + 150) / 2

            mean = (r + g + b) / 3
            diffr = abs(mean - r)
            diffg = abs(mean - g)
            diffb = abs(mean - b)

            maxdev = 2

            if (diffr + diffg + diffb) > maxdev:
                im_arr[i][j][0] = 0
                im_arr[i][j][1] = 0
                im_arr[i][j][2] = 0"""
            
    #im_arr = cv2.cvtColor(im_arr, cv2.COLOR_HLS2BGR)

    im_arr = cv2.cvtColor(im_arr, cv2.COLOR_BGR2GRAY)

        #cap_arr = cv2.threshold(cap_arr,127,255,cv2.THRESH_BINARY)
    
    # Otsu's thresholding after Gaussian filtering
    #blur = cv2.GaussianBlur(cap_arr,(3,3),0)
    ret3, im_arr = cv2.threshold(im_arr,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)


    if crop_to_activity:
        positions = np.nonzero(im_arr)
        if len(positions) > 0:
            left = np.min(positions[1], initial=width-1)
            last_column = max(0, left+crop_extra)
            im_arr = im_arr[:, last_column:]
        """last_column = -1
        for j in range(width):
            for i in range(height):
                v = im_arr[i][j]

                if v != 0:
                    last_column = j
                    break
            if last_column != -1:
                break
        
        last_column = max(0, last_column+crop_extra)
        im_arr = im_arr[:, last_column:]"""
    
    im_arr = cv2.bitwise_not(im_arr)

    return im_arr

def image_to_text(api, image, crop_to_activity=False, crop_extra=0):
    """
    Returns the text and the processed image as a numpy array.
    """
    image_array = process_image(image, crop_to_activity, crop_extra)
    try:
        image = Image.fromarray(np.uint8(image_array))
    except:
        image_array = process_image(image, crop_to_activity)
        image = Image.fromarray(np.uint8(image_array))
    api.SetImage(image)
    return api.GetUTF8Text(), image_array

def press_key_for_t(key, t):
    start_time = time.time()
    pydirectinput.keyDown(key)
    while True:
        current_time = time.time()
        elapsed_time = current_time - start_time
        if elapsed_time >= t:
            pydirectinput.keyUp(key)
            break


def sign2(x):
    if x > 0:
        return 1
    elif x < 0:
        return -1
    else:
        return 0
def FRAC0(x):
    return x - floor(x)

def FRAC1(x):
    return 1 - x + floor(x)

# Implemented from https://www.cse.chalmers.se/edu/year/2011/course/TDA361/grid.pdf
def fast_voxel_traversal_3D(a, b):
    result = []
    tMaxX = 0
    tMaxY = 0
    tMaxZ = 0
    tDeltaX = 0
    tDeltaY = 0
    tDeltaZ = 0
    voxel = Vector3(0,0,0)

    x1 = a.x
    y1 = a.y
    z1 = a.z

    x2 = b.x
    y2 = b.y
    z2 = b.z

    dx = sign2(x2 - x1)
    if dx != 0:
        tDeltaX = min(dx / (x2 - x1), 10000000.0)
    else:
        tDeltaX = 10000000.0
    if dx > 0:
        tMaxX = tDeltaX * FRAC1(x1)
    else:
        tMaxX = tDeltaX * FRAC0(x1)
    voxel.x = int(x1)

    dy = sign2(y2 - y1)
    if dy != 0:
        tDeltaY = min(dy / (y2 - y1), 10000000.0)
    else:
        tDeltaY = 10000000.0
    if dy > 0:
        tMaxY = tDeltaY * FRAC1(y1)
    else:
        tMaxY = tDeltaY * FRAC0(y1)
    voxel.y = int(y1)

    dz = sign2(z2 - z1)
    if dz != 0:
         tDeltaZ = min(dz / (z2 - z1), 10000000.0)
    else:
        tDeltaZ = 10000000.0
    if dz > 0:
        tMaxZ = tDeltaZ * FRAC1(z1)
    else:
        tMaxZ = tDeltaZ * FRAC0(z1)
    voxel.z = int(z1)

    while True:
        if tMaxX < tMaxY:
            if tMaxX < tMaxZ:
                voxel.x += dx
                tMaxX += tDeltaX
            else:
                voxel.z += dz
                tMaxZ += tDeltaZ
            
        else:
            if tMaxY < tMaxZ:
                voxel.y += dy
                tMaxY += tDeltaY
            else:
                voxel.z += dz
                tMaxZ += tDeltaZ
        if tMaxX > 1 and tMaxY > 1 and tMaxZ > 1:
            break
        # process voxel here
        add_voxel = True
        if len(result) != 0:
            # Remove points captured on the diagnol.
            prev = result[-1]
            c_prev = get_middle_of_block(prev)
            d = get_middle_of_block(voxel.copy())
            cd = d.subtract(c_prev)
            ab = a.subtract(b)
            if isclose(cd.dot_product(ab), 0, rel_tol=1e-7) and isclose(cd.magnitude(), 2**0.5, rel_tol=1e-7):
                result.pop()
                add_voxel = False
        if add_voxel:
            result.append(voxel.copy())
    return result

# Need a better algorithm - see "A faster voxel traversal algorithm for raytracing"
def Bresenham3D(a, b):
    a.apply_func_to_all(int)
    b.apply_func_to_all(int)
    x1 = int(a.x)
    y1 = int(a.y)
    z1 = int(a.z)
    x2 = int(b.x)
    y2 = int(b.y)
    z2 = int(b.z)
    ListOfPoints = []
    ListOfPoints.append(Vector3(x1, y1, z1))
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    dz = abs(z2 - z1)
    if (x2 > x1):
        xs = 1
    else:
        xs = -1
    if (y2 > y1):
        ys = 1
    else:
        ys = -1
    if (z2 > z1):
        zs = 1
    else:
        zs = -1
  
    # Driving axis is X-axis"
    if (dx >= dy and dx >= dz):        
        p1 = 2 * dy - dx
        p2 = 2 * dz - dx
        while (x1 != x2):
            x1 += xs
            if (p1 >= 0):
                y1 += ys
                p1 -= 2 * dx
            if (p2 >= 0):
                z1 += zs
                p2 -= 2 * dx
            p1 += 2 * dy
            p2 += 2 * dz
            ListOfPoints.append(Vector3(x1, y1, z1))
  
    # Driving axis is Y-axis"
    elif (dy >= dx and dy >= dz):       
        p1 = 2 * dx - dy
        p2 = 2 * dz - dy
        while (y1 != y2):
            y1 += ys
            if (p1 >= 0):
                x1 += xs
                p1 -= 2 * dy
            if (p2 >= 0):
                z1 += zs
                p2 -= 2 * dy
            p1 += 2 * dx
            p2 += 2 * dz
            ListOfPoints.append(Vector3(x1, y1, z1))
  
    # Driving axis is Z-axis"
    else:        
        p1 = 2 * dy - dz
        p2 = 2 * dx - dz
        while (z1 != z2):
            z1 += zs
            if (p1 >= 0):
                y1 += ys
                p1 -= 2 * dz
            if (p2 >= 0):
                x1 += xs
                p2 -= 2 * dz
            p1 += 2 * dy
            p2 += 2 * dx
            ListOfPoints.append(Vector3(x1, y1, z1))
    return ListOfPoints

class BlockRotation:
    def __init__(self, rotation, position) -> None:
        self.rotation = rotation
        self.position = position

    def copy(self):
        return BlockRotation(self.rotation.copy(), self.position.copy())

    def __eq__(self, other):
        """Overrides the default implementation"""
        if isinstance(other, BlockRotation):
            return self.position.__eq__(other.position) and self.rotation.__eq__(other.rotation)
            #return self.x == other.x and self.y == other.y and self.z == other.z
        return False
    
    def __ne__(self, other):
        """Overrides the default implementation (unnecessary in Python 3)"""
        return not self.__eq__(other)

    def __hash__(self):
        return hash((self.position.__hash__(), self.rotation.__hash__()))

    def __repr__(self) -> str:
        return f"(x: {self.position}, theta: {self.rotation})"

    def __str__(self) -> str:
        return f"(x: {self.position}, theta: {self.rotation})"

class Vector3:
    def dot_product(self, second):
        return self.x * second.x + self.y * second.y + self.z * second.z

    def distance(self, other):
        return self.subtract(other).magnitude()
    
    def manhattan_distance(self, other):
        t = self.subtract(other)
        return abs(t.x) + abs(t.y) + abs(t.z)

    def cross_product(self, second):
        return Vector3(
            self.y * second.z - self.z * second.y,
            -(self.z * second.x - self.x * second.z),
            self.x * second.y - self.y * second.x
        )

    def __init__(self, x :float, y :float, z :float) -> None:
        self.x = x
        self.y = y
        self.z = z

    def reassign(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def apply_func_copy(self, v):
        x = v(self.x)
        y = v(self.y)
        z = v(self.z)
        return Vector3(x,y,z)

    def apply_func_to_all(self, v):
        self.x = v(self.x)
        self.y = v(self.y)
        self.z = v(self.z)
    
    def magnitude(self):
        return (self.x**2 + self.y**2 + self.z**2)**0.5

    def subtract(self, other):
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __sub__(self, other):
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def add(self, other):
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)
    
    def __add__(self, other):
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def scalar_mult(self, x):
        return Vector3(self.x * x, self.y * x, self.z * x)

    def __mult__(self, x):
        if type(x) is Vector3:
            raise Error("Cannot multiply two vectors, use dot product or cross product helper functions.")
        return Vector3(self.x * x, self.y * x, self.z * x)

    def scalar_div(self, x):
        return self.scalar_mult(1.0/x)

    def __div__(self, x):
        if type(x) is Vector3:
            raise Error("Cannot divide two vectors?")
        return self.__mult__(1.0/x)

    def dot_product(self, other):
        return self.x * other.x + self.y * other.y + self.z * other.z

    def distance(self, other):
        return self.subtract(other).magnitude()
    
    def rotate_about_origin_xy(self, origin, angle):
        angle = radians(angle)
        x = self.x - origin.x
        z = self.z - origin.z
        tx = x * cos(angle) - z * sin(angle)
        tz = x * sin(angle) + z * cos(angle)
        tx += origin.x
        tz += origin.z

        return Vector3(tx, self.y, tz)
    
    def __eq__(self, other):
        """Overrides the default implementation"""
        if isinstance(other, Vector3):
            return isclose(self.x, other.x, rel_tol=1e-09, abs_tol=0.0) \
                and isclose(self.y, other.y, rel_tol=1e-09, abs_tol=0.0) \
                and isclose(self.z, other.z, rel_tol=1e-09, abs_tol=0.0)
            #return self.x == other.x and self.y == other.y and self.z == other.z
        return False
    
    def __ne__(self, other):
        """Overrides the default implementation (unnecessary in Python 3)"""
        return not self.__eq__(other)

    def __hash__(self):
        return hash((float(self.x), float(self.y), float(self.z)))

    def copy(self):
        return Vector3(self.x, self.y, self.z)

    def __repr__(self) -> str:
        return f"({self.x}, {self.y}, {self.z})"

    def __str__(self) -> str:
        return f"({self.x}, {self.y}, {self.z})"

class Vector2:
    def __init__(self, x, y) -> None:
        self.x = x
        self.y = y

    def reassign(self, x, y):
        self.x = x
        self.y = y

    def copy(self):
        return Vector2(self.x, self.y)

    def magnitude(self):
        return (self.x**2 + self.y**2)**0.5

    def subtract(self, other):
        return Vector2(self.x - other.x, self.y - other.y)

    def __repr__(self) -> str:
        return f"({self.x}, {self.y})"

    def __str__(self) -> str:
        return f"({self.x}, {self.y})"

class Block:
    def __init__(self, position: Vector3, block_type) -> None:
        self.position = position
        self.type = block_type
        self.update_type = False
        self.voxel = None

    def __hash__(self) -> int:
        return hash((self.position.x, self.position.y, self.position.z))

    def __str__(self) -> str:
        return f"{self.type} - {self.position}"

    def __repr__(self) -> str:
        return f"{self.type} - {self.position}"
    
    def __eq__(self, other):
        if not isinstance(other, type(self)): return NotImplemented
        return self.position.x == other.position.x and self.position.y == other.position.y and self.position.z == other.position.z

class Map:
    def __init__(self, init_with_blocks=None, shared_map=None, shared_lock=None) -> None:
        self.current_map = {}
        self.scans = set()
        self.shared_map = shared_map
        self.shared_lock = shared_lock
        if init_with_blocks != None:
            self.current_map.update({x for x in init_with_blocks})
    
    def get_all_blocks_filtered(self, predicate):
        """
        predicate is a function that takes in a string, if the function returns True, the block is kept.
        If False, the block is removed. Does not alter map, just alters the returned blocks from this call.
        """
        return {x for x in self.current_map if predicate(x)}

    
    def add_dummy_block(self, name, position: Vector3):
        with self.shared_lock:
            if self.shared_map is not None:
                block = Block(position, name)
                self.shared_map[(position, "d")] = block

    def remove_dummy_block(self, position: Vector3):
        with self.shared_lock:
            if self.shared_map is not None:
                if self.shared_map.get((position, "d"), None) is not None:
                    if self.shared_map[(position, "d")].voxel is not None:
                        self.shared_map[(position, "d")].voxel.destroy()
                    del self.shared_map[(position, "d")]
    
    def add_block(self, name, position: Vector3):
        m = self.current_map.get(position, None)
        if m is None:
            self.current_map[position] = Block(position, name)
            with self.shared_lock:
                if self.shared_map is not None:
                    block = Block(position, name)
                    self.shared_map[position] = block

    def update_block_type_vis(self, position: Vector3, new_type: str):
        b = self.current_map.get(position, None)
        if b is not None:
            b.type = new_type
        with self.shared_lock:
            if self.shared_map is not None:
                block = self.shared_map.get(position, None)
                if block is not None:
                    self.shared_map[position].type = new_type
                    self.shared_map[position].update_type = True

    def add_scan(self, scan: BlockRotation):
        self.scans.add(scan)

    def already_scanned(self, scan: BlockRotation):
        return scan in self.scans


