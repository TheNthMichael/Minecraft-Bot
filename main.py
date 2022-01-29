from math import ceil, cos, sin, radians
import tesserocr
from tesserocr import PyTessBaseAPI, get_languages, PSM, OEM, RIL
tesserocr.PyTessBaseAPI(path='C:\\Program Files\\Tesseract-OCR\\tessdata\\')
# OCR Screen Scanner
# By Dornu Inene
# Libraries that you show have all installed
import cv2
import ctypes
import numpy as np
import re
import time
import string
import pydirectinput

# We only need the ImageGrab class from PIL
from PIL import ImageGrab, Image

from utility import *


def move_mouse(x, y):
    ctypes.windll.user32.mouse_event(0x01, x, y, 0, 0)


def walk_square(player: MinecraftPlayer, n):
    origin = Vector3(1.5, -60, -27.5)

    player.add_coordinates_to_queue(origin)

    player.add_coordinates_to_queue(Vector3(origin.x, origin.y, origin.z + n))
    player.add_coordinates_to_queue(Vector3(origin.x - n, origin.y, origin.z + n))
    player.add_coordinates_to_queue(Vector3(origin.x - n, origin.y, origin.z))
    player.add_coordinates_to_queue(Vector3(origin.x, origin.y, origin.z))


def make_square(player: MinecraftPlayer, n):
    """
    Constructs an nxn square.
    """
    assert n > 2
    player.add_coordinates_to_queue(Vector3(0,0,0))

    for i in range(0, n+1):
        player.add_coordinates_to_queue(Vector3(i, 0, 0))
        player.add_rotation_to_queue(Vector2(0, 60))
        player.add_click_to_queue("right")

    player.add_coordinates_to_queue(Vector3(n+1, 0, 0))
    player.add_coordinates_to_queue(Vector3(n+1, 0, 2))

    for i in range(2, n+1):
        if i == 2:
            i += 1
        player.add_coordinates_to_queue(Vector3(n+1, 0, i))
        player.add_rotation_to_queue(Vector2(90, 60))
        player.add_click_to_queue("right")
    
    player.add_coordinates_to_queue(Vector3(n+1, 0, n+1))
    player.add_coordinates_to_queue(Vector3(n-1, 0, n+1))

    for i in range(2, n+1):
        player.add_coordinates_to_queue(Vector3(n-i, 0, n+1))
        player.add_rotation_to_queue(Vector2(180, 60))
        player.add_click_to_queue("right")

    player.add_coordinates_to_queue(Vector3(-1, 0, n+1))
    player.add_coordinates_to_queue(Vector3(-1, 0, n-1))

    for i in range(2, n):
        if i == 2:
            i = 1.5
        player.add_coordinates_to_queue(Vector3(-1, 0, n-i))
        player.add_rotation_to_queue(Vector2(-90, 60))
        player.add_click_to_queue("right")
    


bb_coords = (45, 210, 400, 230)
bb_rotation = (75, 265, 550, 285)
bb_block_coords = (2560-400, 370, 2560, 391)
bb_block_type = (2560-500, 393, 2560, 411)
#bb_block_type = (2560-250, 393, 2560, 412)


player = MinecraftPlayer(bb_coords, bb_rotation, bb_block_coords, bb_block_type)
print(f"Loaded Languages:\n", get_languages('C:\\Program Files\\Tesseract-OCR\\tessdata\\'))

with PyTessBaseAPI(lang='mc', psm=13, oem=3) as api:
    api.SetVariable("load_freq_dawg", "false")
    api.SetVariable("load_system_dawg", "false")

    time.sleep(1)
    print("Starting")

    # Run forever unless you press Esc
    while True:

        success = player.update(api)

        height, width = player.current_position_image.shape
        resized_rotation_image = cv2.resize(player.current_rotation_image, (width, height))
        resized_block_position_image = cv2.resize(player.current_block_position_image, (width, height))
        resized_block_type_image = cv2.resize(player.current_block_type_image, (width, height))
        frame = np.concatenate((player.current_position_image, resized_rotation_image, resized_block_position_image, resized_block_type_image), axis=0)


        cv2.imshow("player", frame)

        # This line will break the while loop when you press Esc
        if cv2.waitKey(1) == 27:
            break

# This will make sure all windows created from cv2 is destroyed
cv2.destroyAllWindows()