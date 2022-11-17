from ursina import *

from threading import Thread, Lock, Event
import datacollector
from minecraft import *


app = Ursina()

# Define a Voxel class.
# By setting the parent to scene and the model to 'cube' it becomes a 3d button.

class Voxel(Button):
    def __init__(self, position=(0,0,0), colorrgb=(255, 255, 255), alpha=255):
        super().__init__(
            parent = scene,
            position = position,
            model = 'cube',
            origin_y = .5,
            texture = 'white_cube',
            color = color.rgba(colorrgb[0], colorrgb[1], colorrgb[2], alpha),
            highlight_color = color.lime,
        )


    def input(self, key):
        if self.hovered:
            if key == 'left mouse down':
                voxel = Voxel(position=self.position + mouse.normal)

            if key == 'right mouse down':
                destroy(self)

    def destroy(self):
        destroy(self)


def update():
    global free_roam, pressed_last_frame

    direction = Vec3(
            camera.forward * (held_keys['w'] - held_keys['s'])
            + camera.right * (held_keys['d'] - held_keys['a'])
            ).normalized()
    camera.x += direction.x * 0.1
    camera.z += direction.z * 0.1
    camera.y += direction.y * 0.1
    #camera.x += held_keys["d"] * 0.2
    #camera.x -= held_keys["a"] * 0.2
    #camera.z += held_keys["w"] * 0.2
    #camera.z -= held_keys["s"] * 0.2
    camera.y += held_keys["space"] * 0.2
    camera.y -= held_keys["shift"] * 0.2

    #camera.rotation_x += mouse.velocity[1] * 800
    #camera.rotation_y -= mouse.velocity[0] * 500

    camera.rotation_x += held_keys["down arrow"] * 0.8
    camera.rotation_x -= held_keys["up arrow"] * 0.8

    camera.rotation_y += held_keys["right arrow"] * 0.8
    camera.rotation_y -= held_keys["left arrow"] * 0.8

    if held_keys["f"] != 0 and not pressed_last_frame:
        pressed_last_frame = True
        free_roam = not free_roam
        print(f"Free_Roam: {free_roam}")
    else:
        pressed_last_frame = False


    sx = 0
    sy = 0
    sz = 0
    with shared_lock:
        for key in shared_map:
            block = shared_map[key]
            sx += block.position.x
            sy += block.position.y
            sz += block.position.z
            if block.update_type and block.voxel is not None:
                block.update_type = False
                colorrgb=[(ord(c.lower())-97)*8 for c in block.type[:3]]
                alpha = 255
                if block.type == "uncertain":
                    alpha = 20
                if block.type == "air":
                    alpha = 60
                block.voxel.color = color.rgba(0, 0, 255, alpha)
            if not block.voxel:
                #print(f"Instantiating block {block}")
                if block.type == "uncertain":
                    #pass
                    block.voxel = Voxel(position=(-block.position.x, block.position.y, block.position.z), colorrgb=[255, 0, 0], alpha=30)
                elif block.type == "air":
                    pass
                    #block.voxel = Voxel(position=(-block.position.x, block.position.y, block.position.z), colorrgb=[0,0,255], alpha=40)
                elif block.type == "path_node":
                    block.voxel = Voxel(position=(-block.position.x, block.position.y, block.position.z), colorrgb=[255,255,255], alpha=255)
                    block.voxel.scale = (0.2, 0.2, 0.2)
                else:
                    if len(block.type) < 3:
                        block.type += "extra"
                    block.voxel = Voxel(position=(-block.position.x, block.position.y, block.position.z), colorrgb=[(ord(c.lower())-97)*8 for c in block.type[:3]])
                block.instantiated = True
        sl = len(shared_map)
        #print("Num Blocks ", sl)
        if sl != 0 and not free_roam:
            sx /= sl
            sy /= sl
            sz /= sl
            camera.x = sx - 20
            camera.y = sy + 20
            camera.z = sz - 20

    with player_lock:
        player.x = -player_position.x + 0.25
        player.y = player_position.y - 0.25
        player.z = player_position.z -0.5


        player_eyes.x = -player_position.x + 0.25
        player_eyes.y = player_position.y + 1 - 0.5
        player_eyes.z = player_position.z-0.5

        forward.x = -player_forward.x - 0.15
        forward.y = player_forward.y - 0.15
        forward.z = player_forward.z - 0.15

        #player.parent = camera
        #camera.origin = player
        #camera.look_at(player_eyes)

        camera.x = -player_position.x - 20
        camera.y = player_position.y + 20
        camera.z = player_position.z - 20



shared_lock = Lock()
shared_map = {}

player_lock = Lock()
player_position = Vector3(0,0,0)
player_forward = Vector3(0,0,0)

existing_blocks = set()

running = Event()
running.set()



voxel = Voxel(position=(0,-60,0))

player = Entity(model='cube', position=(0,-60,0), color=color.rgb(255,255,255), scale=(1, 1, 1)) #Voxel(position=(0,-60,0))
player_eyes = Entity(model='cube', position=(0,-59,0), color=color.rgb(200,200,200), scale=(1, 1, 1))
forward = Entity(model='cube', position=(0,-60,1), color=color.rgb(200,0,0), scale=(0.3, 0.3, 0.3))
free_roam = False
pressed_last_frame = False

camera.y = -50
camera.rotation_x = 30
camera.rotation_y = 50

window.borderless = False

window.windowed_size = 0.3
window.update_aspect_ratio()
window.late_init()

window.position = Vec2(3500, 1920/3)
Sky()

data = Thread(target=datacollector.start, args=(shared_map, shared_lock, running, player_position, player_forward, player_lock))

data.start()
try:
    app.run()
except:
    print("[Error]")
    running.clear()
    data.join()
