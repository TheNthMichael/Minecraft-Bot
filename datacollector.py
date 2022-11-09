from math import sqrt
from actions import *
from cv2 import EVENT_MBUTTONUP
import tesserocr
from tesserocr import PyTessBaseAPI, get_languages, PSM, OEM, RIL
from screeninfo import get_monitors
tesserocr.PyTessBaseAPI(path='C:\\Program Files\\Tesseract-OCR\\tessdata\\')
import cv2
import ctypes
import numpy as np
import time
from minecraft import *


def move_mouse(x, y):
    ctypes.windll.user32.mouse_event(0x01, x, y, 0, 0)


def scan_63(player: MinecraftPlayer):
    angles = [
        # Row 1
        Vector2(0, 60),
        Vector2(47, 50),
        Vector2(65, 37),
        Vector2(70, 22),
        Vector2(70, 3),

        # Row 2
        Vector2(90, 60),
        Vector2(90, 40),
        Vector2(90, 22),
        Vector2(90, 3),

        # Row 3
        Vector2(-179, 60),
        Vector2(135, 50),
        Vector2(117, 37),
        Vector2(112, 22),
        Vector2(112, 3),
    ]

    for angle in angles:
        player.add_rotation_to_queue(angle)


def scan_333(player: MinecraftPlayer):
    """
    From the current coordinate, look a the 3x3 cube surrounding the player.
    """
    y_levels = [90, 60, 35, 0, -30, -55, -90]
    x_levels = [0, 30, 50, 90, 120, 144, 180, -140, -188, -90, -50, -30, 0]


    ey_levels = [50, -45]
    ex_levels = [45, 135, -135, -44]

    # Handle front back left right
    for x in x_levels:
        for y in y_levels:
            player.add_rotation_to_queue(Vector2(x, y))

    for x in ex_levels:
        for y in ey_levels:
            player.add_rotation_to_queue(Vector2(x, y))


def walk_fill_square(player: MinecraftPlayer, n):
    """
    Walk over the filled square from current pos to n
    """
    origin = Vector3(0.5, -60, 0.5)

    player.add_coordinates_to_queue(origin)

    # Look down
    player.add_rotation_to_queue(Vector2(0, 0))

    
    for i in range(n):
        for j in range(n):
            player.add_coordinates_to_queue(Vector3(origin.x + i, origin.y, origin.z + j))
            player.add_rotation_to_queue(Vector2(0, 90))
            player.add_rotation_to_queue(Vector2(0, 0))

    player.add_rotation_to_queue(Vector2(0, 0))


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
    player.add_rotation_to_queue(Vector2(0, 0))
    player.add_coordinates_to_queue(Vector3(0.5,-60,0.5))

    for i in range(0, n+1):
        player.add_coordinates_to_queue(Vector3(i, 0, 0))
        player.add_rotation_to_queue(Vector2(0, 50))
        player.add_click_to_queue("right")

    player.add_coordinates_to_queue(Vector3(n+1, 0, 0))
    player.add_coordinates_to_queue(Vector3(n+1, 0, 2))

    for i in range(2, n+1):
        if i == 2:
            i += 1
        player.add_coordinates_to_queue(Vector3(n+1, 0, i))
        player.add_rotation_to_queue(Vector2(90, 50))
        player.add_click_to_queue("right")
    
    player.add_coordinates_to_queue(Vector3(n+1, 0, n+1))
    player.add_coordinates_to_queue(Vector3(n-1, 0, n+1))

    for i in range(2, n+1):
        player.add_coordinates_to_queue(Vector3(n-i, 0, n+1))
        player.add_rotation_to_queue(Vector2(180, 50))
        player.add_click_to_queue("right")

    player.add_coordinates_to_queue(Vector3(-1, 0, n+1))
    player.add_coordinates_to_queue(Vector3(-1, 0, n-1))

    for i in range(2, n):
        if i == 2:
            i = 1.5
        player.add_coordinates_to_queue(Vector3(-1, 0, n-i))
        player.add_rotation_to_queue(Vector2(-90, 50))
        player.add_click_to_queue("right")
    
def user_select_boxes():
    # mouse callback function
    screen = mss.mss()
    monitor = None
    for m in get_monitors():
        if m.is_primary:
            monitor = m
    if monitor is None:
        raise "Error, screeninfo does not see a primary monitor."
    
    print(f"[INFO] detected primary monitor ({monitor.width}x{monitor.height})")

    # Create a black image, a window and bind the function to window
    mon = {"top": 0, "left": 0, "width": monitor.width, "height": monitor.height}
    img = screen.grab(mon)
    img = np.array(img)
    original = img.copy()
    image_coordinates = []
    on_state = True

    state = 0

    states = [
        {
            "type": "coords",
            "bb": None
        },
        {
            "type": "rotation",
            "bb": None
        },
        {
            "type": "block_coords",
            "bb": None
        },
        {
            "type": "block_type",
            "bb": None
        }
    ]

    def draw_circle(event,x,y,flags,param):
        nonlocal image_coordinates, on_state, img, state, states, original
        # Record starting (x,y) coordinates on left mouse button click
        if event == cv2.EVENT_LBUTTONDOWN:
            image_coordinates = [(x,y)]

        if event == cv2.EVENT_MOUSEMOVE:
            if len(image_coordinates) == 1:
                img = original.copy()
                img = cv2.rectangle(img, image_coordinates[0], (x, y), (255,0,0), 2)

        # Record ending (x,y) coordintes on left mouse bottom release
        elif event == cv2.EVENT_LBUTTONUP:
            image_coordinates.append((x,y))

            # Draw rectangle around ROI
            img = cv2.rectangle(img, image_coordinates[0], image_coordinates[1], (0,255,0), 2)
            print(f"ROI: ({image_coordinates[0][0]}, {image_coordinates[0][1]}), ({image_coordinates[1][0]}, {image_coordinates[1][1]})")

        # Clear drawing boxes on right mouse button click
        elif event == cv2.EVENT_RBUTTONDOWN:
            img = original.copy()

        elif event == cv2.EVENT_MBUTTONUP:
            # Move to next state.
            states[state]["bb"] = image_coordinates
            state = state + 1
            img = original.copy()
            on_state = True


    cv2.namedWindow('image')
    cv2.setMouseCallback('image',draw_circle)

    while(1):
        cv2.imshow('image',img)
        if on_state:
            if state >= len(states):
                print(states)
                break
            print(f"Please click on the top left and bottom right of {states[state]['type']} followed by a right click to confirm...")
            on_state = False
        if cv2.waitKey(20) & 0xFF == 27:
            break
    cv2.destroyAllWindows()
    return states


def slwahce_lidar(player: MinecraftPlayer, r):
    """
    slow-*** lidar, scans the r-sphere around the player.
    no optimizations used, hence the name.
    """
    # Start here.
    player.add_rotation_to_queue(Vector2(179.9,0))

    # Don't forget that rotation is really messed up, use conversions.
    # start at 180
    
    # define the new horizon radius.
    nhr = lambda height, radius: sqrt(radius**2 - height**2)
    # dtheta = lambda x: 45 / x
    dtheta = lambda x: 45 / x
    dy_theta = dtheta(r)

    # Upwards
    y_theta = 90
    x_theta = 0
    for i in range(r):
        radius = nhr(i, r)
        for x_theta in range(0, 360, int(dtheta(radius))):
            player.add_smooth_rotation_to_queue(Vector2(x_theta - 180, y_theta - 90), 1)
        y_theta -= dy_theta

    # Downwards
    y_theta = 90
    x_theta = 0
    for i in range(r):
        radius = nhr(i, r)
        for x_theta in range(0, 360, int(dtheta(radius))):
            player.add_smooth_rotation_to_queue(Vector2(x_theta - 180, y_theta - 90), 1)
        y_theta += dy_theta


def fstahce_lidar(player: MinecraftPlayer, r):
    """
    fast-*** lidar, I mean not really but its as good as its gonna get without implementing mouse smoothment.
    Optimization makes use of guessing d-theta per iter based on current block resolution.
    * Reverts to last_theta + min_theta if guess was incorrect. Makes the assumption that large changes in resolution are fairly rare.
    * Wait this doesn't work for blank area's like the sky (as in would scan the sky really slowly).
    * We really don't need to have precise knowledge of the sky though.
    * Could use a guess and branch method. to fill in gaps since we can assume most area's covering the sky cover a decent portion. 
    """
    """
    Ah this requires updating the scan at runtime
    slow-*** lidar, scans the r-sphere around the player.
    no optimizations used, hence the name.
    """
    # Start here.
    player.add_rotation_to_queue(Vector2(179.9,0))

    # Don't forget that rotation is really messed up, use conversions.
    # start at 180
    
    # define the new horizon radius.
    nhr = lambda height, radius: sqrt(radius**2 - height**2)
    # dtheta = lambda x: 45 / x
    dtheta = lambda x: 45 / x
    dy_theta = dtheta(r)

    # Upwards
    y_theta = 90
    x_theta = 0
    for i in range(r):
        radius = nhr(i, r)
        for x_theta in range(0, 360, int(dtheta(radius))):
            player.add_smooth_rotation_to_queue(Vector2(x_theta - 180, y_theta - 90), 1)
        y_theta -= dy_theta

    # Downwards
    y_theta = 90
    x_theta = 0
    for i in range(r):
        radius = nhr(i, r)
        for x_theta in range(0, 360, int(dtheta(radius))):
            player.add_smooth_rotation_to_queue(Vector2(x_theta - 180, y_theta - 90), 1)
        y_theta += dy_theta



def start(shared_map, shared_lock, running, shared_player_position, player_forward, shared_player_position_lock):
    mode = "s"
    if mode == "s":
        # SinglePlayer
        bb_coords = (45, 210, 400, 231)
        bb_rotation = (75, 265, 550, 285)
        bb_block_coords = (2560-400, 370, 2560, 391)
        bb_block_type = (2560-500, 393, 2560, 411)
    elif mode == "m":
        # Multiplayer
        bb_coords = (45, 190, 400, 210)
        bb_rotation = (75, 245, 550, 265)
        bb_block_coords = (2560-400, 370, 2560, 391)
        bb_block_type = (2560-500, 393, 2560, 411)
    else:
        # Manual entry.
        states = user_select_boxes()
        bb_coords = (states[0]["bb"][0][0], states[0]["bb"][0][1], states[0]["bb"][1][0], states[0]["bb"][1][1])
        bb_rotation = (states[1]["bb"][0][0], states[1]["bb"][0][1], states[1]["bb"][1][0], states[1]["bb"][1][1])
        bb_block_coords = (states[2]["bb"][0][0], states[2]["bb"][0][1], states[2]["bb"][1][0], states[2]["bb"][1][1])
        bb_block_type = (states[3]["bb"][0][0], states[3]["bb"][0][1], states[3]["bb"][1][0], states[3]["bb"][1][1])

    #bb_block_type = (2560-250, 393, 2560, 412)


    player = MinecraftPlayer(bb_coords, bb_rotation, bb_block_coords, bb_block_type, shared_map, shared_lock)
    print(f"Loaded Languages:\n", get_languages('C:\\Program Files\\Tesseract-OCR\\tessdata\\'))

    with PyTessBaseAPI(lang='mc', psm=13, oem=3) as api:
        api.SetVariable("load_freq_dawg", "false")
        api.SetVariable("load_system_dawg", "false")

        time.sleep(3)
        print("Starting")

        player.add_action(
            FastRotationAction(Vector2(0, 90))
        )

        player.add_action(
            # Pathfind3DAction(Vector3(46.4, -55, -9.5))
            Pathfind3DAction(Vector3(62, -61, 30))
        )


        #player.add_action(MoveToCoordinateAction(Vector3(-8.5, -60, -27.5)))
    
        # used to record the time when we processed last frame
        prev_frame_time = 0
        
        # used to record the time at which we processed current frame
        new_frame_time = 0

        # Run forever unless you press Esc
        while running.is_set():

            success = player.update(api)

            with shared_player_position_lock:
                shared_player_position.x = player.position.x
                shared_player_position.y = player.position.y
                shared_player_position.z = player.position.z
                player_forward.x = player.forward_position.x
                player_forward.y = player.forward_position.y
                player_forward.z = player.forward_position.z

            height, width = player.current_position_image.shape
            #resized_rotation_image = cv2.resize(player.current_rotation_image, (width, height))
            #resized_block_position_image = cv2.resize(player.current_block_position_image, (width, height))
            #resized_block_type_image = cv2.resize(player.current_block_type_image, (width, height))
            
            new_frame_time = time.time()
            fps = 1/(new_frame_time-prev_frame_time)
            prev_frame_time = new_frame_time
        
            # converting the fps into integer
            fps = int(fps)
        
            # converting the fps to string so that we can display it on frame
            # by using putText function
            #fps = "fps: "+ str(fps)
            #empty = np.zeros((height, width))
            #cv2.putText(empty, fps, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

            #images = (empty, player.current_position_image, resized_rotation_image, resized_block_position_image, resized_block_type_image)

            #frame = np.concatenate(images, axis=0)

            #cv2.imshow("player", frame)

            # This line will break the while loop when you press Esc
            #if cv2.waitKey(1) == 27:
                #break

    # This will make sure all windows created from cv2 is destroyed
    #cv2.destroyAllWindows()