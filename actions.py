"""
Define the actions the minecraft agent can do.
Format should be:
Class Action(BaseAction):
    def __init__(self, data1, ..., dataN):
        ...
    def Act(self, agent) -> (Boolean, [actions...]):
        ...
        if is_done(action):
            return True, []
        elif more_actions_required(action):
            return True, [action1, ..., actionN]
        elif loop_back_required(action):
            return True, [action1, ..., actionN, self]
        else:
            return False, []
"""
import ctypes
import time
from math import floor, ceil
from map import MapNode
from pathfinder import Pathfinder
from pathfinder3d import Pathfinder3D
from pathfinder3dunopt import Pathfinder3DUnoptimized
from astar import AStar
import pydirectinput
from minecraft import MinecraftPlayer
import utility


class BaseAction:
    def __init__(self) -> None:
        pass
    
    def act(self, agent: MinecraftPlayer) -> tuple:
        pass

class FastRotationAction(BaseAction):
    def __init__(self, orientation: utility.Vector2, error_tolerance: float=0.2) -> None:
        self.orientation = orientation
        self.error_tolerance = error_tolerance

    def act(self, agent: MinecraftPlayer) -> tuple:
        curr_rotation = agent.rotation.copy()
        # Convert to 360 degree version
        if curr_rotation.x < 0:
            curr_rotation.x = 180 + (180 + curr_rotation.x)
        diff = ( self.orientation.x - curr_rotation.x + 180 ) % 360 - 180
        mouse_x = diff + 360 if diff < -180 else diff
        mouse_y = curr_rotation.y - self.orientation.y

        if abs(mouse_x) < self.error_tolerance:
            mouse_x = 0
        if abs(mouse_y) < self.error_tolerance:
            mouse_y = 0

        # Exact movement
        xspeed = 2400
        yspeed = 600

        mx = utility.linmap(mouse_x, -360, 360, -xspeed, xspeed)
        my = utility.linmap(mouse_y, -90, 90, -yspeed, yspeed)

        if abs(mouse_x) < self.error_tolerance and abs(mouse_y) < self.error_tolerance:
            return True, []
        else:
            mx = ceil(mx)
            my = ceil(my)
            #print(f"moving mouse: {(int(mx), int(my))}")
            ctypes.windll.user32.mouse_event(0x01, int(mx), int(-my), 0, 0)
        return False, []

class VariableRotationAction(BaseAction):
    def __init__(self, orientation: utility.Vector2, speed: float,
            error_tolerance: float=0.2) -> None:
        self.orientation = orientation
        self.error_tolerance = error_tolerance
        self.speed = speed
        self.act_count = 0
        self.previous_error_x = 0
        self.previous_error_y = 0
        self.integral_x = 0
        self.integral_y = 0
        self.last_time = 0
        self.kp = 1
        self.ki = 1
        self.kd = 1

    def act(self, agent: MinecraftPlayer) -> tuple:
        if self.act_count == 0:
            self.last_time = time.time() - (1.0 / 30.0) # pretend we were 1 frame behind.
        # increment this to keep track of how many iterations took place.
        self.act_count += 1

        current_time = time.time()
        dt = current_time - self.last_time
        if dt == 0:
            dt = 1.0 / 60.0
        self.last_time = current_time

        curr_rotation = agent.rotation.copy()
        # Convert to 360 degree version
        if curr_rotation.x < 0:
            curr_rotation.x = 180 + (180 + curr_rotation.x)
        diff = ( self.orientation.x - curr_rotation.x + 180 ) % 360 - 180

        # Proportional
        error_x = diff + 360 if diff < -180 else diff
        error_y = curr_rotation.y - self.orientation.y

        # Integral
        self.integral_x += error_x * dt
        self.integral_y += error_y * dt

        # Derivative
        derivative_x = (error_x - self.previous_error_x) / dt
        self.previous_error_x = error_x
        derivative_y = (error_y - self.previous_error_y) / dt
        self.previous_error_y = error_y

        # PID
        output_x = self.kp * error_x + self.ki * self.integral_x + self.kd * derivative_x
        output_y = self.kp * error_y + self.ki * self.integral_y + self.kd * derivative_y

        # Slower, than precise movement.
        xspeed = 2400 * self.speed
        yspeed = 600 * self.speed

        mx = utility.linmap(output_x, -360, 360, -xspeed, xspeed)
        mx = utility.clamp(mx, -xspeed, xspeed)
        my = utility.linmap(output_y, -90, 90, -yspeed, yspeed)
        my = utility.clamp(my, -yspeed, yspeed)

        # ignore pid if we are already within tolerance (avoid integral wind-up)
        if abs(error_x) < self.error_tolerance:
            mx = 0
        if abs(error_y) < self.error_tolerance:
            my = 0

        if abs(error_x) < self.error_tolerance and abs(error_y) < self.error_tolerance:
            return True, []
        else:
            mx = ceil(mx)
            my = ceil(my)
            #print(f"moving mouse: {(int(mx), int(my))}")
            ctypes.windll.user32.mouse_event(0x01, int(mx), int(-my), 0, 0)
        return False, []

class MoveToCoordinateAction(BaseAction):
    def __init__(self, goal: utility.Vector3) -> None:
        self.goal = goal
        self.start = None
        self.keydown_z = None
        self.keydown_x = None
        self.crouch = False
        self.crouch_tolerance = 0.9

    def act(self, agent: MinecraftPlayer) -> tuple:
        """
        Walk to the given coordinate with no pathfinding.
        """
        if self.start is None:
            self.start = agent.position.subtract(utility.Vector3(0,1,0))
            y_level = self.start.subtract(self.goal).y
            if abs(y_level) < 0.1:
                self.crouch_tolerance = 2
            elif y_level > 0:
                self.crouch_tolerance = 0.5
            else:
                self.crouch_tolerance = 0.5

        curr_rotation = agent.rotation.copy()
        if curr_rotation.x < 0:
            curr_rotation.x = 180 + (180 + curr_rotation.x)

        desired_position_rotated: utility.Vector3 = self.goal.rotate_about_origin_xy(
            self.start, -curr_rotation.x
        )
        curr_position_rotated: utility.Vector3 = agent.position.rotate_about_origin_xy(
            self.start, -curr_rotation.x
        )

        delta_position = desired_position_rotated.subtract(curr_position_rotated)

        delta_position_normal = self.goal.subtract(agent.position.subtract(utility.Vector3(0,1,0)))

        forwards = "w"
        backwards = "s"
        left = "a"
        right = "d"
        crouch = "ctrl"

        # Set tolerances
        tap_duration = 0.03
        tap_tolerance = 0.3
        error_tolerance = 0.05

        # Clean up keys pressed and return True to indicate task completion.
        if abs(delta_position.x) < error_tolerance and abs(delta_position.z) < error_tolerance and abs(delta_position.y + 1) < error_tolerance:
            if self.crouch:
                pydirectinput.keyUp(crouch)
            pydirectinput.keyUp(self.keydown_z)
            pydirectinput.keyUp(self.keydown_x)
            print(f"[TASK] Done moving to coordinates {self.goal}")
            return True, []

        # Crouch if needed
        if delta_position_normal.magnitude() < self.crouch_tolerance:
            if not self.crouch:
                self.crouch = True
                pydirectinput.keyDown(crouch)
        else:
            if self.crouch:
                self.crouch = False
                pydirectinput.keyUp(crouch)

        # Handle forward backwards.
        if abs(delta_position.z) > error_tolerance:
            # Need to move forward or backwards
            if delta_position.z >= 0:
                # Behind point, move forwards
                if abs(delta_position.z) < tap_tolerance:
                    # Tap direction if needed.
                    pydirectinput.keyUp(self.keydown_z)
                    self.keydown_z = None
                    pydirectinput.keyDown(forwards)
                    time.sleep(tap_duration)
                    pydirectinput.keyUp(forwards)
                else:
                    if self.keydown_z != forwards:
                        pydirectinput.keyUp(self.keydown_z)
                        pydirectinput.keyDown(forwards)
                        self.keydown_z = forwards
            else:
                # In front of point, move backwards
                if abs(delta_position.z) < tap_tolerance:
                    # Tap direction if needed.
                    pydirectinput.keyUp(self.keydown_z)
                    self.keydown_z = None
                    pydirectinput.keyDown(backwards)
                    time.sleep(tap_duration)
                    pydirectinput.keyUp(backwards)
                else:
                    if self.keydown_z != backwards:
                        pydirectinput.keyUp(self.keydown_z)
                        pydirectinput.keyDown(backwards)
                        self.keydown_z = backwards

        # Handle left right.
        if abs(delta_position.x) > error_tolerance:
            # Need to move forward or backwards
            if delta_position.x >= 0:
                # Right of point, move left.
                if abs(delta_position.x) < tap_tolerance:
                    # Tap direction if needed.
                    pydirectinput.keyUp(self.keydown_x)
                    self.keydown_x = None
                    pydirectinput.keyDown(left)
                    time.sleep(tap_duration)
                    pydirectinput.keyUp(left)
                else:
                    if self.keydown_x != left:
                        pydirectinput.keyUp(self.keydown_x)
                        pydirectinput.keyDown(left)
                        self.keydown_x = left
            else:
                # Left of point, move right.
                if abs(delta_position.x) < tap_tolerance:
                    # Tap direction if needed.
                    pydirectinput.keyUp(self.keydown_x)
                    self.keydown_x = None
                    pydirectinput.keyDown(right)
                    time.sleep(tap_duration)
                    pydirectinput.keyUp(right)
                else:
                    if self.keydown_x != right:
                        pydirectinput.keyUp(self.keydown_x)
                        pydirectinput.keyDown(right)
                        self.keydown_x = right
                
        return False, []

class Pathfind2DAction(BaseAction):
    def __init__(self, goal: utility.Vector3) -> None:
        self.goal = goal
        self.state = "scan" # either "scan" or "move"
        self.start = None
        self.pathfinder = None
        self.coordinate_tolerance = 0.5

    def act(self, agent: MinecraftPlayer) -> tuple:
        if self.start is None:
            self.start = agent.position.copy()
        if self.pathfinder is None:
            self.pathfinder = Pathfinder(self.start.copy(), self.goal.copy())
        
        current_position = agent.position.copy()
        
        # Convert from floats to grid.
        block_position = agent.position.copy()
        block_position.x = min(floor(block_position.x), ceil(block_position.x))
        block_position.y = min(floor(block_position.y), ceil(block_position.y))
        block_position.z = min(floor(block_position.z), ceil(block_position.z))

        delta_position = self.goal.subtract(current_position)

        if abs(delta_position.x) < self.coordinate_tolerance\
            and abs(delta_position.z) < self.coordinate_tolerance:
            print(f"[INFO] Pathfinding complete, arrived at {self.goal} with error {delta_position}")
            return True, []

        if self.state == "scan":
            # Search area for coordinates and obstacles (defined by 3x3 ground + walls on each side of 3x3 ground)
            # We can use the insertion of rotation events + reinsertion of pathfind event at index 1 afterwards to interrupt the current process
            # and start the rotation events, after those complete, return to pathfinding.
            self.state = "move"

            # Recall that player position = block_on.y + 1
            scans = utility.get_neighboring_blocks(block_position)

            # Remove scans that we already did and generate new rotation events.
            events = [
                FastRotationAction(x.rotation) for x in scans
                    if not x.position in agent.map.current_map\
                        and not agent.map.already_scanned(x)
            ]
            # insert loopback event after scan rotations.
            events.append(self)

            # record the scans we did in the map.
            for x in scans:
                agent.map.add_scan(x)
            
            return True, events
        else:
            # Given the map and current position, determine the best position to pick.
            # If current position is completely blocked, set current position to blocked and backtrack.
            # We can block the position by inserting a block into the map with id "blocked coordinate" and the given position.y+2.
            self.state = "scan"
            neighbors = utility.get_neighboring_blocks_dict(block_position)

            choices = []
            blocked_neighbors = []

            # Remove blocked neighbors from choices
            for ne in neighbors:
                # This is the direction
                n = neighbors[ne]
                # Check that node does not have blocks blocking it.
                blocked = False
                hole = False
                for nw in n["rest"]:
                    blocked = blocked or nw.position in agent.map.current_map
                    if nw.position in agent.map.current_map:
                        print(f"Block {nw.position} blocks {n['start'].position}.")
                
                # Check that node is not a hole
                hole = hole or n["start"].position not in agent.map.current_map
                if n["start"].position not in agent.map.current_map:
                    print(f"Block {n['start'].position} does not exist to stand on.")
                if not (blocked or hole):
                    choices.append(n["start"])
                else:
                    blocked_neighbors.append(n["start"])
            
            # Exit if we run out of possible paths
            if len(choices) <= 0:
                print(f"[ERROR] Pathfinding failed to find path to {self.goal}, all neighboring blocks are blocked at coordinate {current_position}")
                print(f"Map: {agent.map.current_map}")
                return True

            for bl in blocked_neighbors:
                agent.map.add_block("PATHFINDING_BLOCKED", bl.position.copy())
                agent.map.add_block("PATHFINDING_BLOCKED", bl.position.copy().add(utility.Vector3(0,1,0)))

            blocked_neighbors = [x.position.copy() for x in blocked_neighbors]

            # Using d*-lite iterate func which will add any new obstacles to the map, then get the next position.
            choice = self.pathfinder.iterate(blocked_neighbors)

            if choice is None:
                print(f"[INFO] Pathfinding complete, arrived at {self.goal} with error {delta_position}")
                return True, []

            print(f"Adding pathfind node {choice}")
            events = [MoveToCoordinateAction(choice.copy())]
            events.append(self)
            
            return True, events

class Pathfind3DAction(BaseAction):
    def __init__(self, goal: utility.Vector3) -> None:
        self.goal = goal
        self.state = "scan"
        self.is_first_iter = True
        self.start = None
        self.pathfinder = None
        self.edge_costs_prior = {}

    def act(self, agent: MinecraftPlayer) -> tuple:
        current_position = agent.position.subtract(utility.Vector3(0, 1, 0))
        
        # Convert from floats to grid.
        block_position = agent.position.subtract(utility.Vector3(0, 1, 0))
        block_position.x = min(floor(block_position.x), ceil(block_position.x))
        block_position.y = min(floor(block_position.y), ceil(block_position.y))
        block_position.z = min(floor(block_position.z), ceil(block_position.z))

        if self.start is None:
            self.start = block_position.copy()
        if self.pathfinder is None:
            self.pathfinder = AStar(
                self.start.copy(),
                self.goal.copy(),
                agent.qmap
            )

        delta_position = self.goal.subtract(block_position)

        if abs(delta_position.x) < 0.8\
            and abs(delta_position.z) < 0.8\
            and abs(delta_position.y) < 0.8:
            print(f"[INFO] Pathfinding complete, arrived at {self.goal} with error {delta_position}")
            agent.qmap.clear_pathfinding_values()
            return True, []

        if self.state == "scan":
            print("scan")
            self.state = "move"
            # Record edge costs of neighboring states.
            if not agent.qmap.recording:
                agent.qmap.record_edge_cost_changes()
            successors = self.pathfinder.m_start.successors(agent.qmap)
            blocks = set()
            for s_prime in successors:
                # add block to set to be scanned plus its air nodes.
                y_level = s_prime.position.subtract(self.pathfinder.m_start.position)
                blocks.add(s_prime.position)
                blocks.add(s_prime.position.add(utility.Vector3(0, 1, 0)))
                blocks.add(s_prime.position.add(utility.Vector3(0, 2, 0)))
                if y_level != 0:
                    blocks.add(s_prime.position.add(utility.Vector3(0, 3, 0)))
                if y_level == 1:
                    blocks.add(self.pathfinder.m_start.position.add(utility.Vector3(0, 3, 0)))

            # Add scans and loopback to queue.
            # for all successors, add successor to blocks, followed by 2-3 extra scans if needed.
            scans = [(x, agent.look_at(utility.get_middle_of_block(x))) for x in blocks]
            events = []
            for block, scan in scans:
                new_scan = utility.BlockRotation(scan, block)
                qmap_result = agent.qmap.get(block)
                if len([x for x in self.pathfinder.m_start.scans if x.position == block]) == 0:
                    agent.qmap.add_scan(self.pathfinder.m_start.position, new_scan)
                    if qmap_result is not None:
                        if qmap_result.block_type != "uncertain":
                            continue # skip a known block, still add scan since we know whats there
                    events.append(FastRotationAction(scan))
                    
            events.append(self)
            return True, events
        else:
            print("move")
            self.state = "scan"
            # Based on the scan, insert knowledge nodes into map (only air nodes for now)
            scans = self.pathfinder.m_start.scans
            for scan in scans:
                # We can tell if a scan resolved an air node by the what exists in the map at that point.
                element = agent.qmap.get(scan.position)
                #print(f"querying scan-{scan.position}: {element}")
                if element is None:
                    agent.qmap.add(MapNode("air", scan.position, []))
                elif element.block_type == "uncertain":
                    agent.qmap.update_type(element.position, "air", True)

            # Based on the scan and knowledge nodes, get new costs for neighbors.
            # Iterate d*-lite scan with nodes that changed cost.
            previous_state = self.pathfinder.m_start
            changed_node_pairs = agent.qmap.calculate_node_cost_changes()
            if not agent.qmap.recording:
                agent.qmap.record_edge_cost_changes()
            self.pathfinder.iterate_scan(changed_node_pairs)
            if self.is_first_iter:
                self.is_first_iter = False
                self.pathfinder.compute_shortest_path()
                

            # Get move from d*-lite move
            next_state = self.pathfinder.iterate_move()
            print(f"Moving to state: {next_state.position}-{next_state.block_type} with cost {previous_state.cost(next_state, agent.qmap)}")

            if next_state is None:
                print(f"Arrived at goal {self.goal}")
                agent.qmap.clear_pathfinding_values()
                return True, []

            # Add move and loopback to queue unless at goal.
            events = [
                FastRotationAction(agent.look_at(utility.get_middle_of_block(next_state.position))),
                MoveToCoordinateAction(utility.get_middle_top_of_block(next_state.position)),
                self
            ]

            return True, events

class Pathfind3DAStarAction(BaseAction):
    def __init__(self, goal: utility.Vector3) -> None:
        self.goal = goal
        self.state = "scan"
        self.is_first_iter = True
        self.start = None
        self.pathfinder = None
        self.edge_costs_prior = {}

    def act(self, agent: MinecraftPlayer) -> tuple:
        current_position = agent.position.subtract(utility.Vector3(0, 1, 0))
        
        # Convert from floats to grid.
        block_position = agent.position.subtract(utility.Vector3(0, 1, 0))
        block_position.x = min(floor(block_position.x), ceil(block_position.x))
        block_position.y = min(floor(block_position.y), ceil(block_position.y))
        block_position.z = min(floor(block_position.z), ceil(block_position.z))

        if self.start is None:
            self.start = block_position.copy()
        if self.pathfinder is None:
            self.pathfinder = AStar(
                self.start.copy(),
                self.goal.copy(),
                agent.qmap
            )

        delta_position = self.goal.subtract(block_position)

        if abs(delta_position.x) < 0.8\
            and abs(delta_position.z) < 0.8\
            and abs(delta_position.y) < 0.8:
            print(f"[INFO] Pathfinding complete, arrived at {self.goal} with error {delta_position}")
            agent.qmap.clear_pathfinding_values()
            return True, []

        if self.state == "scan":
            print("scan")
            self.state = "move"
            # Record edge costs of neighboring states.
            if not agent.qmap.recording:
                agent.qmap.record_edge_cost_changes()
            successors = self.pathfinder.m_start.successors(agent.qmap)
            blocks = set()
            for s_prime in successors:
                # add block to set to be scanned plus its air nodes.
                y_level = s_prime.position.subtract(self.pathfinder.m_start.position)
                blocks.add(s_prime.position)
                blocks.add(s_prime.position.add(utility.Vector3(0, 1, 0)))
                blocks.add(s_prime.position.add(utility.Vector3(0, 2, 0)))
                if y_level != 0:
                    blocks.add(s_prime.position.add(utility.Vector3(0, 3, 0)))
                if y_level == 1:
                    blocks.add(self.pathfinder.m_start.position.add(utility.Vector3(0, 3, 0)))

            # Add scans and loopback to queue.
            # for all successors, add successor to blocks, followed by 2-3 extra scans if needed.
            scans = [(x, agent.look_at(utility.get_middle_of_block(x))) for x in blocks]
            events = []
            for block, scan in scans:
                new_scan = utility.BlockRotation(scan, block)
                qmap_result = agent.qmap.get(block)
                if len([x for x in self.pathfinder.m_start.scans if x.position == block]) == 0:
                    agent.qmap.add_scan(self.pathfinder.m_start.position, new_scan)
                    if qmap_result is not None:
                        if qmap_result.block_type != "uncertain":
                            continue # skip a known block, still add scan since we know whats there
                    events.append(FastRotationAction(scan))
                    
            events.append(self)
            return True, events
        else:
            print("move")
            self.state = "scan"
            # Based on the scan, insert knowledge nodes into map (only air nodes for now)
            scans = self.pathfinder.m_start.scans
            for scan in scans:
                # We can tell if a scan resolved an air node by the what exists in the map at that point.
                element = agent.qmap.get(scan.position)
                #print(f"querying scan-{scan.position}: {element}")
                if element is None:
                    agent.qmap.add(MapNode("air", scan.position, []))
                elif element.block_type == "uncertain":
                    agent.qmap.update_type(element.position, "air", True)

            # Based on the scan and knowledge nodes, get new costs for neighbors.
            # Iterate d*-lite scan with nodes that changed cost.
            previous_state = self.pathfinder.m_start
            changed_node_pairs = agent.qmap.calculate_node_cost_changes()
            if not agent.qmap.recording:
                agent.qmap.record_edge_cost_changes()
            self.pathfinder.iterate_scan(changed_node_pairs)

            # Get path from A*
            next_states = self.pathfinder.iterate_move()
            events = []
            for next_state in next_states:
                print(f"Moving to state: {next_state.position}-{next_state.block_type} with cost {previous_state.cost(next_state, agent.qmap)}")
                events.append(FastRotationAction(agent.look_at(utility.get_middle_of_block(next_state.position))))
                events.append(MoveToCoordinateAction(utility.get_middle_top_of_block(next_state.position)))

            # Add move and loopback to queue unless at goal.
            events.append(self)

            return True, events

class ClickAction(BaseAction):
    def __init__(self, msb: str, duration: float) -> None:
        self.msb = msb
        self.duration = duration
    
    def act(self, agent: MinecraftPlayer) -> tuple:
        if self.msb == "right":
            pydirectinput.rightClick(duration=self.duration)
            print("righclicked")
        elif self.msb == "left":
            pydirectinput.leftClick(duration=self.duration)
            print("righclicked")
        return True, []

class LookAtAction(BaseAction):
    def __init__(self, position: utility.Vector3) -> None:
        self.position = position
    
    def act(self, agent: MinecraftPlayer) -> tuple:
        rt = agent.look_at(self.position)
        return True, [FastRotationAction(rt)]
