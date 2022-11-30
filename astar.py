from priority_queue import PriorityQueue, Priority
from map import QueryMap, MapNode
from utils import *
from utility import Vector3


class AStar:
    _MAX_PATH_LENGTH = 100

    def __init__(self, start: Vector3, goal: Vector3, map: QueryMap) -> None:
        """
        Initialize the d*-lite algorithm.
        """
        self.current_path = []
        self.map_changed = True
        self.map = map
        self.start = start
        self.m_start = map.get(start)
        if self.m_start is None:
            self.m_start = MapNode("ground", start, [])
            print(f"Starting pathfinder from node {self.m_start.position}-{self.m_start.block_type}")
            self.map.add(self.m_start)

        self.goal = goal
        self.m_goal = map.get(goal)
        if self.m_goal is None:
            self.m_goal = MapNode("uncertain", goal, [])
            map.add(self.m_goal)

    def heuristic(self, s: MapNode, s_prime: MapNode):
        #return s.position.distance(s_prime.position)
        #return s.position.manhattan_distance(s_prime.position)
        dv = s.position.subtract(s_prime.position)
        dx = abs(dv.x)
        dy = abs(dv.y)
        dz = abs(dv.z)
        return (dx + (2**0.5 * dy) + dz)

    def get_best_node_index(self):
        if len(self.open) == 0:
            raise "Cannot find the best node of an empty list."
        smallest_f = float('inf')
        best_node = 0
        for idx, node in enumerate(self.open):
            new_f = node.g + self.heuristic(node, self.m_goal)
            if new_f < smallest_f:
                smallest_f = new_f
                best_node = idx

        return best_node

    def get_dir(self, v: Vector3):
        if v.x + v.y + v.z != 1:
            return -1
        elif v.x == 1:
            return 0
        elif v.y == 1:
            return 1
        elif v.z == 1:
            return 2
        else:
            raise "Not Valid"

    def linearize_path(self, start: MapNode, path: list):
        # let x = 0, y = 1, z = 2
        # There's no point in doing this on paths of 2 or less nodes. 
        if len(path) <= 2:
            return path
        result = []
        dv = path[1].position.subtract(start.position)
        dv.apply_func_to_all(abs)
        last_dir = self.get_dir(dv)
        if last_dir == -1:
            result.append(path[1])

        for i in range(len(path) - 1):
            curr = path[i]
            next = path[i+1]
            dv = curr.position.subtract(next.position)
            dv.apply_func_to_all(abs)
            dir = self.get_dir(dv)
            if dir == -1:
                result.append(curr)
                print("HERE")
            elif last_dir != dir:
                result.append(curr)
                last_dir = dir
        result.append(path[-1])
        #result = [*set(result)] # Shouldn't be needed, unsure if there is a weird edge case.
        return result

    
    def iterate_scan(self, changed_node_pairs):
        if not self.m_goal.successor_path_exists(self.map):
            raise "No path exists..."
        if len(changed_node_pairs) != 0:
            self.map_changed = True

    def iterate_move(self):
        """
        Iterates the pathfinding algorithm after each move/scan.
        Ensures that we only replan if the map changed.
        @param current_state - Where we are starting the pathfinding from.
        @param map_changed - True if the map changed, False otherwise.

        @returns A list of linearized MapNode elements that we can traverse without needing to rescan.
        """
        if self.map_changed:
            self.map.clear_pathfinding_values()
            self.current_path = self.compute_shortest_path()

            if self.current_path is None or not self.m_goal.successor_path_exists(self.map):
                raise "No path exists..."

        if len(self.current_path) == 0:
            raise "You were already at the goal"

        path = self.current_path[1:]

        #print(path)

        """last_index = 0
        for idx, node in enumerate(path):
            if len(node.get_scans_required(self.map)) != 0:
                last_index = idx
                break
            last_index = idx"""

        """print(last_index)
        # Get the subpath.
        sub_path = path[:last_index + 1]
        print(sub_path)

        # Linearize the path such that we remove nodes between points that are straight lines.
        linearized_sub_path = self.linearize_path(self.m_start, sub_path)

        # Set the start to the node on the last index.
        self.m_start = path[last_index]

        # Set the current path to last index + 1 to the end.
        self.current_path = path[last_index:]

        print(linearized_sub_path)

        return linearized_sub_path"""

        
        next_step = self.current_path[1]
        self.current_path = self.current_path[2:]

        self.m_start = next_step

        return [next_step]


    def compute_shortest_path(self):
        """
        Compute the shortest path using the A* algorithm.
        """
        print("finding shortest path")
        self.open = []
        self.closed = set()

        self.m_start.g = 0

        self.open.append(self.m_start)

        while len(self.open) > 0:
            best_idx = self.get_best_node_index()
            best_node = self.open[best_idx]
            self.open.pop(best_idx)
            self.closed.add(best_node)

            if best_node == self.m_goal:
                print("found shortest path")
                path = []
                current = best_node
                while current is not None:
                    path.append(current)
                    current = current.backpointer
                return path[::-1]

            successors = best_node.successors(self.map)
            children = []
            for successor in successors:
                if best_node.cost(successor, self.map) == float('inf'):
                    continue
                
                if successor not in self.closed and successor not in self.open:
                    successor.backpointer = best_node
                
                children.append(successor)

            for child in children:
                if child in self.closed:
                    #print(f"skipped {child}")
                    continue

                child.g = best_node.g + 1
                skip = False
                for open_node in self.open:
                    if child == open_node:
                        skip = True
                        if child.g < open_node.g:
                            open_node.g = child.g
                            open_node.backpointer = best_node
                        
                if not skip:
                    self.open.append(child)

                



