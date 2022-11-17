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
        return s.position.manhattan_distance(s_prime.position)

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
    
    def iterate_scan(self, changed_node_pairs):
        if len(changed_node_pairs) != 0:
            self.map_changed = True

    def iterate_move(self):
        """
        Iterates the pathfinding algorithm after each move/scan.
        Ensures that we only replan if the map changed.
        @param current_state - Where we are starting the pathfinding from.
        @param map_changed - True if the map changed, False otherwise.

        @returns A MapNode element giving the next move to make. 
        """
        if self.map_changed:
            self.map.clear_pathfinding_values()
            self.current_path = self.compute_shortest_path()

            if self.current_path is None:
                raise "No path exists..."

        if len(self.current_path) == 0:
            raise "You were already at the goal"
        
        next_step = self.current_path[1]
        self.current_path = self.current_path[2:]

        self.m_start = next_step

        return next_step


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

                



