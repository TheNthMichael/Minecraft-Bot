from priority_queue import PriorityQueue, Priority
from map import QueryMap, MapNode
from utils import *
from utility import Vector3
import math
# https://github.com/mds1/path-planning/blob/d6ee3e08b1cb2935e1c298ff2fb66af5fd7bc718/all_functions.py#L903 may possibly resolve the looping issue.
class Pathfinder3DUnoptimized:
    def __init__(self, start: Vector3, goal: Vector3, map: QueryMap) -> None:
        """
        Initialize the unoptimized version of the d*-lite algorithm.
        """
        self.current_path = []
        self.U = PriorityQueue()
        self.k_m = 0
        self.map = map
        self.start = start
        self.m_start = map.get(start)
        if self.m_start is None:
            self.m_start = MapNode("ground", start, [])
            print(f"Starting pathfinder from node {self.m_start.position}-{self.m_start.block_type}")
            self.map.add(self.m_start)

        self.m_last = self.m_start
        self.goal = goal
        self.m_goal = map.get(goal)
        if self.m_goal is None:
            self.m_goal = MapNode("uncertain", goal, [])
            map.add(self.m_goal)
        self.m_goal.rhs = 0
        self.m_goal.g = 0
        self.U.insert(self.m_goal, self.calculate_key(self.m_goal))
        #self.compute_shortest_path()

    def calculate_key(self, u: MapNode):
        return Priority(
            min(u.g, u.rhs) + self.heuristic(self.m_start, u) + self.k_m,
            min(u.g, u.rhs)
        )

    def contains(self, u: MapNode) -> bool:
        return u in self.U.vertices_in_heap

    def heuristic(self, s: MapNode, s_prime: MapNode):
        #return s.position.distance(s_prime.position)
        #return s.position.manhattan_distance(s_prime.position)
        dv = s.position.subtract(s_prime.position)
        dx = abs(dv.x)
        dy = abs(dv.y)
        dz = abs(dv.z)
        return max(dx, dy, dz)
        

    def update_vertex(self, u: MapNode):
        if u != self.m_goal:
            costs = [u.cost(s_prime, self.map) + s_prime.g for s_prime in u.successors(self.map, lazy=True)]
            u.rhs = min(costs)
        if self.contains(u):
            self.U.remove(u)
        if not math.isclose(u.g, u.rhs, rel_tol=1e-6):
            self.U.insert(u, self.calculate_key(u))

    def compute_shortest_path(self):
        while self.U.top_key() < self.calculate_key(self.m_start)\
            or not math.isclose(self.m_start.rhs, self.m_start.g, rel_tol=1e-6):
            u = self.U.top()
            k_old = self.U.top_key()
            self.U.remove(u)
            new_key = self.calculate_key(u)
            if k_old < new_key:
                self.U.insert(u, new_key)
            elif u.g > u.rhs:
                u.g = u.rhs
                for s in u.successors(self.map):
                    self.update_vertex(s)
            else:
                u.g = float('inf')
                self.update_vertex(u) # u cannot be a successor to itself.
                for s in u.successors(self.map):
                    self.update_vertex(s)
        print(f"compute_shortest_path expanded {MapNode.count} so far")
        #self.print_shortest_path()

    def iterate_move(self):
        if self.m_start == self.m_goal:
            return self.m_goal
        if self.m_start.g == float('inf'):
            raise "Error, No Path Exists"
        
        print(f"Successors: {[self.m_start.cost(s_prime, self.map) + s_prime.g for s_prime in self.m_start.successors(self.map)]}")
        prev = self.m_start
        costs = [(s_prime, self.m_start.cost(s_prime, self.map) + s_prime.g) for s_prime in self.m_start.successors(self.map)]
        self.m_start, min_cost = min(costs, key=lambda x: x[1])
        # Settle ties:
        ties = [x for x in costs if x[1] == min_cost]
        # break tie as min cost and heuristic.
        self.m_start, min_cost = min(ties, key=lambda x: prev.cost(x[0], self.map) + self.heuristic(prev, x[0]))

        return self.m_start

    def iterate_scan(self, changed_node_pairs):
        if len(changed_node_pairs) != 0:
            self.k_m += self.heuristic(self.m_last, self.m_start)
            self.m_last = self.m_start
            for u, v, c_old in changed_node_pairs:
                self.update_vertex(u)
                self.update_vertex(v)
            self.compute_shortest_path()

    def print_shortest_path(self):
        for e in self.current_path:
            self.map.other_map.remove_dummy_block(e.position)

        self.current_path = []
        current = self.m_start
        prev = current
        while current != self.m_goal:
            if current.rhs == float('inf'):
                raise "NoPathExists"
            prev = current
            costs = [(s_prime, current.cost(s_prime, self.map) + s_prime.g) for s_prime in current.successors(self.map)]
            current, min_cost = min(costs, key=lambda x: x[1])
            # Settle ties:
            #ties = [x for x in costs if x[1] == min_cost]
            # break tie as min cost and heuristic.
            #current, min_cost = min(ties, key=lambda x: prev.cost(x[0], self.map) + self.heuristic(prev, x[0]))

            # We should never have a min g cost that is the same as the last visited node.
            temp_node = current.copy()
            self.map.other_map.add_dummy_block("path_node", temp_node.position)
            self.current_path.append(temp_node)