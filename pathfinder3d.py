from re import L
from d_star_lite import computeShortestPath
from priority_queue import PriorityQueue, Priority
from map import QueryMap, MapNode
from utils import *
from utility import Vector3


class Pathfinder3D:
    _MAX_PATH_LENGTH = 100

    def __init__(self, start: Vector3, goal: Vector3, map: QueryMap) -> None:
        """
        Initialize the d*-lite algorithm.
        """
        self.U = PriorityQueue()
        self.k_m = 0

        self.start = start
        self.m_start = map.get(start)
        self.m_last = self.m_start
        if self.m_start is None:
            raise "Error, cannot pathfind without knowing where the ground is (Scan downward before starting)"

        self.goal = goal
        self.m_goal = map.get(goal)
        if self.m_goal is None:
            self.m_goal = MapNode("uncertain", goal, [])
            self.m_goal.rhs = 0
            map.add(self.m_goal)
        self.U.insert(self.m_goal, Priority(self.heuristic(self.m_start, self.m_goal), 0))
        self.compute_shortest_path()

    def contains(self, u: tuple) -> bool:
        return u in self.U.vertices_in_heap

    def heuristic(self, s: MapNode, s_prime: MapNode):
        return s.position.distance(s_prime.position)
    
    def calculate_key(self, u: MapNode):
        return Priority(
            min(u.g, u.rhs) + self.heuristic(self.m_start, u) + self.k_m,
            min(u.g, u.rhs)
        )

    def update_vertex(self, u: MapNode):
        u_contains = self.contains(u)
        if u.g != u.rhs and u_contains:
            self.U.update(u, self.calculate_key(u))
        elif u.g != u.rhs and not u_contains:
            self.U.insert(u, self.calculate_key(u))
        elif u.g == u.rhs and u_contains:
            self.U.remove(u)

    def compute_shortest_path(self):
        while self.U.top_key() < self.calculate_key(self.m_start) or self.m_start.rhs > self.m_start.g:
            u = MapNode(self.U.top())
            k_old = self.U.top_key()
            k_new = self.calculate_key(u)

            if k_old < k_new:
                self.U.update(u, k_new)
            elif u.g > u.rhs:
                u.g = u.rhs
                self.U.remove(u)
                for s in u.successors(self.map):
                    if s != self.m_goal:
                        s.rhs = min(s.rhs, s.cost(u, self.map) + u.g)
                    self.update_vertex(s)
            else:
                g_old = u.g
                u.g = float('inf')
                predecessors_union_u = u.successors(self.map).append(u) # u cannot be its own successor therefore no check for that is needed.
                for s in predecessors_union_u:
                    if s.rhs == s.cost(u, self.map) + g_old:
                        if s != self.m_goal:
                            costs = [s.cost(s_prime, self.map) + s_prime.g for s_prime in s.successors]
                            s.rhs = min(costs)
                        self.update_vertex(s)

    def iterate_move(self):
        """
        Decomposed the main loop of d*-lite into one that I can manually iterate.
        """
        if self.m_start == self.m_goal:
            return self.m_goal
        if self.m_start.rhs == float('inf'):
            raise "NoPathExists"
        
        self.m_start = min(self.m_start.successors(self.map), key=lambda s_prime: self.m_start.cost(s_prime, self.map) + s_prime.g)
        return self.m_start
        # move to self.m_start.
        # get what paths changed (u and v nodes for each path)

    def iterate_scan(self, changed_node_pairs):
        if len(changed_node_pairs) != 0:
            self.k_m += self.heuristic(self.m_last, self.m_start)
            self.m_last = self.m_start
            for u, v, c_old in changed_node_pairs:
                new_cost = u.cost(v, self.map)
                if c_old > new_cost:
                    if u != self.m_goal:
                        u.rhs = min(u.rhs, new_cost + v.g)
                elif u.rhs == c_old + v.g:
                    if u != self.m_goal:
                        costs = [u.cost(s_prime, self.map) + s_prime.g for s_prime in u.successors]
                        u.rhs = min(costs)
                self.update_vertex(u)
        self.compute_shortest_path()