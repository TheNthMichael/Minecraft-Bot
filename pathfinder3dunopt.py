from priority_queue import PriorityQueue, Priority
from map import QueryMap, MapNode
from utils import *
from utility import Vector3


class Pathfinder3DUnoptimized:
    def __init__(self, start: Vector3, goal: Vector3, map: QueryMap) -> None:
        """
        Initialize the unoptimized version of the d*-lite algorithm.
        """
        self.U = PriorityQueue()
        self.k_m = 0
        self.map = map
        self.start = start
        self.m_start = map.get(start)
        if self.m_start is None:
            self.m_start = MapNode("ground", start, [])
            print(f"Starting pathfinder from node {self.m_start.position}-{self.m_start.block_type}")
            self.map.add(self.m_start)
            #raise "Error, cannot pathfind without knowing where the ground is (Scan downward before starting)"
        self.m_last = self.m_start
        self.goal = goal
        self.m_goal = map.get(goal)
        if self.m_goal is None:
            self.m_goal = MapNode("uncertain", goal, [])
            map.add(self.m_goal)
        self.m_goal.rhs = 0
        self.U.insert(self.m_goal, self.calculate_key(self.m_goal))
        self.compute_shortest_path()

    def calculate_key(self, u: MapNode):
        return Priority(
            min(u.g, u.rhs) + self.heuristic(self.m_start, u) + self.k_m,
            min(u.g, u.rhs)
        )

    def contains(self, u: MapNode) -> bool:
        return u in self.U.vertices_in_heap

    def heuristic(self, s: MapNode, s_prime: MapNode):
        return s.position.distance(s_prime.position)

    def update_vertex(self, u: MapNode):
        if u != self.m_goal:
            costs = [u.cost(s_prime, self.map) + s_prime.g\
                for s_prime in u.successors(self.map)]
            u.rhs = min(costs)
        if self.contains(u):
            self.U.remove(u)
        if u.g != u.rhs:
            self.U.insert(u, self.calculate_key(u))

    def compute_shortest_path(self):
        while self.U.top_key() < self.calculate_key(self.m_start)\
            or self.m_start.rhs != self.m_start.g:
            u = self.U.top()
            self.U.remove(u)
            print(u)
            k_old = self.U.top_key()
            new_key = self.calculate_key(u)
            if k_old < new_key:
                self.U.insert(u, new_key)
            elif u.g > u.rhs:
                u.g = u.rhs
                for s in u.successors(self.map):
                    self.update_vertex(s)
            else:
                u.g = float('inf')
                for s in u.successors(self.map):
                    self.update_vertex(s)
                self.update_vertex(u) # u cannot be a successor to itself.

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
            print(self.m_last.position)
            print(self.m_start.position)
            self.k_m += self.heuristic(self.m_last, self.m_start)
            self.m_last = self.m_start
            for u, v, c_old in changed_node_pairs:
                self.update_vertex(u)
            self.compute_shortest_path()