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
            #raise "Error, cannot pathfind without knowing where the ground is (Scan downward before starting)"
        self.m_last = self.m_start
        self.goal = goal
        self.m_goal = map.get(goal)
        if self.m_goal is None:
            self.m_goal = MapNode("uncertain", goal, [])
            map.add(self.m_goal)
        self.m_goal.rhs = 0
        self.U.insert(self.m_goal, Priority(self.heuristic(self.m_start, self.m_goal), 0))
        self.compute_shortest_path()

    def contains(self, u: MapNode) -> bool:
        return u in self.U.vertices_in_heap

    def heuristic(self, s: MapNode, s_prime: MapNode):
        #return s.position.distance(s_prime.position)
        return s.position.manhattan_distance(s_prime.position)
    
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
            u = self.U.top()
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
                predecessors_union_u = u.successors(self.map)
                predecessors_union_u.append(u) # u cannot be its own successor therefore no check for that is needed.
                for s in predecessors_union_u:
                    if s.rhs == (s.cost(u, self.map) + g_old):
                        if s != self.m_goal:
                            costs = [s.cost(s_prime, self.map) + s_prime.g for s_prime in s.successors(self.map, lazy=True)]
                            s.rhs = min(costs)
                    self.update_vertex(s)
        #print(f"compute_shortest_path expanded {MapNode.count} so far")
        #self.print_shortest_path()

    def iterate_move(self):
        """
        Decomposed the main loop of d*-lite into one that I can manually iterate.
        """
        if self.m_start == self.m_goal:
            return self.m_goal
        if self.m_start.rhs == float('inf'):
            raise "NoPathExists"
        
        #print(f"Successors: {[(self.m_start.cost(s_prime, self.map), s_prime.g, s_prime.position) for s_prime in self.m_start.successors(self.map)]}")
        prev = self.m_start
        costs = [(s_prime, self.m_start.cost(s_prime, self.map) + s_prime.g) for s_prime in self.m_start.successors(self.map)]
        self.m_start, min_cost = min(costs, key=lambda x: x[1])
        # Settle ties:
        ties = [x for x in costs if x[1] == min_cost]
        # break tie as min cost and heuristic.
        self.m_start, min_cost = min(ties, key=lambda x: prev.cost(x[0], self.map) + self.heuristic(prev, x[0]))

        return self.m_start
        # move to self.m_start.
        # get what paths changed (u and v nodes for each path)

    def print_shortest_path(self):
        for e in self.current_path:
            self.map.other_map.remove_dummy_block(e.position)

        self.current_path = []
        current = self.m_start
        while current != self.m_goal:
            if current.rhs == float('inf'):
                raise "NoPathExists"

            prev = current
            costs = [(s_prime, current.cost(s_prime, self.map) + s_prime.g) for s_prime in current.successors(self.map)]
            current, min_cost = min(costs, key=lambda x: x[1])
            # Settle ties:
            ties = [x for x in costs if x[1] == min_cost]
            # break tie as min cost and heuristic.
            current, min_cost = min(ties, key=lambda x: prev.cost(x[0], self.map) + self.heuristic(prev, x[0]))
            temp_node = current.copy()
            self.map.other_map.add_dummy_block("path_node", temp_node.position)
            self.current_path.append(temp_node)

    def iterate_scan(self, changed_node_pairs):
        if len(changed_node_pairs) != 0:
            #print("The following changed:")
            self.k_m += self.heuristic(self.m_last, self.m_start)
            self.m_last = self.m_start
            for u, v, c_old in changed_node_pairs:
                new_cost = u.cost(v, self.map)
                #print(f"{u} -> {v} went from {c_old} -> {new_cost}")
                if c_old > new_cost:
                    if u != self.m_goal:
                        u.rhs = min(u.rhs, new_cost + v.g)
                elif u.rhs == (c_old + v.g):
                    if u != self.m_goal:
                        costs = [u.cost(s_prime, self.map) + s_prime.g for s_prime in u.successors(self.map)]
                        u.rhs = min(costs)
                self.update_vertex(u)
        self.compute_shortest_path()