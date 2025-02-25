"""
Out of laziness we are using the following code: https://github.com/mdeyo/d-star-lite
Since the code uses a fairly messy api for grid set up and names that may not be applicable
with the 3 vector system we currently have, we will define an api for handling this.
"""
from graph import Node, Graph
from grid import GridWorld
from utils import *
from d_star_lite import initDStarLite, moveAndRescan
from utility import Vector3


class Pathfinder:
    _MAX_GRID_SIZE = 100000
    _PADDING = 50
    
    # The amount of shells around the current position that we have knowledge of.
    _VIEWING_RANGE = 2

    def __init__(self, start: Vector3, goal: Vector3) -> None:
        # We expect start and goal to be castable to int.
        self.start = start
        self.goal = goal

        # We should really avoid massive grids as they will use a large amount of memory.
        # Search areas should be less than 100x100 baseline (without buffer spaces) as searching larger areas will likely cause other problems.
        diff = goal.subtract(start)
        self.width = int(abs(diff.x)) + Pathfinder._PADDING * 2
        self.height = int(abs(diff.z)) + Pathfinder._PADDING * 2
        self.s_start = coordsToStateName(Vector3(Pathfinder._PADDING, 0, Pathfinder._PADDING))
        self.s_goal = coordsToStateName(Vector3(self.width - Pathfinder._PADDING, 0, self.height - Pathfinder._PADDING))
        assert self.width * self.height < Pathfinder._MAX_GRID_SIZE
        self.graph = GridWorld(self.width, self.height)
        self.graph.setStart(self.s_start)
        self.graph.setGoal(self.s_goal)
        self.k_m = 0
        self.s_last = self.s_start
        self.queue = []
        self.graph, self.queue, self.k_m = initDStarLite(self.graph, self.queue, self.s_start, self.s_goal, self.k_m)
        self.s_current = self.s_start
        self.pos_coords = stateNameToCoords(self.s_current)

    
    def iterate(self, blocked_neighbors: list):
        # Update list
        for n in blocked_neighbors:
            grid_loc = world2Grid(n, self.start, Pathfinder._PADDING)
            row = int(grid_loc.z)
            col = int(grid_loc.x)
            if self.graph.cells[row][col] == 0:
                self.graph.cells[row][col] = -1
        
        s_new, self.k_m = moveAndRescan(self.graph, self.queue, self.s_current, Pathfinder._VIEWING_RANGE, self.k_m)
        if s_new == "goal":
            return None
        self.s_current = s_new
        grid_coords = stateNameToCoords(self.s_current)
        world_coords = grid2World(grid_coords[0], grid_coords[1], self.start, Pathfinder._PADDING)
        print(f"next node - {self.s_current}; real - {world_coords}")
        return world_coords