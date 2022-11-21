from cmath import inf
import copy
from utility import *


# For each move, you must scan the ground + the two nodes above the ground to ensure that you can moved there.
# scanning y = -1 plus the scans y = 0 and y = 1 for each cardinal direction yiels the ground scans for all y level ground elements.
# lookat(x, -1, z)
# lookat(x, 0, z)
# lookat(x, 1, z)
# lookat(x, 2, z)
# lookat(x, 3, z)
# we do not need to look at impossible elements.
# The following scans must be applied to all 4 directions, minimize the number of scanned blocks.
# If ground exists at y=-1, ground does not exist at y=0, nor does it exist at y=1 as the scan of y=-1 would not be possible, we need to check y=2 to ensure we can move.
# If ground exists at y=0, ground does not exist at y=1 as the scan of y=0 would not be possible, we need to check y=2 to ensure we can move.
# If ground exists at y=1, we are unsure about the 2 blocks above and thus need to check y=2 and y=3.

# If we decide to scan all of these, it results in 12+2 scans or 14 scans. (12 ground elements + 2 blocks above y=1)
# If we creatively pick where on the block to look (st it is visible and the ray can intersect other points of interest),
#   a block at y=-1 would only need 2 scans, a block at y=0 would need 2 scans, a block at y=1 would need 3 scans.
#   This resovles to at most 12 scans (3 scans per cardinal direction) and at least 8 scans. 
#   This gives better performance however we may have issues with precision errors or figuring out where to look on a block.


# We will add two new types to the map in order to deal with the 3d case.
# air and uncertain.
# An air block asserts that we know that no block exists there.
# An uncertain block is a dummy block that can be overridden by map scans.
# that adds extra complexity, what is a better way to do this?


possible_actions = [
    Vector3(1, -1, 0),
    Vector3(0, -1, 1),
    Vector3(-1, -1, 0),
    Vector3(0, -1, -1),

    Vector3(1, 0, 0),
    Vector3(0, 0, 1),
    Vector3(-1, 0, 0),
    Vector3(0, 0, -1),

    Vector3(1, 1, 0),
    Vector3(0, 1, 1),
    Vector3(-1, 1, 0),
    Vector3(0, 1, -1),
]

def satisfies_move_preconditions(y_level, ground, air1, air2, air3, air4):
    """
    Logic for determining if a move on any block is allowed.
    Gives the freedom for impossible moves if we have uncertain knowledge
    about the map for this move. (There is no uncertain knowledge here if the block was scanned)
    """
    air4_predicate = True
    if air4 is not None:
        air4_predicate = air4.block_type == "air" or air4.block_type == "uncertain"
    air4_predicate = (y_level != 1 or air4_predicate)

    air3_predicate = True
    if air3 is not None:
        air3_predicate = air3.block_type == "air" or air3.block_type == "uncertain"
    air3_predicate = (y_level == 0 or air3_predicate)

    air2_predicate = True
    if air2 is not None:
        air2_predicate = air2.block_type == "air" or air2.block_type == "uncertain"

    air1_predicate = True
    if air1 is not None:
        air1_predicate = air1.block_type == "air" or air1.block_type == "uncertain"

    ground_predicate = True
    if ground is not None:
        ground_predicate = ground.block_type != "air" or ground.block_type == "uncertain"

    return ground_predicate and air1_predicate and air2_predicate and air3_predicate

class MapNode:
    count = 0
    def __init__(self, block_type: str, position: Vector3, scans: list) -> None:
        """
        Create a new map node consiting of the information passed in.
        @param type - The type of the block the map node is.
        @param position - The location of the block.
        @param scans - A list of rays (yaw, pitch) recorded from 1.6 units above the current position (eye height)  
        """
        self.block_type = block_type
        self.position = position
        self.scans = copy.deepcopy(scans)
        self.rhs = float('inf')
        self.g = float('inf')
        self.backpointer = None
        if block_type == "uncertain":
            MapNode.count += 1
            #print(f"Generated Succ {self}")

    def copy(self):
        t = MapNode(self.block_type, self.position.copy(), [x.copy() for x in self.scans])
        t.rhs = self.rhs
        t.g = self.g
        t.backpointer = self.backpointer
        return t

    def actions(self):
        successors = []
        for action in possible_actions:
            successors.append(action.add(self.position))
        return successors

    def successors(self, map, lazy=False):
        """
        Return which of the maximum 4 minimum 0 branches out of the 12 are allowed.
        We can determine info by querying the map after a scan.
        Since we can ask this query to states that have not been scanned,
        we need to determine the difference between air and unknown spaces (given by scans).
        We also need to define predicates that tell us what states are applicable.
        
        property: exists((dx, dy, dz)) and air(dx, dy+1, dz) and air(dx, dy+2, dz) <and one_is_zero(dx, dz) and (dx, dy, dz) in (-1, 0, 1)> <these will be predetermined by possible actions list>
        MapGet((dx, dy, dz))
        @param map - the QueryMap containing all known/uncertain MapNodes.
        @param lazy - If this is True, non-existing MapNodes will not be added to the map to persist.
        """
        # All actions here are possible, however the cost is determined by the canMovePreconditions.
        # If an uncertain map element does not exist, we need to create it.
        # Uncertain map elements can be updated if a scan hits it. (updating its type and/or scans)
        successors = []
        for action in possible_actions:
            ground = action.add(self.position)
            ground_element = map.get(ground)
            if ground_element is None:
                ground_element = MapNode("uncertain", ground, [])
                if not lazy:
                    map.add(ground_element)

            successors.append(ground_element)
        return successors
    
    def cost(self, other, map) -> float:
        y_level = other.position.subtract(self.position).y # -1 if position is greater, 0 if equal, 1 if position is less.
        air1 = other.position.add(Vector3(0,1,0))
        air2 = other.position.add(Vector3(0,2,0))
        air3 = other.position.add(Vector3(0,3,0))
        air4 = self.position.add(Vector3(0, 3, 0))
        ground_element = map.get(other.position)
        air1_element = map.get(air1)
        air2_element = map.get(air2)
        air3_element = map.get(air3)
        air4_element = map.get(air4)

        if not satisfies_move_preconditions(y_level, ground_element, air1_element, air2_element, air3_element, air4_element):
            # cost to go to node is inf since it is not possible.
            return float('inf')
        else:
            # cost is the distance 1 or sqrt(2) -> gonna try just 1 for now.
            return self.position.distance(other.position)

    def __str__(self) -> str:
        return f"pos: {self.position}, type: {self.block_type}, rhs: {self.rhs}, g: {self.g}"

    def __eq__(self, other):
        """Overrides the default implementation"""
        if isinstance(other, MapNode):
            return self.position == other.position\
                and self.block_type == other.block_type
        return False
    
    def __ne__(self, other):
        """Overrides the default implementation (unnecessary in Python 3)"""
        return not self.__eq__(other)

    def __hash__(self):
        return hash((self.position.__hash__(), self.block_type))



class QueryMap:
    def __init__(self, other_map: Map, share_lock=None) -> None:
        """
        A map that can answer specific queries required for pathfinding.
        Map stored as a HashMap consisting of MapNodes.
        """
        self.map = {}
        self.other_map = other_map
        self.share_lock = share_lock
        self.recording = False
        self.changed_nodes = {}

    def clear_pathfinding_values(self):
        for k in self.map:
            v = self.map[k]
            if v is not None:
                v.g = float('inf')
                v.rhs = float('inf')
                v.backpointer = None

    def record_edge_cost_changes(self):
        """
        Save every node that changed,
        When we need to calculate prior, revert graph using recorded nodes.
        Then Unrevert to get the posterior.
        """
        self.recording = True
        self.changed_nodes = {}

    def retrieve_records(self):
        self.record = False
        return self.changed_nodes

    def overwrite(self, changes):
        """
        Restore the graph its previous state given by the changes.
        """
         # restore to record and backup current.
        backup = {}
        for key in changes:
            backup[key] = self.map.get(key, None)
            if backup[key] is not None:
                self.map.pop(key, None)
            if changes[key] is not None:
                self.map[key] = changes[key]
        return backup

    def calculate_node_successor_costs(self, positions: list):
        """
        Given a list of Vector3 positions. Query the map for the element at that point.
        We can assert that the map does have a node there. For all possible actions at
        that node, if there is an element at that action, record the edge costs.

        returns a dict mapping tuple of positions (u,v) to a float cost.
        """
        results = {}
        for key in positions:
            node = self.map[key]
            actions = node.actions()
            for action in actions:
                s_prime = self.map.get(action, None)
                if s_prime is None:
                    s_prime = MapNode("uncertain", action, [])
                cost_a = node.cost(s_prime, self)
                cost_b = s_prime.cost(node, self)
                results[(node.position, s_prime.position)] = cost_a
                results[(s_prime.position, node.position)] = cost_b

        return results

    def calculate_node_cost_changes(self) -> list:
        """
        Uses the recording of added nodes or updated nodes to revert
        the graph to its prior state and calculate the node costs
        before any of the changes. It then reverts back to the
        current graph to check for if any node costs changed.

        I am likely doing something wrong here.
        """
        self.recording = False
        backup = self.overwrite(self.changed_nodes) # graph is reverted to previous state.
        costs = self.calculate_node_successor_costs(self.changed_nodes.keys())
        self.overwrite(backup) # graph is restored

        changed_node_pairs = []
        for key in self.changed_nodes.keys():
            node = self.map.get(key, None)
            if node is not None:
                actions = node.actions()
                for action in actions:
                    s_prime = self.map.get(action, None)
                    if s_prime is None:
                        s_prime = MapNode("uncertain", action, [])

                    cost_a = node.cost(s_prime, self)
                    cost_b = s_prime.cost(node, self)
                    old_cost_a = costs[(node.position, s_prime.position)]
                    old_cost_b = costs[(s_prime.position, node.position)]

                    if old_cost_a != cost_a:
                        changed_node_pairs.append((node, s_prime, old_cost_a))
                    if old_cost_b != cost_b:
                        changed_node_pairs.append((s_prime, node, old_cost_b))

        return changed_node_pairs

        


    def add(self, element: MapNode) -> None:
        """
        Add the given MapNode to the Map, we should be able to retrieve this through the position property.
        @param element - A MapNode containing information about the map at that point.
        """
        if element.position in self.map:
            raise f"Element {element} already exists in the map."

        if element is None:
            return
        
        # Record the costs prior if we are recording.
        if self.recording and self.changed_nodes.get(element.position, None) is None:
            self.changed_nodes[element.position.copy()] = element.copy()

        self.map[element.position] = element
        if element.block_type == "uncertain" or element.block_type == "air":
            self.other_map.add_block(element.block_type, element.position)

    def update(self, element: MapNode, only_if_uncertain=True) -> None:
        """
        Update the element at the given position.
        Used to update uncertain or unknown elements once we gain information.
        """
        if element is None:
            return

        old_element = self.map.get(element.position, None)
        if old_element is None:
            if self.recording and self.changed_nodes.get(element.position, None) is None:
                self.changed_nodes[element.position.copy()] = None
            self.map[element.position] = element
        elif (not only_if_uncertain) or old_element.block_type == "uncertain" or old_element.block_type == "air":
            if self.recording:
                self.changed_nodes[element.position.copy()] = self.map[element.position].copy()
            self.map.pop(element.position, None)
            self.map[element.position] = element

    def update_type(self, position: Vector3, new_type: str, only_if_uncertain=True):
        """
        If block at the given position exists, change its type.
        """
        old_element = self.map.get(position, None)
        if old_element is None:
            return
        if (not only_if_uncertain) or old_element.block_type == "uncertain" or old_element.block_type == "air":
            if self.recording and old_element.block_type != new_type:
                self.changed_nodes[position.copy()] = old_element.copy()
            self.map[position].block_type = new_type

    def add_scan(self, position: Vector3, scan: BlockRotation):
        """
        Add a scan to the MapNode at the given position.
        """
        if position in self.map:
            self.map[position].scans.append(scan)

    def get(self, position: Vector3) -> MapNode:
        """
        returns the MapNode at the given position or None if it does not exist.
        """
        if position not in self.map:
            return None
        return self.map[position]

    def delete(self, position: Vector3):
        """
        Removes the element in the given position from the map.
        """
        self.map.pop(position, None)