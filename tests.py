from map import QueryMap, MapNode
from utility import Map, Vector3
import copy

def test_query_map_overwrite():
    # define a 3x3 ground with a 3 block tall wall on the right.
    new_blocks = [
        Vector3(0, 0, 0),
        Vector3(1, 0, 0),
        Vector3(0, 0, 1),

        Vector3(-1, 0, 0),
        Vector3(0, 0, -1),

        Vector3(1, 0, 1),
        Vector3(-1, 0, -1),
        Vector3(-1, 0, 1),
        Vector3(1, 0, -1),

        # Wall
        Vector3(1, 1, 1),
        Vector3(0, 1, 1),
        Vector3(-1, 1, 1),

        Vector3(1, 2, 1),
        Vector3(0, 2, 1),
        Vector3(-1, 2, 1),

        Vector3(1, 3, 1),
        Vector3(0, 3, 1),
        Vector3(-1, 3, 1),


    ]
    qmap = QueryMap(None, None)
    qmap.record_edge_cost_changes()

    for block in new_blocks:
        qmap.add(MapNode("Ground", block, []))

    original = copy.deepcopy(qmap.map)
    backup = qmap.overwrite(qmap.changed_nodes) # graph is reverted to previous state.
    costs = qmap.calculate_node_successor_costs(qmap.changed_nodes.keys())
    qmap.overwrite(backup) # graph is restored

    assert qmap.map == original


def test_query_map_record_function():
    # define a 3x3 ground with a 3 block tall wall on the right.
    new_blocks = [
        Vector3(0, 0, 0),
        Vector3(1, 0, 0),
        Vector3(0, 0, 1),

        Vector3(-1, 0, 0),
        Vector3(0, 0, -1),

        Vector3(1, 0, 1),
        Vector3(-1, 0, -1),
        Vector3(-1, 0, 1),
        Vector3(1, 0, -1),

        # Wall
        Vector3(1, 1, 1),
        Vector3(0, 1, 1),
        Vector3(-1, 1, 1),

        Vector3(1, 2, 1),
        Vector3(0, 2, 1),
        Vector3(-1, 2, 1),

        Vector3(1, 3, 1),
        Vector3(0, 3, 1),
        Vector3(-1, 3, 1),


    ]
    qmap = QueryMap(None, None)
    qmap.record_edge_cost_changes()

    for block in new_blocks:
        qmap.add(MapNode("uncertain", block, []))

    changes = qmap.calculate_node_cost_changes()
    a = {}
    print(len(changes))
    for u, v, old_c in changes:
        #print(f"{u} -> {v}, {old_c} -> {u.cost(v, qmap)}")
        if a.get((u.position, v.position), None) is None:
            a[(u.position, v.position)] = []
        a[(u.position, v.position)].append(f"{old_c} -> {u.cost(v, qmap)}")

    print(len(a))
    for k in a:
        if len(a[k]) > 1:
            print(f"{k}, {a[k]}")


test_query_map_record_function()