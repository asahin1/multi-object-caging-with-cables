# Third-party
import networkx as nx

# Standard
from collections import deque
import logging

# M-MOCP
from m_mocp.object_caging_graph import ObjectCagingGraph

logger = logging.getLogger(__name__)

def extract_active_subgraph(G: nx.DiGraph, threshold: float = 1e-3) -> nx.DiGraph:
    """
    Extracts the subgraph from G for a given edge weight lower bound.

    Args:
      G (networkx.DiGraph): A directional graph
      threshold (float): Lower bound for edge weight

    Returns:
      nx.Digraph: The subgraph induced from the edges of G with weight greater than threshold
    """

    def filter_edge(u, v):
        return G[u][v].get("weight", 0) > threshold

    def filter_node(n):
        return any(
            G[u][v].get("weight", 0) > threshold
            for u, v in (
                [(n, nbr) for nbr in G.successors(n)]
                + [(pred, n) for pred in G.predecessors(n)]
            )
        )

    return nx.subgraph_view(G, filter_edge=filter_edge, filter_node=filter_node)


def get_cable_subgraphs(
    n_cables: int,
    oc_graph: ObjectCagingGraph,
    milp_variable_vals: dict[str, float],
) -> list[nx.DiGraph]:
    """
    Constructs a directional subgraph for each cable based on object caging graph and an MILP solution.

    Args:
      n_cables (int): number of cables
      oc_graph (ObjectCagingGraph): object caging graph
      milp_variable_vals (dict[str,float]): a dictionary mapping variable names to their values

    Returns:
      list[nx.DiGraph]: A directed graph for each cable

    Raises:
      KeyError: If an edge in the object graph does not correspond to a variable in the MILP solution.
    """
    H = []
    for k in range(n_cables):
        h = nx.DiGraph()
        for n1, n2 in oc_graph.edges:
            edge1 = f"x{k + 1}_{n1},{n2}"
            edge2 = f"x{k + 1}_{n2},{n1}"
            if edge1 not in milp_variable_vals:
                raise KeyError(
                    f"Edge {edge1} in object caging graph is not part of MILP solution."
                )
            if edge2 not in milp_variable_vals:
                raise KeyError(
                    f"Edge {edge2} in object caging graph is not part of MILP solution."
                )
            w1 = milp_variable_vals[edge1]
            w2 = milp_variable_vals[edge2]
            h.add_edge(n1, n2, weight=w1)
            h.add_edge(n2, n1, weight=w2)
        logger.debug("Cable subgraph weighted: %s", h.edges(data=True))
        H.append(extract_active_subgraph(h))
    return H


def identify_isolated_subroutes(
    H: list[nx.DiGraph], start_positions: list[int]
) -> list[list[int]]:
    """
    Given a list of cable subgraphs and their start positions identify the isolated subroutes.

    Args:
      H (list[nx.DiGraph]): list of cable subgraphs
      start_positions (list[int]): start positions of each cable

    Returns:
      list[list[int]]: a list of cycles (may be empty if no isolated subroutes)
    """
    identified_cycles = []
    for k, h in enumerate(H):
        connected_components = list(nx.weakly_connected_components(h))
        logger.debug("Connected components: %s", connected_components)
        if len(connected_components) == 1:
            continue
        for comp in connected_components:
            if start_positions is None or start_positions[k] not in comp:
                part_graph = h.subgraph(comp).copy()
                start = nx.utils.arbitrary_element(part_graph.nodes)
                cycle = [u for u, _ in nx.eulerian_path(part_graph, start)]
                identified_cycles.append(cycle)
    return identified_cycles


def compute_uncoord_trail(cable_subgraph, start_vertex, object_caging_graph):
    path = [(None, start_vertex)]
    visited = []
    while path:
        if len(path) == len(cable_subgraph.edges) + 1:
            trail = [p[1] for p in path]
            return trail
        current = path[-1][1]
        successors = []
        for s in cable_subgraph.successors(current):
            if (current, s) in path:
                continue
            if path + [(current, s)] in visited:
                continue
            successors.append(s)
        assert len(successors) < 3
        if len(successors) < 1:
            visited.append(path.copy())
            path.pop()
        elif len(successors) < 2:
            path.append((current, successors[0]))
        else:
            assert len(successors) == 2
            assert len(path) > 1
            # Favor in cycle neighbor if exists
            current_obj_id = object_caging_graph.edge_object(path[-1][0], path[-1][1])
            favored_successor = None
            for s in successors:
                if object_caging_graph.edge_object(current, s) == current_obj_id:
                    favored_successor = s
            if favored_successor is None:
                favored_successor = successors[0]
            path.append((current, favored_successor))
    return None


def compute_trail(
    cable_subgraph: nx.DiGraph, start_vertex, object_caging_graph: ObjectCagingGraph
):
    current = start_vertex
    trail = [current]
    while True:
        successors = list(cable_subgraph.successors(current))
        if len(successors) < 1:
            break
        elif len(successors) < 2:
            current = successors[0]
        else:
            assert (
                len(successors) == 2
            )  # On object caging graphs we cannot have more than two outgoing edges
            assert (
                len(trail) > 1
            )  # Otherwise we cannot check the last edge, if length of the trail is 1, it cannot be on a degree-4 vertex anyway
            # Pick the successor that keeps cable within object cycle
            current_obj_id = object_caging_graph.edge_object(trail[-2], trail[-1])
            current = None
            for s in successors:
                if object_caging_graph.edge_object(trail[-1], s) == current_obj_id:
                    current = s
                    break
            if current is None:
                return None
        trail.append(current)
    if len(trail) != len(cable_subgraph.edges) + 1:
        return None
    return trail


def compute_interlace_labels(
    oc_graph: ObjectCagingGraph, trails: list[list[int]]
) -> dict[int, bool]:
    """
    Computes the interlace labels for a given cable trail collection.

    Args:
      oc_graph (ObjectCagingGraph): object caging graph
      trails (list[list[int]]): List of trails, where each graph represents a cable configuration.

    Returns:
      dict[int,bool]: A map from vertex ids to booleans indicating whether cables interlace.

    Raises:
      RuntimeError: If there exists an edge of the object caging graph that is not occupied by any of the trails
    """
    B = {}
    for object_cycle in oc_graph.object_cycles:
        vertices = object_cycle[:-1]
        n_vert = len(vertices)
        for i in range(n_vert):
            # Check every consecutive triple vertices in object cycle
            v1 = object_cycle[i % n_vert]
            v2 = object_cycle[(i + 1) % n_vert]
            v3 = object_cycle[(i + 2) % n_vert]
            if v2 in B:
                continue
            e1 = frozenset((v1, v2))
            e2 = frozenset((v2, v3))
            e1_cable = None  # Should denote the cable and step along the trail that covers v1-v2
            e2_cable = None  # Should denote the cable and step along the trail that covers v2-v3
            for k, trail in enumerate(trails):
                for j in range(len(trail) - 1):
                    p1 = trail[j]
                    p2 = trail[j + 1]
                    edge = frozenset((p1, p2))
                    if edge == e1:
                        e1_cable = (k, j)
                    elif edge == e2:
                        e2_cable = (k, j)
                    else:
                        continue
            # If two consecutive edges are not covered by the same cable
            # there must be an interlace
            if e1_cable is None or e2_cable is None:
                print(trails)
                print(f"e1: {e1}, cable: {e1_cable}")
                print(f"e2: {e2}, cable: {e2_cable}")
                raise RuntimeError(
                    "COMPUTE INTERLACE LABELS: Each edge must have been occupied by a cable segment!"
                )
            if e1_cable[0] != e2_cable[0]:
                B[v2] = True
            elif abs(e1_cable[1] - e2_cable[1]) > 1:
                B[v2] = True
            else:
                B[v2] = False
    return B


def deadlock_exists(trails: list[list[int]], B: dict[int, bool]):
    """
    Checks whether the given trail and interlace label combination leads to a deadlock.

    Args:
      trails (list[list[int]]): cable trails
      B (dict[int,bool]): interlace labels

    Returns:
      bool: True if deadlock, False if not
    """
    n_cables = len(trails)
    init_map: dict[int, None | int] = {v: None for v in B}
    interlace_trails = [[v for v in trail if B[v] == 1] for trail in trails]
    trail_deques = [deque(trail) for trail in interlace_trails]
    for i in range(n_cables):
        td = trail_deques[i]
        if init_map[td[0]] is None:
            init_map[td[0]] = i
        else:
            init_map[td[0]] = None
        td.popleft()
    logger.debug("init_map: %s", init_map)
    logger.debug("trail_deques: %s", trail_deques)

    vertex_cable_end_map: dict[int, None | int] = {v: None for v in B}
    deadlock = True
    while any([len(td) for td in trail_deques]):
        deadlock = True
        for i in range(n_cables):
            logger.debug(trail_deques)
            td = trail_deques[i]
            if len(td) == 0:
                continue
            v = td[0]
            other_cable = vertex_cable_end_map[v]
            if other_cable is None:
                vertex_cable_end_map[v] = i
            elif other_cable != i:
                td.popleft()
                trail_deques[other_cable].popleft()
                vertex_cable_end_map[v] = None
                deadlock = False
            if init_map[v] is not None and init_map[v] != i:
                td.popleft()
                init_map[v] = None
                deadlock = False
        if deadlock:
            return True
    return False
