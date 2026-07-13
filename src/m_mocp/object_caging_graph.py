import networkx as nx

from m_mocp.point_2d import Point2D


class ObjectCagingGraph:

    def __init__(self, nx_graph=None, object_cycles=[], edge_object_map={}) -> None:
        self._G = nx_graph if nx_graph else nx.Graph()
        self.object_cycles = object_cycles  # List - [[cycle 1 nodes], [], ...]
        self.edge_object_map = edge_object_map

    @property
    def nodes(self):
        """Returns the NetworkX NodeView."""
        return self._G.nodes

    @property
    def edges(self):
        """Returns the NetworkX EdgeView."""
        return self._G.edges

    def edge_object(self, n1, n2):
        return self.edge_object_map[frozenset((n1, n2))]

    def degree(self, node_id):
        if not self._G.has_node(node_id):
            raise ValueError(f"Node {node_id} is not in graph.")
        return self._G.degree(node_id)

    def neighbors(self, node_id):
        if not self._G.has_node(node_id):
            raise ValueError(f"Node {node_id} is not in graph.")
        return self._G.neighbors(node_id)

    def coords(self, node_id):
        if not self._G.has_node(node_id):
            raise ValueError(f"Node {node_id} is not in graph.")
        return self._G.nodes[node_id]["coords"]

    def cost(self, n1, n2):
        if not self._G.has_node(n1):
            raise ValueError(f"Node {n1} is not in graph.")
        if not self._G.has_node(n2):
            raise ValueError(f"Node {n2} is not in graph.")
        if not self._G.has_edge(n1, n2):
            raise ValueError(f"Edge ({n1},{n2}) is not in graph.")
        return self._G[n1][n2]["cost"]

    @classmethod
    def from_dict(cls, data):
        """
        Pure logic. Turns a dictionary into an object.
        NO file I/O here.
        """
        # Validate data
        if "nodes" not in data or "edges" not in data or "loops" not in data:
            raise ValueError("Invalid dictionary format")

        new_nx_graph = nx.Graph()

        # Parse logic
        neighbors = {}
        object_cycles = []
        for node_id, info in data["nodes"].items():
            node_id = int(node_id)
            new_nx_graph.add_node(node_id, coords=Point2D(info["x"], info["y"]))

            neighbors[node_id] = {
                int(nbh_id): edge_id for nbh_id, edge_id in info["neighbors"].items()
            }
            for nbh_id, edge_id in neighbors[node_id].items():
                edge_data = data["edges"][str(edge_id)]
                points = [Point2D(x, y) for x, y in zip(edge_data["x"], edge_data["y"])]
                if "cost" in edge_data:
                    cost = edge_data["cost"]
                else:
                    total = 0
                    for i in range(len(points) - 1):
                        total += points[i].getDistance(points[i + 1])
                    cost = total
                new_nx_graph.add_edge(node_id, nbh_id, id=int(edge_id), cost=cost)

        for _, info in data["loops"].items():
            object_cycles.append(info)

        edge_object_map = {}
        for k, cycle in enumerate(object_cycles):
            for i in range(len(cycle) - 1):
                v1 = cycle[i]
                v2 = cycle[i + 1]
                edge_object_map[frozenset((v1, v2))] = k

        return cls(new_nx_graph, object_cycles, edge_object_map)

    @classmethod
    def from_file(cls, filepath, name):
        """
        I/O only. Delegates logic to from_dict.
        """
        import json

        with open(filepath, "r") as f:
            all_graph_data = json.load(f)
            if name not in all_graph_data:
                raise KeyError(
                    f"Graph '{name}' not found. Available: {list(all_graph_data)}"
                )
            raw_graph_data = all_graph_data[name]

        return cls.from_dict(raw_graph_data)
