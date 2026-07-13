# M-MOCP
from m_mocp.algorithms import (
    compute_trail,
    get_cable_subgraphs,
    identify_isolated_subroutes,
    compute_interlace_labels,
    deadlock_exists,
)
from m_mocp.object_caging_graph import ObjectCagingGraph

# Standard
import logging

logger = logging.getLogger(__name__)


def build_minimal_mocp_milp(solver, oc_graph: ObjectCagingGraph, start_positions):
    n_cables = len(start_positions)

    # 1. Define Variables
    # ======================================================================
    variables = {}  # Dictionary that maps string names to solver variable handles

    # x{t+1}_{n1},{n2}: Incidence vectors, tells whether subgraph of cable t+1 contains edge (n1,n2)
    for t in range(n_cables):
        # Loop over each edge in the oc_graph
        for n1, n2 in oc_graph.edges:
            name1 = f"x{t + 1}_{n1},{n2}"
            name2 = f"x{t + 1}_{n2},{n1}"
            variables[name1] = solver.add_variable(vtype="B", name=name1)
            variables[name2] = solver.add_variable(vtype="B", name=name2)

    # z: upper bound on cable length (to be minimized)
    variables["length_upperbound"] = solver.add_variable(name="length_upperbound")

    # f{t+1}_{v}: Demand vector, indicates whether v is a start, end, or neither for cable t+1
    for t in range(n_cables):
        for v in oc_graph.nodes:
            name = f"f{t + 1}_{v}"
            if start_positions[t] == v:
                variables[name] = solver.add_variable(
                    lb=-1, ub=-1, name=name
                )  # start vertex
            elif oc_graph.degree(v) == 4:
                variables[name] = solver.add_variable(
                    lb=0, ub=0, name=name
                )  # degree-4 vertex
            else:
                variables[name] = solver.add_variable(vtype="B", name=name)

    # 2. Define Constraints
    # ======================================================================
    constraints = {}  # Dictionary that maps string names to solver constraint handles

    # Feasible set X
    # a) Edge coverage
    for n1, n2 in oc_graph.edges:
        lhs = 0
        for t in range(n_cables):
            edge1_name = f"x{t + 1}_{n1},{n2}"
            edge2_name = f"x{t + 1}_{n2},{n1}"
            lhs += variables[edge1_name] + variables[edge2_name]
        rhs = 1
        constraint_name = f"edge_coverage-{{{n1},{n2}}}"
        constraints[constraint_name] = solver.add_constraint(
            lhs, "==", rhs, constraint_name
        )

    # b) Multi-cable coverage of each object cycle
    for object_id, cycle in enumerate(oc_graph.object_cycles):
        for t in range(n_cables):
            lhs = 0
            for i in range(len(cycle) - 1):
                edge1_name = f"x{t + 1}_{cycle[i]},{cycle[i + 1]}"
                edge2_name = f"x{t + 1}_{cycle[i + 1]},{cycle[i]}"
                lhs += variables[edge1_name] + variables[edge2_name]
            rhs = len(cycle) - 2
            constraint_name = f"object_coverage-{object_id}_cable-{t + 1}"
            constraints[constraint_name] = solver.add_constraint(
                lhs, "<=", rhs, constraint_name
            )

    # Feasible set F
    # Supply-Demand along a cable
    for t in range(n_cables):
        lhs = 0
        for v in oc_graph.nodes:
            name = f"f{t + 1}_{v}"
            lhs += variables[name]
        rhs = 0
        constraint_name = f"Supply_demand-cable_{t + 1}"
        constraints[constraint_name] = solver.add_constraint(
            lhs, "==", rhs, constraint_name
        )

    # Coupling constraints (Flow constraints)
    for t in range(n_cables):
        for v in oc_graph.nodes:
            flow_in = 0
            flow_out = 0
            for nbh in oc_graph.neighbors(v):
                edge_in_name = f"x{t + 1}_{nbh},{v}"
                edge_out_name = f"x{t + 1}_{v},{nbh}"
                flow_in += variables[edge_in_name]
                flow_out += variables[edge_out_name]
            demand_name = f"f{t + 1}_{v}"
            lhs = flow_in - flow_out
            rhs = variables[demand_name]
            constraint_name = f"Flow_at_{v}-cable_{t + 1}"
            constraints[constraint_name] = solver.add_constraint(
                lhs, "==", rhs, constraint_name
            )

    # Length upper bound for cables
    for t in range(n_cables):
        lhs = 0
        for n1, n2 in oc_graph.edges:
            edge1_name = f"x{t + 1}_{n1},{n2}"
            edge2_name = f"x{t + 1}_{n2},{n1}"
            lhs += (variables[edge1_name] + variables[edge2_name]) * oc_graph.cost(
                n1, n2
            )
        rhs = variables["length_upperbound"]
        constraint_name = f"Length_upperbound-cable_{t + 1}"
        constraints[constraint_name] = solver.add_constraint(
            lhs, "<=", rhs, constraint_name
        )

    # Symmetry break by deterministically assigning an incident edge to a cable if two cables start from same vertex
    start_nbhs = {}
    for t in range(n_cables):
        start = start_positions[t]
        if start not in start_nbhs:
            start_nbhs[start] = False
        else:
            start_nbhs[start] = sorted(list(oc_graph.neighbors(start)), reverse=True)

    for t in range(n_cables):
        start = start_positions[t]
        if not start_nbhs[start]:
            # Only a single cable was starting from this vertex
            continue
        nbh = start_nbhs[start].pop()
        edge_name = f"x{t + 1}_{start},{nbh}"
        lhs = variables[edge_name]
        constraint_name = f"Symmetry_break_{start}_{t + 1}"
        constraints[constraint_name] = solver.add_constraint(
            lhs, "==", 1, constraint_name
        )

    # Define Objective
    # ======================================================================
    solver.set_objective(variables["length_upperbound"], minimize=True)

    return variables, constraints


def build_mocp_lazy_ise_only(
    solver, variables, constraints, n_cables, oc_graph, start_positions
):
    solver.n_lazy_constraints["ise"] = 0

    def lazy_isolated_subroute_elimination(context):
        variable_vals = context.get_value(variables)
        H = get_cable_subgraphs(n_cables, oc_graph, variable_vals)
        identified_cycles = identify_isolated_subroutes(H, start_positions)
        if len(identified_cycles) == 0:
            return
        else:
            solver.n_lazy_constraints["ise"] += len(identified_cycles)
            add_mocp_isolated_subroute_constraints(
                solver,
                variables,
                constraints,
                oc_graph,
                identified_cycles,
                n_cables,
                lazy=True,
            )

    return lazy_isolated_subroute_elimination


def build_mocp_lazy_ise_and_deadlock(
    solver, variables, constraints, n_cables, oc_graph, start_positions
):
    solver.n_lazy_constraints["ise"] = 0
    solver.n_lazy_constraints["deadlock"] = 0

    def lazy_constraints(context):
        variable_vals = context.get_value(variables)
        H = get_cable_subgraphs(n_cables, oc_graph, variable_vals)
        identified_cycles = identify_isolated_subroutes(H, start_positions)
        if len(identified_cycles) > 0:
            solver.n_lazy_constraints["ise"] += len(identified_cycles)
            add_mocp_isolated_subroute_constraints(
                solver,
                variables,
                constraints,
                oc_graph,
                identified_cycles,
                n_cables,
                lazy=True,
            )
            return
        else:
            trails = []
            for h, start in zip(H, start_positions):
                trail = compute_trail(h, start, oc_graph)
                if trail is None:
                    # Self-induced deadlock
                    solver.n_lazy_constraints["deadlock"] += 1
                    add_mocp_deadlock_elimination_constraints(
                        solver,
                        variables,
                        constraints,
                        oc_graph,
                        variable_vals,
                        n_cables,
                    )
                    return
                trails.append(trail)
            B = compute_interlace_labels(oc_graph, trails)
            if not deadlock_exists(trails, B):
                return
            else:
                # Coordination deadlock
                solver.n_lazy_constraints["deadlock"] += 1
                add_mocp_deadlock_elimination_constraints(
                    solver,
                    variables,
                    constraints,
                    oc_graph,
                    variable_vals,
                    n_cables,
                )
                return

    return lazy_constraints


def add_mocp_isolated_subroute_constraints(
    solver,
    variables,
    constraints,
    oc_graph: ObjectCagingGraph,
    cycles,
    n_cables,
    lazy=True,
):
    logger.debug("Adding lazy constraints for %d isolated subroutes", len(cycles))
    for cycle in cycles:
        for t in range(n_cables):
            lhs1 = 0  # flow in direction 1
            lhs2 = 0  # flow in direction 2
            subroute_vertices = set()
            len_cycle = len(cycle)
            for i in range(len_cycle):
                v1 = cycle[(i) % len_cycle]
                v2 = cycle[(i + 1) % len_cycle]
                edge1_name = f"x{t + 1}_{v1},{v2}"
                edge2_name = f"x{t + 1}_{v2},{v1}"
                lhs1 += variables[edge1_name]
                lhs2 += variables[edge2_name]
                subroute_vertices.add(v1)
            lhs1 -= len_cycle - 1
            lhs2 -= len_cycle - 1
            rhs = 0  # flow into subroute
            for v in subroute_vertices:
                for nbh in oc_graph.neighbors(v):
                    if nbh in subroute_vertices:
                        continue
                    edge_name = f"x{t + 1}_{nbh},{v}"
                    rhs += variables[edge_name]
            constraint1_name = f"isolated_subroute: {cycle}, dir1 - cable_{t + 1}"
            constraint2_name = f"isolated_subroute: {cycle}, dir2 - cable_{t + 1}"

            if lazy:
                constraints[constraint1_name] = solver.add_lazy_constraint(
                    lhs1, "<=", rhs
                )
                constraints[constraint2_name] = solver.add_lazy_constraint(
                    lhs2, "<=", rhs
                )
            else:
                constraints[constraint1_name] = solver.add_constraint(
                    lhs1, "<=", rhs, constraint1_name
                )
                constraints[constraint2_name] = solver.add_constraint(
                    lhs2, "<=", rhs, constraint2_name
                )

        # solver.n_lazy_constraints["ise"] += 1


def add_mocp_deadlock_elimination_constraints(
    solver, variables, constraints, oc_graph: ObjectCagingGraph, variable_vals, n_cables
):
    logger.debug("Adding lazy constraint for encountered deadlock")
    lhs = 0
    for t in range(n_cables):
        # Loop over each edge in the oc_graph
        for n1, n2 in oc_graph.edges:
            name1 = f"x{t + 1}_{n1},{n2}"
            name2 = f"x{t + 1}_{n2},{n1}"
            val1 = variable_vals[name1]
            val2 = variable_vals[name2]
            if val1 > 0.9:
                lhs += 1 - variables[name1]
            else:
                lhs += variables[name1]
            if val2 > 0.9:
                lhs += 1 - variables[name2]
            else:
                lhs += variables[name2]

    constraint_name = f"deadlock_elimination"
    constraints[constraint_name] = solver.add_lazy_constraint(lhs, ">=", 1)
