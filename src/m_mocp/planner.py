# Third-party
import networkx as nx

# Standard
import logging

# M-MOCP
from m_mocp.algorithms import (
    compute_trail,
    compute_uncoord_trail,
    compute_interlace_labels,
    get_cable_subgraphs,
    identify_isolated_subroutes,
    deadlock_exists,
)
from m_mocp.milp_formulations import (
    build_minimal_mocp_milp,
    build_mocp_lazy_ise_only,
    build_mocp_lazy_ise_and_deadlock,
)
from m_mocp.object_caging_graph import ObjectCagingGraph
from m_mocp.timer import Timer, GlobalTimeoutError

logger = logging.getLogger(__name__)

class Planner:
    """
    MILP-based Multi-Object Caging Planner.

    Attributes:
        oc_graph (ObjectCagingGraph): The graph representation of the scene.
        start_positions (List[int]): Initial position of each cable.
        n_cables (int): Number of cables.
        milp_solver : A wrapper for solving MILPs.
    """

    def __init__(
        self, oc_graph: ObjectCagingGraph, start_positions: list[int], milp_solver
    ) -> None:
        self.oc_graph: ObjectCagingGraph = oc_graph
        self.start_positions: list[int] = start_positions
        self.n_cables: int = len(self.start_positions)
        self.milp_solver = milp_solver
        self.timer: Timer
        logger.debug(
            "Initialized planner with %d cables starting from %s",
            self.n_cables,
            self.start_positions,
        )

    def plan(self, timeout_seconds, interlace_coordination=False):
        """
        Run the planner with specified timeout limit and interlace coordination assumption.
        """
        logger.info(
            "Planning with timeout: %.3f, interlace_coordination: %s",
            timeout_seconds,
            interlace_coordination,
        )
        self.timer = Timer(timeout_seconds)
        H: list[nx.DiGraph] | None = None
        B: dict[int, bool] | None = None
        trails: list[list[int]] | None = None
        try:
            # 1. Solve MILP
            milp_solution = self._solve_edge_assignment(interlace_coordination)

            # 2. Compute Cable Subgraphs
            H = get_cable_subgraphs(self.n_cables, self.oc_graph, milp_solution)
            identified_cycles = identify_isolated_subroutes(H, self.start_positions)
            if len(identified_cycles) > 0:
                raise RuntimeError(
                    "MILP solved with lazy constraints but solution contains isolated subroutes."
                )
            logger.info("Computed cable subgraphs.")
            for k, h in enumerate(H):
                logger.debug("Cable %d subgraph: %s", k + 1, h.edges)

            # 3. Compute Deadlock Free Cable Trails and Interlace Labels
            if interlace_coordination:
                trails = []
                for h, start in zip(H, self.start_positions):
                    trail = compute_trail(h, start, self.oc_graph)
                    if trail is None:
                        raise RuntimeError(
                            "MILP solved with lazy deadlock constraints but solution contains self-induced deadlock."
                        )
                    trails.append(trail)
            else:
                trails = []
                for h, start in zip(H, self.start_positions):
                    trail = compute_uncoord_trail(h, start, self.oc_graph)
                    if trail is None:
                        raise RuntimeError("MILP solved but trail computation failed. ")
                    trails.append(trail)
            B = compute_interlace_labels(self.oc_graph, trails)

            logger.info("Computed cable trails.")
            for k, trail in enumerate(trails):
                logger.debug("Cable %d trail: %s", k + 1, trail)

            logger.info("Computed interface labels.")
            logger.debug("Interface labels: %s", B)

            # 5. Verify No Deadlock
            if interlace_coordination and deadlock_exists(trails, B):
                status = "deadlock"
                raise RuntimeError(
                    "MILP solved with lazy deadlock constraints but solution contains coorination deadlock."
                )
            else:
                status = "success"
                logger.info("M-MOCP success.")

            return {
                "status": status,
                "runtime": self.timer.elapsed,
                "cable_subgraphs": H,
                "interlace_labels": B,
                "cable_trails": trails,
                "cost": self.milp_solver.get_objective_value(),
            }

        except RuntimeError as re:
            logger.warning("Termination Triggered %s", re)
            return {
                "status": "optimization failure",
                "runtime": self.timer.elapsed,
                "cable_subgraphs": H,
                "interlace_labels": B,
                "cable_trails": trails,
                "cost": None,
            }
        except GlobalTimeoutError as gte:
            logger.warning("Termination Triggered %s", gte)
            return {
                "status": "timeout",
                "runtime": self.timer.elapsed,
                "cable_subgraphs": H,
                "interlace_labels": B,
                "cable_trails": trails,
                "cost": None,
            }

    def _solve_edge_assignment(self, interlace_coordination) -> dict[str, float]:
        """
        Solves the edge assignment MILP based on interlace coordination assumption

        Args:
          interlace_coordination (bool): True if coordinated interlace, False if single-robot interlace

        Returns:
            dict[str,float]: dictionary mapping MILP variable name to values

        Raises:
            RuntimeError: If MILP cannot be solved successfully.

        """
        self.milp_solver.initialize_model()
        # Build base model
        variables, constraints = self._build_milp_model()

        if interlace_coordination:
            lazy_cb = self._build_lazy_cb_coord(variables, constraints)
        else:
            lazy_cb = self._build_lazy_cb_single_robot(variables, constraints)

        self.timer.check()
        self.milp_solver.set_time_limit(self.timer.remaining)
        logger.debug("Calling MILP solver with time limit: %.3f", self.timer.remaining)
        success_milp, status_milp = self.milp_solver.solve(lazy_cb)

        if not success_milp:
            logger.warning("MILP Not Successful. Status: %s", status_milp)
            raise RuntimeError("MILP could not be solved.")

        variable_vals = {
            name: self.milp_solver.get_value(var) for name, var in variables.items()
        }
        logger.debug("Found MILP solution.")
        return variable_vals

    def _build_milp_model(self):
        """Construct base MILP model and return variables, constraints."""
        variables, constraints = build_minimal_mocp_milp(
            self.milp_solver, self.oc_graph, self.start_positions
        )
        return variables, constraints

    def _build_lazy_cb_coord(self, variables, constraints):
        """Generate the MILP integer solution callback function for coordinated interlace."""
        lazy_cb = build_mocp_lazy_ise_and_deadlock(
            self.milp_solver,
            variables,
            constraints,
            self.n_cables,
            self.oc_graph,
            self.start_positions,
        )
        return lazy_cb

    def _build_lazy_cb_single_robot(self, variables, constraints):
        """Generate the MILP integer solution callback function for single-robot interlace."""
        lazy_cb = build_mocp_lazy_ise_only(
            self.milp_solver,
            variables,
            constraints,
            self.n_cables,
            self.oc_graph,
            self.start_positions,
        )
        return lazy_cb

    def export_milp_model(self, filename):
        """Export MILP model to file."""
        self.milp_solver.export(filename)

    def export_milp_solution(self, filename):
        """Export MILP solution to file."""
        self.milp_solver.export(filename)
