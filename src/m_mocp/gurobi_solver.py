# Third-party
import gurobipy as gp
from gurobipy import GRB

class GurobiLazyContext:
    def __init__(self, solver, model):
        self.solver = solver
        self.model = model

    def get_value(self, var):
        return self.model.cbGetSolution(var)

    def add_lazy_constraint(self, expr):
        self.model.cbLazy(expr)

class GurobiSolver:
    VTYPE_MAP = {"C": GRB.CONTINUOUS, "B": GRB.BINARY, "I": GRB.INTEGER}

    def __init__(self, verbose=False, threads=None, filename=None):
        """
        Configures the Gurobi Environment.

        Args:
            verbose (bool): If True, prints Gurobi logs to console.
            threads (int): Number of CPU threads (None = auto).
            filename (Path):
        """
        self.verbose = verbose
        self.threads = threads

        if filename:
            self.env = gp.Env(empty=True, logfilename=filename)
        else:
            self.env = gp.Env(empty=True)
        self.env.setParam("OutputFlag", 1)
        self.env.setParam("LogToConsole", verbose)
        self.env.start()

        # The model lives here now
        self._model: gp.Model | None = None
        self._lp_model: gp.Model | None = None

        self._n_cuts = 0
        self.n_lazy_constraints = {}
        self.solve_log = {}
        self.root_bound: float | None = None
        self.root_obj: float | None = None

    @property
    def model(self):
        if self._model is None:
            raise RuntimeError("Model not initialized. Call initialize_model() first.")
        return self._model

    def initialize_model(self):
        """Resets the model for a new problem."""
        self._model = gp.Model(env=self.env)
        if self.threads is not None:
            self.model.setParam("Threads", self.threads)

    def set_time_limit(self, timeout_seconds: float):
        """Safely passes the remaining time limit to Gurobi."""
        # Gurobi requires a positive time limit.
        # If the remaining time is effectively zero, give it a tiny buffer
        # so it can gracefully start and immediately exit.
        safe_timeout = max(0.001, timeout_seconds)
        self.model.setParam("TimeLimit", safe_timeout)

    def add_variable(self, lb=0, ub=GRB.INFINITY, vtype="C", name=""):
        """
        Solver-Agnostic Variable Creator.

        Args:
            lb (float): Lower bound.
            ub (float): Upper bound (None = Infinity).
            vtype (str): 'C' (Continuous), 'B' (Binary), 'I' (Integer).
            name (str): Name for debugging.
        """
        # 1. Translate Bounds
        # Gurobi uses GRB.INFINITY, but formulation uses Python's None
        gurobi_ub = ub if ub is not None else GRB.INFINITY

        # 2. Translate Variable Type
        # Default to Continuous if the type isn't recognized
        gurobi_type = self.VTYPE_MAP.get(vtype, GRB.CONTINUOUS)

        return self.model.addVar(lb=lb, ub=gurobi_ub, vtype=gurobi_type, name=name)

    def add_constraint(self, lhs, sense, rhs, name=""):
        """
        Adds a constraint given as a math expression.
        Example: solver.add_constraint(x + y, "<=", 5)
        """
        if sense == "<=":
            self.model.addConstr(lhs <= rhs, name=name)
        elif sense == "==":
            self.model.addConstr(lhs == rhs, name=name)
        elif sense == ">=":
            self.model.addConstr(lhs >= rhs, name=name)
        else:
            raise ValueError(f"Invalid constraint sense: {sense}")

    def add_lazy_constraint(self, lhs, sense, rhs):
        """
        Adds a lazy constraint given as a math expression.
        Example: solver.add_constraint(x + y, "<=", 5)
        """
        self.model.cbLazy(lhs, sense, rhs)
        # return

    def set_objective(self, expression, minimize=True):
        sense = GRB.MINIMIZE if minimize else GRB.MAXIMIZE
        self.model.setObjective(expression, sense)

    def solve_lp_relaxation(self):
        self.model.update()
        self._lp_model = self.model.relax()
        self._lp_model.optimize()

        status = self._lp_model.Status
        if status == GRB.OPTIMAL:
            return True, "optimal"
        elif status == GRB.TIME_LIMIT:
            # Gurobi hit the time limit we set!
            return False, "timeout"
        elif status == GRB.INFEASIBLE:
            return False, "infeasible"
        else:
            return False, "other"

    def solve(self, lazy_callback=None):

        def internal_callback(model, where):
            if where == GRB.Callback.MIPNODE:
                if model.cbGet(GRB.Callback.MIPNODE_NODCNT) == 0:
                    # Get the current objective bound and best solution found so far
                    self.root_bound = model.cbGet(gp.GRB.Callback.MIPNODE_OBJBND)
                    self.root_obj = model.cbGet(gp.GRB.Callback.MIPNODE_OBJBST)

            if where == GRB.Callback.MIPSOL:
                if lazy_callback is not None:
                    context = GurobiLazyContext(self, model)
                    lazy_callback(context)

            if where == GRB.Callback.MIP:
                node_count = model.cbGet(gp.GRB.Callback.MIP_NODCNT)
                if node_count == 0:
                    # This will update continuously during root cuts
                    self.root_bound = model.cbGet(gp.GRB.Callback.MIP_OBJBND)
                    self.root_obj = model.cbGet(gp.GRB.Callback.MIP_OBJBST)
                # Get the current count of cutting planes applied
                num_cuts = model.cbGet(GRB.Callback.MIP_CUTCNT)
                # Store or print the count as needed
                if num_cuts > self._n_cuts:
                    self._n_cuts = num_cuts

        if lazy_callback is not None:
            # Required for lazy constraints
            self.model.setParam("LazyConstraints", 1)

        self.model.update()
        self.model.optimize(callback=internal_callback)

        # --- Collect final logging metrics ---
        self.solve_log = {
            "nodes": int(self.model.NodeCount),
            "cuts": int(self._n_cuts),
            "lazy_constraints": self.n_lazy_constraints,
            "vars": int(self.model.NumVars),
            "constraints": int(self.model.NumConstrs),
            "nonzeros": int(self.model.NumNZs),
            "runtime": float(self.model.Runtime),
            "root_bound": self.root_bound,
            "root_objective": self.root_obj,
        }

        # --- Map status ---
        status_map = {
            GRB.OPTIMAL: "optimal",
            GRB.TIME_LIMIT: "timeout",
            GRB.INFEASIBLE: "infeasible",
        }
        status_str = status_map.get(self.model.Status, "other")
        solved = self.model.Status == GRB.OPTIMAL
        if solved:
            self.solve_log["final_objective"] = self.model.ObjVal
            self.solve_log["final_gap"] = self.model.MIPGap
            self.solve_log["relaxation_gap"] = abs(
                self.solve_log["final_objective"] - self.solve_log["root_bound"]
            ) / abs(self.solve_log["final_objective"])

        return solved, status_str

    def get_value(self, variable, relaxed=False):
        if relaxed:
            if self._lp_model is None:
                raise RuntimeError("LP relaxation not solved.")
            idx = variable.index
            var = self._lp_model.getVars()[idx]
        else:
            var = variable
        return var.X

    def get_objective_value(self, relaxed=False):
        if relaxed:
            if self._lp_model is None:
                raise RuntimeError("LP relaxation not solved.")
            return self._lp_model.ObjVal
        else:
            return self.model.ObjVal

    def export(self, filename="model.lp", relaxed=False):
        target = self._lp_model if relaxed else self.model
        if target is None:
            raise RuntimeError("Model not initialized or relaxed.")
        target.update()
        target.write(filename)

    def export_log(self, filename="gurobi_metrics.json"):
        """Export captured metrics and presolve info to JSON."""
        import json

        with open(filename, "w") as f:
            json.dump(self.solve_log, f, indent=4)
