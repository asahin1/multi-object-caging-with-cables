# Third-party
import yaml

# Standard
from dataclasses import dataclass
from datetime import datetime
import json
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import sys

# M-MOCP
from m_mocp.gurobi_solver import GurobiSolver
from m_mocp.object_caging_graph import ObjectCagingGraph
from m_mocp.planner import Planner

logger = logging.getLogger(__name__)

@dataclass
class MMOCPConfig:
    timeout_seconds: int = 100
    data_dir_name: str = "data"
    graph_dir: str = "graphs"
    problem_dir: str = "problems"
    problem_file_ext: str = "problems.json"
    out_dir: str = "out"
    cable_trails_filename: str = "cables.jsonl"
    interlace_labels_filename: str = "interlace_labels.jsonl"
    metadata_filename: str = "metadata.json"
    milp_model_filename: str = "milp_model.lp"
    milp_solution_filename: str = "milp_solution.sol"


def setup_logging(verbose: bool, log_filename: str):
    """
    Configures a dual-logging system:
    - Console: Clean, user-friendly (INFO or DEBUG)
    - File: Detailed, timestamped (Always DEBUG)
    """

    # Get the logger for the package
    logger = logging.getLogger("m_mocp")
    logger.handlers = []  # remove existing (NullHandler).

    #  STOP propagation (This prevents the ROOT logger from double-printing)
    logger.propagate = False

    logger.setLevel(logging.DEBUG)  # Capture all

    # Formatters
    console_fmt = logging.Formatter(fmt="[%(name)s][%(levelname)s] %(message)s")
    file_fmt = logging.Formatter(
        "%(asctime)s - %(name)s - [%(levelname)s] - %(message)s (%(filename)s:%(lineno)d)"
    )

    # Console Handler
    ch = logging.StreamHandler(sys.stdout)
    ch.setLevel(logging.DEBUG if verbose else logging.INFO)
    ch.setFormatter(console_fmt)

    # File Handler
    fh = RotatingFileHandler(
        log_filename, maxBytes=5 * 1024 * 1024, backupCount=3, encoding="utf-8"
    )
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(file_fmt)

    logger.addHandler(ch)
    logger.addHandler(fh)


def main():
    import argparse

    # Parse problem name and time stamp
    parser = argparse.ArgumentParser()
    parser.add_argument("context_name", help="Which context to load from")
    parser.add_argument("problem_name", help="Which problem to load")
    parser.add_argument(
        "-c",
        "--coordination",
        action="store_true",
        help="Require coordinated interlace if provided",
    )
    parser.add_argument(
        "--config_file_path",
        required=False,
        help="Which config to load",
        default="config/config.yaml",
    )
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    config_file = Path(args.config_file_path)
    if not config_file.exists():
        raise FileNotFoundError(f"Config file not found: {config_file}")

    with open(config_file, "r") as f:
        config_data = yaml.safe_load(f) or {}

    mmocp_config = MMOCPConfig(**config_data)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    result_path = (
        Path(mmocp_config.out_dir) / args.context_name / args.problem_name / timestamp
    )

    problem_file_name = args.context_name + "_" + mmocp_config.problem_file_ext
    problem_file_path = (
        Path(mmocp_config.data_dir_name) / mmocp_config.problem_dir / problem_file_name
    )

    # Load Problem
    problem_instances = {}
    with open(problem_file_path, "r") as problem_file:
        problems = json.load(problem_file)
        for problem_name in problems:
            if problem_name.startswith(args.problem_name):
                problem_instances[problem_name] = problems[problem_name]
        if len(problem_instances) == 0:
            print(f"Problem {args.problem_name} not found in {problem_file_path}.")
            return

    for problem_name, problem_data in problem_instances.items():
        graph_file_name = problem_data["graph"]["file"]
        graph_name = problem_data["graph"]["name"]

        graph_file_path = (
            Path(mmocp_config.data_dir_name) / mmocp_config.graph_dir / graph_file_name
        )

        # Load and construct graph
        try:
            graph = ObjectCagingGraph.from_file(graph_file_path, graph_name)
        except KeyError as e:
            print(f"%s")
            return

        start_positions = problem_data["init_config"]

        # Validate start positions (degrees)
        for i, v in enumerate(start_positions):
            degree = graph.degree(v)
            if degree != 2:
                raise RuntimeError(
                    f"Cable {i + 1} enters from a vertex with degree {degree}."
                )

        instance_result_path = result_path / problem_name

        os.makedirs(instance_result_path, exist_ok=True)
        metadata_file = instance_result_path / mmocp_config.metadata_filename
        setup_logging(args.verbose, instance_result_path / "process.log")

        # Initialize MILP solver
        solver = GurobiSolver(
            verbose=args.verbose,
            filename=str(instance_result_path / "gurobi_verbose.log"),
        )

        # Initialize the MMOCP and plan
        planner = Planner(graph, start_positions, solver)
        result = planner.plan(
            mmocp_config.timeout_seconds, interlace_coordination=args.coordination
        )
        if result["status"] == "success":
            logger.info(
                "M-MOCP Success. Find results at %s",
                instance_result_path,
            )
        else:
            logger.warning("Valid cable action sequences could not be found")
            logger.info("Planner status: %s", result["status"])

        if result["cable_trails"] is not None:
            cable_trails_file = (
                instance_result_path / mmocp_config.cable_trails_filename
            )
            with open(cable_trails_file, "w") as f:
                json.dump([trail for trail in result["cable_trails"]], f, indent=4)

        if result["interlace_labels"] is not None:
            interlace_labels_file = (
                instance_result_path / mmocp_config.interlace_labels_filename
            )
            with open(interlace_labels_file, "w") as f:
                json.dump(result["interlace_labels"], f, indent=4)

        metadata = {
            "timestamp": timestamp,
            "problem_filename": problem_file_name,
            "problem_name": args.problem_name,
            "graph_filename": graph_file_name,
            "graph_name": graph_name,
            "status": result["status"],
            "runtime": result["runtime"],
            "cost": result["cost"],
            "timeout_limit": mmocp_config.timeout_seconds,
            "interlace_coordination": args.coordination,
        }
        with open(metadata_file, "w") as f:
            json.dump(metadata, f, indent=4)

        solver.export_log(instance_result_path / "custom_gurobi_log.json")

        milp_model_file = str(instance_result_path / mmocp_config.milp_model_filename)
        planner.export_milp_model(milp_model_file)
        if result["status"] == "success":
            milp_solution_file = str(
                instance_result_path / mmocp_config.milp_solution_filename
            )
            planner.export_milp_solution(milp_solution_file)
