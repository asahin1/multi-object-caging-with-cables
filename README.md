# MILP-based Multi-Object Caging Planner (M-MOCP)

## Key Dependencies
- gurobipy
- networkx
- numpy
- PyYAML

## Setup

Create and activate a virtual environment:

```bash
python3 -m venv .venv
source .venv/bin/activate   # Linux/macOS
pip install e . # For editable mode
```

A license should not be necessary for solving most of the problem instances included in this repository. In case it is needed, a free academic license for Gurobi can be obtained from [here](https://www.gurobi.com/academics). Once a license file is downloaded, you need to run the following to set the license file path for Gurobi in your terminal session:

```bash
export GRB_LICENSE_FILE=<full-path-to-your-.lic-file>
```

## Run
For running the M-MOCP on a single problem instance:

```bash
m-mocp [<context_name>] [<problem_name>] [-c] # Use -c flag for requiring coordinated interlace
# Example: m-mocp 5_obj graph_scene1-m4-i1 -c
```

For running all problem instances, see targets in [makefile](./makefile).

## Available Context and Problem Names
Following contexts are available, specifying the number of objects in the scene: `2_obj`, `3_obj`, `4_obj`, `5_obj`, `6_obj`, `7_obj`.
Problem names follow the format: `graph_scene<scene-no>-m<n-cables>-i<instance-no>`

View the `.json` files in [data/graphs](./data/graphs) and [data/problems](./data/problems) for details.

