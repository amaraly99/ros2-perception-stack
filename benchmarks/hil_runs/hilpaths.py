"""hilpaths.py: the paths and pins of hil_env.sh for the Python tools. Same variable names, same defaults."""
import os

HIL_RUNS = os.path.dirname(os.path.abspath(__file__))
OURS = os.environ.get("HIL_OURS") or os.path.dirname(os.path.dirname(HIL_RUNS))
GEOM = os.environ.get("HIL_GEOM") or os.path.join(HIL_RUNS, "config", "sim_geometry.json")
HELD_REF = os.environ.get("HIL_HELD_REF") or os.path.join(HIL_RUNS, "config", "held_reference.json")
BENCH = os.path.join(OURS, "benchmarks")
PRODUCER = os.path.join(OURS, "src", "yolo_producer")
FIXTURES = os.path.join(HIL_RUNS, "fixtures")
