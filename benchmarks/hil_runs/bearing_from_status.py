#!/usr/bin/env python3
"""Bearing error from a v3 supervisor `status` line to the frozen target.

usage: bearing_from_status.py "<status line>"
prints: "<bearing_err_deg> <yaw_deg> <x> <y> <z>"   exit 0 if |err|<=5, 1 if not, 2 if unparseable
"""
import json, math, os, re, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hilpaths
_G = json.load(open(hilpaths.GEOM))            # 5Q: target and tolerance from sim_geometry.json (were constants, same values)
TARGET = tuple(_G["target_xyz"])
TOL = float(_G["start_bearing_tol_deg"])
s = " ".join(sys.argv[1:])
f = {}
for k in ("yaw", "x", "y", "z"):
    m = re.search(rf"\b{k}=(-?\d+(?:\.\d+)?)", s)
    if not m:
        print("unparseable", file=sys.stderr); sys.exit(2)
    f[k] = float(m.group(1))
bearing = math.degrees(math.atan2(TARGET[1] - f["y"], TARGET[0] - f["x"]))
err = (bearing - f["yaw"] + 180) % 360 - 180
print(f"{err:.3f} {f['yaw']:.2f} {f['x']:.2f} {f['y']:.2f} {f['z']:.2f}")
sys.exit(0 if abs(err) <= TOL else 1)
