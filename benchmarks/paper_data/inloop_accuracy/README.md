# In-loop detection accuracy — tooling, geometry, and the blocker

Status 2026-09-16: tooling written and validated, experiment blocked on the
simulator side. No results yet.

## What the experiment measures

COCO accuracy says how good a detector is on a photograph given unlimited time.
It says nothing about the quality of the detections the controller receives. In
the closed loop the detector can skip frames, or deliver a box computed from a
frame that is already stale.

The model is deterministic, so its output for a given image cannot change under
load. Any in-loop accuracy change must therefore come from timing. That gives the
experiment its own bug check, and `benchmarks/inloop_accuracy.py` reports the
error twice:

- **at-capture**, against the drone pose when the frame was taken. This is
  detection quality proper and should be flat across detector configurations.
  If it moves, suspect a bug before believing a finding.
- **at-delivery**, against the pose when the detection reached the consumer.
  This is what the controller acts on, and should grow with detector latency.

Metrics are centroid error in pixels, miss rate and false-positive rate. IoU is
not reported: `/sim/target_pose` carries a 3D point, not an extent, so box
overlap against the reference is undefined.

## Geometry, established from data rather than assumed

Camera is pinhole, 640x480, `cx=320`, `cy=240`. Focal length is passed with
`--fx`: the Simulink camera block renders at `fx=fy=1200` (HFOV 29.9 deg) in the
current model and had 750 in the June model. The 554 in
`matlab/hil_ros_init_LT.m:34` is an assumed 60 deg FOV the block never used.
`/sim/drone_pose` carries `[x, y, z, pitch, yaw]` and `/sim/target_pose` carries
`[x, y, z, yaw]`.

The world bearing of an image column is

    bearing = yaw - atan2(u - cx, fx)

The sign of that term was chosen by measurement, not convention: fitting it to
the archived detections triangulates a single static sign with a 0.32 m bearing
residual, while the opposite sign gives 2.27 m.

## The target position: an apparent offset, most likely a different sign

`/sim/target_pose` is a hardcoded constant, `[35.5, 23.7, 3.2, pi]`
(`matlab/hil_ros_init_LT.m:81`), republished at ~1 Hz.

Fitting the position that reprojects onto the detections in the two archived
bags, both recorded with the June Simulink model:

| bag | fx | fitted x, y, z | RMS | drone parks at |
|---|---|---|---|---|
| `hil_full_20260606_002324` | 554 | 34.75, 3.14, 3.15 | 33.5 px | 32.51, 4.24 |
| `hil_full_20260606_002324` | 750 | 38.03, 1.51, 3.18 | 38.6 px | |
| `hil_full_20260606_002324` | 1200 | 46.04, -2.24, 3.31 | 46.5 px (fails) | |
| `hil_full_20260523_153453` | 554 | 33.77, 3.43, 2.87 | 78.4 px (fails) | 31.67, -1.64 |

The fitted y sits about 20 m from the published 23.7 at every focal length, so
the focal length does not explain the gap. What does fit:

- In both bags the drone itself parks at y = -2 to 4, beside the fitted sign,
  so those runs flew to a sign near y = 3.
- The scene holds a second stop sign (trap 1 below).
- The June model started the drone at integrator ICs x 0, y 20, z 5; the model
  used since July starts it at x -15, y 10, z 10.
- The paper's HIL figure (a July run) shows the drone at (30.6, 22.8), 5.0 m from
  the published constant with a 2.5 deg bearing error and the sign centred.

The likeliest reading is that the June runs approached the other sign, and the
constant is right for the current model. **Before scoring**, record one bag with
the current model and run `--triangulate --fx 1200` on it; the published
constant should agree. Only if it does not is a simulator-side change needed.

## Two traps found while validating, both now guarded

1. **A second stop sign in the scene.** 53 of 680 frames contain two, and
   "highest confidence" jumps between them. That contamination alone moved the
   reprojection RMS from 38 px to 46 px and initially made the geometry look
   irreconcilable. The estimator now uses only frames holding exactly one
   instance of the target class, above a confidence floor.
2. **A runaway local search.** Bearing-only geometry creates a long flat valley,
   and an unclamped search walks hundreds of metres down it and reports the
   result as an answer. The search is now a bounded grid plus a clamped refine,
   and it says when the best fit sits on the box edge.

The tool refuses to emit metrics when the reprojection RMS exceeds 40 px, since
nothing downstream of a failed geometry check is meaningful.

## Usage

```
pip install mcap mcap-ros2-support

# always run this first: does the geometry check out, and does the
# published target agree with what the detector sees?
python3 benchmarks/inloop_accuracy.py --bag RUN.mcap --triangulate

# score, once a trustworthy target position is known
python3 benchmarks/inloop_accuracy.py --bag RUN.mcap --target X,Y,Z \
    --telemetry <run>_telemetry.csv --label yolo26n_cpu --json-out out.json
```

`--telemetry` supplies true capture timestamps from `ts_capture`, which is what
separates the at-capture and at-delivery errors. Without it both collapse onto
the publish stamp and the bug check cannot run.

## Recording requirement

`/sim/drone_pose` and `/sim/target_pose` are in `FULL_EXTRA_TOPICS` in
`scripts/record_bag.sh` but not in the lightweight set. Detector-sweep runs must
either record in full mode or have those two topics added to the lightweight
list. They are small (~20 Hz and ~1 Hz) so adding them is the cheaper option.
