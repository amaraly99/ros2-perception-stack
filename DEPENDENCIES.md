# Dependencies of this loss-window ablation branch

This branch is the pinned stack commit `4490660` plus the integration state of the loss-window ablation batch
(session 5a, 2026-10-16), whose one behavioural change is the `detection_lost_sec` parameter in the servo FSM. Its
default reproduces the pinned rule exactly, 2 x `watchdog_period_sec` = 0.4 s. It carries source, configuration and
the model registry only; the run code, batch runner and scorer live in the branch `exp/detector-accuracy` under
`benchmarks/hil_runs/` and `benchmarks/hil_tools/`.

The integration state in this commit reproduces
`24b84f65315e4fbcd274c4b71570a9ad2f9677bce723672051b987c1ce56d31d`, the sha256 of
`integration/ablation_lw/ours_on_4490660_lw.patch` in the run-code branch, which is the patch the ablation batch
applied.

## Left out of this branch on purpose

- `opencv/` and `opencv_contrib/`: **OpenCV 4.10.0** built from source in the worktree root, about 4 GB, ignored by
  `.gitignore` since before the pin. The C++ nodes load it inside the stack image through
  `LD_LIBRARY_PATH=/workspace/opencv/build/lib`. The Pi also carries apt `libopencv-*` 4.10.0+dfsg-5, which the
  container does not use.
- Detector model binaries: every `.hef` and `.onnx` that `models/model_registry.json` names under `models/hef/` and
  `models/onnx/`. The registry is in this branch and is the single source of truth for ids, paths and metadata; the
  binaries are not. HEF files are Hailo Model Zoo v5.1.0 builds for the Hailo-10H; the ONNX files come from the two
  export scripts the registry names.
- `build/`, `install/`, `log/`, `bags/`: build output and recorded bags, ignored.
- Run data: the per-run records live outside this repository, under `/mnt/usb/hil_runs/<batch>/`.

## Already in the base commit 4490660, not added here

Listed so the reorganization knows they are third-party and where they sit.

- `src/orbslam3/orb_slam3/Thirdparty/Sophus` and `src/ov2slam_ros/Thirdparty/Sophus`: **Sophus 1.1.0**.
- `src/ov2slam_ros/Thirdparty/ceres-solver`: **Ceres Solver 2.0.0**.
- `src/ov2slam_ros/Thirdparty/`: also `opengv`, `obindex2`, `ibow_lcd`, `backward-cpp`.
- `src/orbslam2/Thirdparty/` and `src/orbslam3/orb_slam3/Thirdparty/`: `DBoW2` and `g2o`.

## Environment the runs used

From `benchmarks/hil_tools/README.md`, section "Environment pins", verified on this Pi 2026-10-17:
Raspberry Pi 5 (8 GB) with the AI HAT+ 2 (Hailo-10H), kernel 6.18.50+rpt-rpi-2712, HailoRT 5.1.1, stack image
`ros2_perception_stack:latest` 195667c737d4 (ROS 2 Jazzy, `rmw_fastrtps_cpp`), run-code Python 3.13.5 with
mcap 1.4.0 and mcap-ros2-support 0.5.7, host tools Python 3.13.5 with numpy 2.2.4, scipy 1.17.1,
matplotlib 3.10.8, OpenCV 4.13.0 (pip wheel) and onnxruntime 1.24.4, hailo_platform 5.1.1.
