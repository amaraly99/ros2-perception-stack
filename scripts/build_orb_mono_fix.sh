#!/bin/bash
# Build the ORB-SLAM2/3 mono fix libraries from scripts/orb_mono_fix.patch (2026-10-25).
#
# The fix (paper: Verso mono results, rows marked *):
#   ORB-SLAM2  Initializer.cc   RH > 0.40  -> 0.50          (value of the ORB-SLAM3 code)
#   ORB-SLAM2  Tracking.cc      first-map gate 100 -> 50   (value of the ORB-SLAM3 code)
#   ORB-SLAM2  LocalMapping.cc  new-point parallax gate cos < 0.9998 -> 0.9987 (2.9 deg)
#   ORB-SLAM3  LocalMapping.cc  same parallax gate, mono (non-inertial) branch only
#
# What it does: applies the patch, builds both libraries in their Docker images, saves them as
#   src/orbslam2/lib/libORB_SLAM2.so.fix_clean
#   src/orbslam3/install/ros2_orb_slam3/lib/liborb_slam3_lib.so.fix_clean
# then restores the original sources. The installed libraries are NOT changed; the campaign driver
# ~/hil_campaign/orbfix_campaign.sh installs the .fix_clean copy for its runs and restores the original.
#
# Usage (repo root):  scripts/build_orb_mono_fix.sh
set -euo pipefail
R=$(cd "$(dirname "$0")/.." && pwd)
P=$R/scripts/orb_mono_fix.patch
cd "$R"
git apply --check -p1 "$P"
restore(){ git apply -R -p1 "$P" 2>/dev/null || true; }
trap restore EXIT
git apply -p1 "$P"
# make rebuilds only files newer than their objects: touch the patched files so they always rebuild
touch src/orbslam2/src/Initializer.cc src/orbslam2/src/Tracking.cc src/orbslam2/src/LocalMapping.cc \
      src/orbslam3/orb_slam3/src/LocalMapping.cc

# ORB-SLAM2: its build folder was configured with src/orbslam2 mounted at /workspace
L2=src/orbslam2/lib/libORB_SLAM2.so
O2=$(sha256sum $L2 | cut -c1-16)
sudo docker run --rm --entrypoint "" -v "$R/src/orbslam2":/workspace orbslam2_fixed \
  bash -c "cd /workspace/build && make -j3 ORB_SLAM2" > /tmp/build_orb2_fix.log 2>&1 || { echo "ORB-SLAM2 build failed, see /tmp/build_orb2_fix.log"; exit 1; }
sudo cp $L2 $L2.fix_clean

# ORB-SLAM3: the Pangolin package was built at /workspace/src/Pangolin, hence the second mount
L3=src/orbslam3/install/ros2_orb_slam3/lib/liborb_slam3_lib.so
sudo docker run --rm --entrypoint "" -v "$R":/workspace -v "$R/src/orbslam3/Pangolin":/workspace/src/Pangolin orbslam3_container \
  bash -c "source /opt/ros/jazzy/setup.bash && cd /workspace/src/orbslam3/build/ros2_orb_slam3 && make -j3 orb_slam3_lib" > /tmp/build_orb3_fix.log 2>&1 \
  || { echo "ORB-SLAM3 build failed, see /tmp/build_orb3_fix.log"; exit 1; }
sudo rmdir "$R/src/Pangolin" 2>/dev/null || true
sudo cp src/orbslam3/build/ros2_orb_slam3/liborb_slam3_lib.so $L3.fix_clean

# ORB-SLAM2 builds straight into lib/: put the original back (the trap restores the sources)
restore; trap - EXIT
touch src/orbslam2/src/Initializer.cc src/orbslam2/src/Tracking.cc src/orbslam2/src/LocalMapping.cc src/orbslam3/orb_slam3/src/LocalMapping.cc
sudo docker run --rm --entrypoint "" -v "$R/src/orbslam2":/workspace orbslam2_fixed \
  bash -c "cd /workspace/build && make -j3 ORB_SLAM2" > /tmp/build_orb2_orig.log 2>&1
echo "ORB-SLAM2 original lib before: $O2  after: $(sha256sum $L2 | cut -c1-16)"
echo "fix libraries: $(sha256sum $L2.fix_clean | cut -c1-16) (ORB-SLAM2)  $(sha256sum $L3.fix_clean | cut -c1-16) (ORB-SLAM3)"
python3 - "$L2.fix_clean" "$L3.fix_clean" <<'PY'
import struct, sys
for f in sys.argv[1:]:
    d = open(f, "rb").read()
    print(f.split("/")[-1], "gate 0.9987 present:", d.count(struct.pack("<d", 0.9987)) > 0)
PY
