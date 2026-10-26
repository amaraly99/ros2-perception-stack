# READING_25W: step 2 record (2026-10-25)

Paths: worktree `C:\Users\Ahmed\sim_25A` (detached at 28114bd), files below are in its `matlab\` unless
marked HIL (live folder `...\MATLAB\HIL`, not a git repo) or "existing tree" (`C:\Users\Ahmed\hil_wt_unified`,
branch sim/unified-start-2026-10-04 at 946e7a7, clean). XML line numbers refer to the unzipped models in
`sim_25W\inspect\<name>\simulink\systems\system_root.xml` (unzipped copies, never opened in MATLAB).

## hil_session_start_slam.m (head), VERIFIED
- Model per mode from config/hil/models.yaml (`hs_pick_model`, lines 253-281; called at 69-71):
  models.yaml:9 mono `hil_closed_loop_unified`, :10 rgbd `hil_closed_loop_baseline_RGBD`,
  :16 stereo 0.61 `hil_closed_loop_baseline_061`.
- Stereo: `set_stereo(1)` then `hil_ros_init_LT` (113-115). RGB-D: `set_rgbd(1)` then `hil_ros_init_LT_rgbd` (116-118).
- Starts the model LIVE, not held: pacing on, `SimulationCommand start` (136-137), waits for a frame (138-150).
  No `sim_hold_at` anywhere in the file.
- Publisher: `sim_camera_publisher_timer_LT` or `_rgbd` (152-158), then `hil_run_supervisor` (161).
- Does NOT itself fly the scripted trajectory: it blocks the MATLAB prompt in `hs_probe_loop` (176, 180-245) and
  runs `slam_traj_probe_unified` only when the Pi sends `probe_init` / `probe_mission` (195-235). That probe is the
  Verso path: sideways init leg (slam_traj_probe_unified.m:185-186, 286), 6-DOF helix cruise (328-354), commands
  written to `sim_cmdvel` (520-522). The prompt's INFERRED "most likely starts that trajectory" is therefore
  corrected: it arms it and runs the model unheld.
- Environment differs from ours: Fast DDS profile resolved with Wi-Fi IPs 192.168.1.201 / 192.168.1.60 (72-73, 94-106),
  build cache in tempdir (107-108), `clear all` (64).

## Head supervisor (hil_run_supervisor.m), VERIFIED
- Model from env `HIL_MODEL` (111). Held-start code unchanged from ours: HOLD_AT_S 0.25 (153), `sim_hold_at` set
  before `start` (175, 180), `start_hold` (262), `resume` (280). Diff 946e7a7..28114bd only adds commands.
- `pub_start`: RGB-D session runs `sim_camera_publisher_timer_LT_rgbd` (344-351), stereo session runs
  `sim_camera_publisher_timer_LT` (352-357), otherwise `hil_pub_control('start', hz)` as before (359).
- `probe_*` answers `error no_loop` unless the probe loop runs (412-421).

## Our session start (existing tree = HIL except one line), VERIFIED
- Launch `matlab -sd HIL -r "hil_ros_init_LT; hil_run_supervisor"`; the Pi then sends `pub_start 20`, `start_hold`
  (HIL/logs/supervisor_2026-10-20_1228_main.log:27-29; 258 of 258 pub_start lines in HIL/logs are "pub_start 20").
- Start pose comes from the model's integrator ICs, not from a script. Hold: `read_cmdvel_live_interp.m:51-66`
  pauses the model when sim time reaches `sim_hold_at` (0.25 s); status then reads t=0.30.
- hil_ros_init_LT.m: HIL line 19 hard-sets `rmw_fastrtps_cpp`, existing tree and head line 19 pass the env through
  (only diff, `diff` output). Left publisher `/sim/camera/image_raw` bgr8 640x480 best effort (40-48); right
  `/sim/camera/right/image_raw` bgr8 only if STEREO_ON (61-86); `/cmd_vel` subscriber (100-102); target pose
  (35.1, 2.92, 3.08) (121-126); 20 Hz state timer (145-153).
- Publisher in our runs: `hil_pub_control` (timer 1/hz, 38-46) + `hil_publish_frame`: re-sends the held frame every
  tick (93-110) and only goes quiet after 120 s of identical frames (19). Same file at HIL, 946e7a7 and 28114bd.

## Amar's extra-camera publishers, VERIFIED
- `sim_camera_publisher_timer_LT.m`: 20 Hz (26); skips a tick whose left frame equals the last one sent (173-176),
  i.e. a paused (held) model is not re-sent; right eye sent with the left, one stamp (232-236).
- `hil_ros_init_LT_rgbd.m`: depth `/sim/camera/depth/image_raw`, 16UC1 mm, best effort, frame_id camera (31-41).
- `sim_camera_publisher_timer_LT_rgbd.m`: 20 Hz (16), same duplicate skip (93-96), colour+depth together (148-154).

## Models (XML diff against the live HIL model a38a664c), VERIFIED
- `hil_closed_loop_unified.slx` (97b3a7e4): system_root.xml byte-identical to the live model; blockdiagram.xml
  differs only in ModelVersionFormat.
- `hil_closed_loop_baseline_061.slx` (3e360771): live model with the right camera mount y = -0.61
  (tmountOffset line 124, mountPoint 140); right camera stored commented out (104).
- `hil_closed_loop_baseline_RGBD.slx` (26fb1fb9): live model with left `DepthOutportEnabled` on (88) and block
  `write_depth_left` (336) writing `latest_depth_left`; right camera commented out, mount -0.11.
- All three: left camera fx [554, 554] (80), 480x640 (82), optical centre [320, 240], mount [0.2 0 0.2] (76/92),
  rotation [0 5 0], ISO8855, inherited sample time; right camera fx 554, 480x640, rotation [0 5 0];
  ICs x 15.12, y 3.84, z 7.5, yaw 2*pi-0.0460 (b061 lines 366-384); `read_cmdvel_live_interp(u(1..6))` (291);
  Accelerator mode, fixed step 1/20, StopTime inf, EnablePacing on (configSet0.xml:11-12, blockdiagram.xml:13,15).

## Follow-up (2026-10-25 evening): every place that skips an identical frame or stops on an unchanged one

Model side, VERIFIED (unzipped stateflow charts): write_frame, write_frame_right, write_depth_right,
write_depth_left and sim_clock_writer each do one unconditional `assignin` per step. Nothing to change.

MATLAB side, VERIFIED at 28114bd (line numbers of matlab\ there):
| place | behaviour | action in sim_25W\matlab |
|---|---|---|
| hil_publish_frame.m:19, 93, 100-101 | after 120 s of an unchanged frame, sends nothing | IDLE_THROTTLE_SEC = Inf |
| sim_camera_publisher_timer_LT.m:173-178 | skips a tick whose left frame equals the last sent (stereo pair) | SKIP_REPEATS = false |
| sim_camera_publisher_timer_LT_rgbd.m:93-98 | same skip (colour + depth) | SKIP_REPEATS = false |
| sim_camera_publisher_timer_LT_rgbd.m:138-146 | drops a repeated stamp only if env RGBD_DROP_DUP_STAMPS=1 | wrapper sets it empty |
| hil_publish_frame.m:49-73 + hil_pub_control.m:26-37 | after pub_start, skips the image already in latest_frame until the first new one, once | kept: one-shot, never re-engages; every re-hold armed before the hold (test_fu_*.txt B2) |
| hil_publish_frame.m:111-121 | stops re-sending after 15 s of an EMPTY frame (not an unchanged one) | kept: not a held case |
The other 11 copies are byte-identical to 28114bd (git hash-object = blob at 28114bd:matlab/<file>).

Re-test, 3 minutes held each, this host (test_fu_*.txt, logs\session7..12):
| run | L | R | D | after 120 s | pose |
|---|---|---|---|---|---|
| fu_mono (19:38) | 16.04 | - | - | 16.04 | exact |
| fu_mono2 (19:58) | 19.84 | - | - | 19.85 | exact |
| fu_stereo (19:48), fu_stereo2 (20:03) | 16.08, 16.07 | 16.08, 16.07 | - | 16.07, 16.05 | exact |
| fu_rgbd (19:53) | 19.82 | - | 19.82 | 19.82 | exact |
A/B in one MATLAB (19:44): unchanged 28114bd publisher 16.04 Hz, changed copy 16.03 Hz (ab_control\test_ab_original_mono.txt,
test_ab_changed_mono.txt), so the change does not alter the mono rate; 16 vs 19.8 Hz is the host's state at the time
(INFERRED: a Pi subscriber over Wi-Fi; 25W's 19.84 Hz and Pi-3's 16.1 Hz fit). Stereo is 16.1 Hz in both runs: two
images per tick, overrunning ticks dropped by the timer (BusyMode drop), INFERRED. MATLAB memory change over each
3-minute hold: -10 to +8 MB. Held frames: left 3b26f933 in every run, right 861cce9e.
