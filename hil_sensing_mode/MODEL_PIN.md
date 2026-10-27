# MODEL_PIN (2026-10-27 evening, 27W revised: RGB-D model check; no code change since c1a692b)

## Commits
Local branch `sim-25w` in the worktree `C:\Users\Ahmed\sim_25A`, author Ahmed Dhaouadi, not pushed (no upstream):
- 27W revised (2026-10-27 evening): adds this file as `hil_sensing_mode/MODEL_PIN.md`, with the RGB-D model check
  below. No other file changes; every sha256 in the table below re-read the same (sha256sum, 2026-10-27 evening).
- **c1a692b3d4793c93da4bd85635dcd0c2ceac6971** (27W): `'flight_repeats', 'on'` option. The stereo and RGB-D copies
  then send repeated frames in flight too, as the 3652892 copies did; without it they behave as at 7817471.
  Lightweight tag `paper-batch-c1a692b` pushed to origin 2026-10-27 (VERIFIED `git ls-remote`); the branch is not.
- **78174712f6bccfb40590ea0450bfd5a6aced79a5** (26W step 4): `'rate16'` option and `keep16.ps1`, the start in the 16 Hz
  publisher state with a 60 s held check of the send intervals.
- **5ab51d27d37d8fb7dc86470b0c9114cd91e18e75** (26W step 3): the three publisher copies re-send an unchanged frame only
  while the supervisor reports the model held; otherwise they behave as Amar's files.
- 3652892746bbb5ef7e43a3b4aeeb48893325288a (25W): adds `hil_sensing_mode/` (parent 28114bd).
The existing tree `C:\Users\Ahmed\hil_wt_unified` stays on sim/unified-start-2026-10-04 at 946e7a7, clean (VERIFIED
`git status --short`, 0 lines, 2026-10-26).

## Files the wrapper loads
sha256 of the files as run on this host, identical for `C:\Users\Ahmed\sim_25W\<file>` and
`C:\Users\Ahmed\sim_25A\hil_sensing_mode\<file>`. On disk the .m files under matlab\ are CRLF text; the wrapper and
keep16.ps1 are LF (VERIFIED 2026-10-27, CR byte count 0; the wrapper's 26W sha256 is that of the LF blob). The git
blob (in c1a692b, path `hil_sensing_mode/<file>`) is the line-ending-proof pin. "= 28114bd" means the blob is
identical to `matlab/<file>` at 28114bd. VERIFIED (sha256sum, git rev-parse, 2026-10-26; the three 27W rows
2026-10-27).

| file | sha256 | git blob | vs 28114bd |
|---|---|---|---|
| hil_session_start_heldstart_mode.m | d11c6763c9fd5500294ca6fa5bffa747711dbc9b05120409e07d871ad5fb5fee | 86e17552 | new (26W: 'rate16', 'check'; 27W: 'flight_repeats') |
| keep16.ps1 | a223006dcfc8aef823c5a2d336c5987e1ba8bf01711b970613f409f75eda01b9 | c813ca7a | new (26W), used only with 'rate16' |
| matlab/hil_closed_loop_unified.slx | 97b3a7e40a922c05cef669023d161cec192a1a6897386eda866ccc15215c246f | ffd3a4e2 | = 28114bd |
| matlab/hil_closed_loop_baseline_061.slx | 3e360771ea3eff7bc5916afeb8af7571c113f991055fefb944f8729b3b0140a3 | 1d23eca6 | = 28114bd |
| matlab/hil_closed_loop_baseline_RGBD.slx | 26fb1fb91d7f2290c3560ffca5515f170975f466b5147e8d7974b3957d01f6e8 | c2a515a7 | = 28114bd |
| matlab/hil_publish_frame.m | 21d2e3dc6f73a4454265068bc3c0ae7ca6cae7eb1ecc43e677aa9acecb25020c | 9451f0c2 | changed: idle throttle Inf while held, else 120 |
| matlab/sim_camera_publisher_timer_LT.m | f7266d787e9d7b2e34539602cd80639892535af7b357a42b6f0be483ff070fd1 | 625ed819 | changed: repeat skip off while held or HIL_FLIGHT_REPEATS=on, else on |
| matlab/sim_camera_publisher_timer_LT_rgbd.m | 9bf88301ffab0b8e5c4d3421c487b266a7c76b6cef6d365a399f1de3cf389c33 | 30c4e803 | changed: repeat skip off while held or HIL_FLIGHT_REPEATS=on, else on |
| matlab/hil_pub_control.m | b88197d5b5b989c1e1f59be2ecf1f781cfb3a000edfec8db9f57fed17c6909ec | 8a9ad1a6 | = 28114bd |
| matlab/hil_ros_init_LT.m | d9890be4753732388444f95b642e00c3f9c1b8deebbc9354109360d7964110f9 | ab100472 | = 28114bd |
| matlab/hil_ros_init_LT_rgbd.m | d591d0833de9f248432a32c041d949d7d15b9c6975f50fde65fdae9a3dd8ca47 | d49c8704 | = 28114bd |
| matlab/hil_run_supervisor.m | ac01d2b8a50d68b6be0cf3667b22ba9457294c514950ab9cebe6a16a36c60ab3 | 29b5efa6 | = 28114bd |
| matlab/read_cmdvel_live_interp.m | eedcc50d857df98bd20504542b41e8b587f0d6cb86ae17fa7b723d0fb847e36c | 925ededf | = 28114bd |
| matlab/set_rgbd.m | de7d772ca0296bb1033f0215a203f97b579716ad568110dc17e918f4e3919350 | 83ecec8a | = 28114bd |
| matlab/set_stereo.m | 259db2eb58b440f68766c156aebbc4e73eb02fa4416ba5f48d0a02f7bb701489 | 95852cd7 | = 28114bd |
| matlab/stop_hil_supervisor.m | 6ede2d7628917a4a06c345db9e1284b4ef323da610e868c889d42ab6b53c491f | 06fc9d59 | = 28114bd |

Superseded 25W pins (3652892): hil_session_start_heldstart_mode.m ad9838d8..., hil_publish_frame.m 2b17104f...,
sim_camera_publisher_timer_LT.m 9b8acaf4..., sim_camera_publisher_timer_LT_rgbd.m abf2e6ff....
Superseded 26W pins (7817471): hil_session_start_heldstart_mode.m 52d44618... (blob be56f551),
sim_camera_publisher_timer_LT.m 22c8fd64... (cf60b176), sim_camera_publisher_timer_LT_rgbd.m 008b3891... (7a2c73d4).

## Start, one line per mode (16 Hz state)
    cd C:\Users\Ahmed\sim_25W; hil_session_start_heldstart_mode('mono', 'rate16')
    cd C:\Users\Ahmed\sim_25W; hil_session_start_heldstart_mode('stereo', 'rate16')
    cd C:\Users\Ahmed\sim_25W; hil_session_start_heldstart_mode('rgbd', 'rate16')
Tell the Pi to go only after the line `[heldstart] 16 Hz state: OK`. Re-check a running session with
`hil_session_start_heldstart_mode('check')`.

With the in-flight behaviour of the 3652892 copies (27W; like-for-like with the stereo cells flown Oct 25):
    cd C:\Users\Ahmed\sim_25W; hil_session_start_heldstart_mode('rgbd', 'rate16', 'flight_repeats', 'on')
    cd C:\Users\Ahmed\sim_25W; hil_session_start_heldstart_mode('stereo', 'rate16', 'flight_repeats', 'on')
Mono needs no option: hil_publish_frame sends repeats in flight by Amar's design, and the option does not touch it.
The wrapper prints `[heldstart] flight_repeats on: repeated frames are sent in flight too`.

## RGB-D model check against Amar's branch (27W revised, 2026-10-27 evening), VERIFIED
- `git fetch origin sim/unified-start-2026-10-04` (no checkout, no merge): head 28114bd1b2a036e4b4ba79241295a7d55d35ca43,
  the commit the copies come from. matlab/hil_closed_loop_baseline_RGBD.slx there (last changed a988f10, Amar, 2026-10-25)
  has sha256 26fb1fb9..., byte-identical to the copy above. The RGB-D publisher copy differs from his file only in the
  SKIP_REPEATS lines (git diff 28114bd:matlab/... c1a692b:hil_sensing_mode/matlab/...); init and set_rgbd are equal.
- Model (copy of his blob loaded in -batch, sim_27W\data\model_check_27W.txt): camera `Simulation 3D Camera` on
  Quadrotor1, ImageSize [480, 640], FocalLength [554, 554], OpticalCenter [320, 240], no distortion, depth port on;
  `Simulation 3D Camera Right`, write_frame_right and write_depth_right commented. Image port to `MATLAB Function1`
  (assignin latest_frame), depth port to write_depth_left (assignin latest_depth_left). Fixed step 1/20, ode1.
- Colour: /sim/camera/image_raw, sensor_msgs/Image bgr8, 640x480, step 1920, frame_id 'camera' (hil_ros_init_LT.m:40-48,
  hil_ros_init_LT_rgbd.m:29; publisher :113-115 packs B, G, R). It is the left camera, the image the detector reads.
- Depth: /sim/camera/depth/image_raw, 16UC1, little-endian, step 1280, millimetres (metres x 1000, rounded),
  0 = invalid (non-finite, <= 0, sky, > 65.535 m) (hil_ros_init_LT_rgbd.m:31-40, publisher :119-126). Matches
  DepthMapFactor 1000. On the wire the held frame's median nonzero depth reads 19621 (the sign is about 20 m away).
- Stamps: both messages take one sim_heartbeat read, the Simulink clock (publisher :130-138; sim_clock_writer).
  On the wire (sim_27W\data\wire_table_27W.md) no stamp was outside the sent ones and every stamp seen on one stream
  only is within the messages lost on the other.

## How the simulator tells the Pi which model it runs, VERIFIED
- **Parameter reply over TCP, not heartbeat and not a file.** The Pi sends `model_info` to the supervisor on port 55556
  (repo: benchmarks/hilbench/supervisor.py:128-130, called before run 1 by session.py:142-156); MATLAB answers from
  the loaded model (matlab/hil_run_supervisor.m:328, `model_info_line` at 459). The Pi compares it with
  config/hil/models.yaml and its own copies of the .slx files (model_check.py:135-180). No hash is used (model_check.py:4).
- `/sim/heartbeat` is only the sim clock (sim_clock_writer chart; hil_ros_init_LT.m:215), no identity.
- **model_info does not change with the 26W commits.** The replies in the 26W held runs (sim_26W\data\held_*_events.txt,
  2026-10-26) are byte-identical to the 25W ones:
  - mono: `model_info model=hil_closed_loop_unified x_ic=15.12 y_ic=3.84 z_ic=7.5 yaw_ic=6.23718531 fx_left=554 fx_right=554 right_cam=off right_y=-0.11 depth_port=off left_image=rgb right_image=rgb`
  - stereo: `model_info model=hil_closed_loop_baseline_061 x_ic=15.12 y_ic=3.84 z_ic=7.5 yaw_ic=6.23718531 fx_left=554 fx_right=554 right_cam=on right_y=-0.61 depth_port=off left_image=rgb right_image=rgb`
  - rgbd: `model_info model=hil_closed_loop_baseline_RGBD x_ic=15.12 y_ic=3.84 z_ic=7.5 yaw_ic=6.23718531 fx_left=554 fx_right=554 right_cam=off right_y=-0.11 depth_port=on left_image=rgb right_image=rgb`
- **model_info does not change with the 27W commit or its option.** RGB-D with `'rate16', 'flight_repeats', 'on'`
  (sim_27W\data\held_rgbd_opt_controller.txt, 2026-10-27 03:11) replied, byte-identical to the rgbd line above:
  `model_info model=hil_closed_loop_baseline_RGBD x_ic=15.12 y_ic=3.84 z_ic=7.5 yaw_ic=6.23718531 fx_left=554 fx_right=554 right_cam=off right_y=-0.11 depth_port=on left_image=rgb right_image=rgb`
  The three RGB-D probe flights (sim_27W\data\flight_rgbd_*_controller.txt) and the stereo flight with the option
  (flight_stereo_opt) gave the rgbd and stereo lines above, byte for byte. Evening (27W revised): the held test with
  the option (wire_held_rgbd_opt) and the three wire flights (wire_rgbd_opt, wire_rgbd_opt2, wire_rgbd_3652892) replied
  the same rgbd line, byte for byte (sim_27W\data\wire_*_controller.txt). model_info_line reads model parameters and
  CAM_LEFT_IMAGE, CAM_RIGHT_IMAGE, RGBD_COLOUR_MONO only (hil_run_supervisor.m:459-495), not HIL_FLIGHT_REPEATS.
- What the Pi cannot see: any file hash, the commits, the publisher change, the option, or the 16 vs 20 Hz state. The
  reply is the same as with Amar's unchanged files.
