# EXPECTED (written 2026-10-25 before reading Amar's sim/unified-start-2026-10-04 head)

Basis: the live mono model (HIL/hil_closed_loop.slx, sha256 a38a664c...), its last held session
(HIL/logs/supervisor_2026-10-21_1943_main.log: "[cam-LT] 16.0 to 16.3 Hz sent" while held), and the
prompt (rendered camera 640x480, fx = 554 px, 60 deg field). Everything below is a prediction, not a result.

## Per mode

| mode | image topics | resolution / fx | encoding | send rate on this host |
|---|---|---|---|---|
| mono | /sim/camera/image_raw only | 640x480, fx = fy = 554 | bgr8 | about 16 Hz (same timer as today) |
| stereo | /sim/camera/image_raw + /sim/camera/right/image_raw | both 640x480, fx 554; right eye 0.61 m to the right (ISO 8855 Y = -0.61) | bgr8 both | lower than mono per eye, guess 8 to 14 Hz, eyes in step |
| rgbd | /sim/camera/image_raw + a depth topic (guess /sim/camera/depth/image_raw) | 640x480, fx 554, depth aligned with the left eye | depth 32FC1 (m) or 16UC1 (mm) | lower than mono, guess 8 to 14 Hz |

All modes: state topics /sim/drone_pose, /sim/pitch_angle, /sim/heartbeat at 20 Hz (hil_ros_init_LT),
/cmd_vel subscribed. Held pose x 15.12, y 3.84, z 7.5, yaw about 357.4 deg, which is 20.0 m horizontal
from the frozen sign at (35.1, 2.92). Expected: hil_session_start_slam starts Amar's scripted "Verso"
trajectory, so the wrapper must bypass that part.

## NOT GO if any of these holds
1. A mode does not reach the hold, or the drone pose while held differs from (15.12, 3.84, 7.5) by more than 0.05 m.
2. Anything moves the drone while held (scripted trajectory, nonzero /cmd_vel integration).
3. In mono, the left camera differs from the live model in topic, resolution, fx, encoding, mount or publish timer.
4. A mode's expected image topics are not published, or publish at 0 Hz.
5. The wrapper can only work by editing a file tracked at the head, or the live HIL tree/model.
6. A session cannot be shut down cleanly before the next one starts.

## Follow-up (written 2026-10-25 19:40, before any change)

Expected change: only the MATLAB publisher code, in copies under sim_25W\matlab (the wrapper loads that
folder instead of the worktree). The models need no change: their frame writers (write_frame,
write_frame_right, write_depth_right, write_depth_left) assign on every step without a test.
- hil_publish_frame.m (mono): the 120 s idle throttle goes, so a held frame keeps going out every tick.
- sim_camera_publisher_timer_LT.m (stereo) and sim_camera_publisher_timer_LT_rgbd.m (RGB-D): the
  "same as last frame" skip goes, so every valid tick sends, as hil_publish_frame already does.

Expected send rates while held, 3 minutes, this host:
| mode | before (25W) | after |
|---|---|---|
| mono | 19.8 Hz, then 0 after 120 s unchanged | 19.8 Hz all 180 s (same timer, within 5 %) |
| stereo | 0 Hz (4 pairs per re-hold) | L and R 12 to 20 Hz each, L = R count |
| rgbd | 0 Hz (4 pairs per re-hold) | colour and depth 12 to 20 Hz each, C = D count |

Held pose unchanged: (15.12, 3.84, 7.50), yaw 357.36 deg, 20.001 m.
NOT GO if: any topic falls to 0 Hz after 120 s; eyes or depth out of step; mono differs from 19.8 Hz by
more than 5 %; pose moves; MATLAB memory grows by more than 1 GB over the 3 minutes (the throttle was added
because a long idle stream once ran MATLAB's ROS 2 layer out of memory, hil_publish_frame.m:7-12, 86-92).
