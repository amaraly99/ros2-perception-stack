# REPORT_25W (2026-10-25)

**NOT GO.** Mono is GO. Stereo and RGB-D hold correctly but send no images while held: Amar's publishers skip repeated frames (sim_camera_publisher_timer_LT.m:173-176, _rgbd.m:93-96), so the Pi's detector cannot place a box. VERIFIED (test_stereo.txt, test_rgbd.txt). Fixing it edits tracked code.

Repo `C:\Users\Ahmed\hil_wt_unified` (946e7a7, clean). Worktree `C:\Users\Ahmed\sim_25A`, detached at 28114bd (origin head, contains f27918c).

| mode | model | cameras (fx 554, 640x480) | topics | sent while held | held pose |
|---|---|---|---|---|---|
| mono | hil_closed_loop_unified | left | /sim/camera/image_raw bgr8 | 19.8 Hz | PASS |
| stereo | hil_closed_loop_baseline_061 | left, right y -0.61 | + /sim/camera/right/image_raw bgr8 | 0 Hz | PASS |
| rgbd | hil_closed_loop_baseline_RGBD | left, left depth | + /sim/camera/depth/image_raw 16UC1 mm | 0 Hz | PASS |

PASS, VERIFIED: paused at t 0.30, pose (15.12, 3.84, 7.50), yaw 357.36 deg, 20.001 m from the sign. Stereo and RGB-D sent 4 frames per Pi re-hold, then nothing.

Start: `cd C:\Users\Ahmed\sim_25W; hil_session_start_heldstart_mode('mono')`, or 'stereo' or 'rgbd'.

Switching modes without restarting MATLAB works (RGB-D to mono, held, 19.8 Hz), but its first held frame differed slightly (trees); prefer a fresh MATLAB.

Mono vs current model, VERIFIED: block diagram byte-identical, held frame pixel-identical (sha256 3b26f933), same hold code and publisher. Differs only in model name, worktree scripts (supervisor adds probe_* and model_info; probe_* refused) and build cache (sim_25W\simgen).

hil_session_start_slam starts the model unheld; Verso flies only on the Pi's probe_init.

Mono left running, held. Fig. 3: fig3_model.png. Details: READING_25W.md.
