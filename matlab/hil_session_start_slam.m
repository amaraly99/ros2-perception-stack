function hil_session_start_slam(mode, baseline, left_image, right_image)
% hil_session_start_slam.m — prepares MATLAB for a Pi-driven SLAM HIL session (benchmarks/slam_runner.py).
% The controller benchmark keeps using hil_session_start.m, which this file does not change.
%
%   >> cd <repo>\matlab
%   >> hil_session_start_slam('mono')                      % mono
%   >> hil_session_start_slam('stereo', 0.36)              % stereo, baseline 0.36 m
%   >> hil_session_start_slam('rgbd')                      % RGB-D
%   >> hil_session_start_slam(mode, baseline, left_image, right_image)   % image type: 'rgb' | 'mono'
%
% The model is chosen by the naming rule of config/hil/models.yaml (file stem):
%     mono   hil_closed_loop_unified
%     rgbd   hil_closed_loop_unified_rgbd
%     stereo hil_closed_loop_unified_stereo_<NNN>   NNN = baseline x 100: 011 036 042 054 061
%   For stereo, set_stereo(1) runs before hil_ros_init_LT; for rgbd, set_rgbd(1) and
%   hil_ros_init_LT_rgbd run, and the RGB-D camera publisher is used. CAM_LEFT_IMAGE and
%   CAM_RIGHT_IMAGE are set. The last step is a waiting loop on the MAIN thread: when the Pi sends
%   probe_init or probe_mission (hil_run_supervisor.m), the loop runs the generated probe copy
%   slam_traj_probe_unified (rgbd: slam_traj_probe_rgbd_unified; made by scripts/gen_probe.py).
%   The prompt stays busy while the loop waits. Stop it with Ctrl-C or stop_hil_supervisor.
%
% Steps (the SETUP order of hil_run_supervisor.m):
%   1. environment: Fast DDS (RMW + a resolved copy of config/hil/fastrtps_matlab.xml),
%      ROS_DOMAIN_ID 0, HIL_MODEL (the model the supervisor controls), build cache in tempdir
%   2. hil_ros_init_LT                 (ROS node, publishers, 20 Hz state timer)
%   3. open the shared model, read back + check its start pose, run it in real time,
%      wait for the first camera frame (max 180 s)
%   4. sim_camera_publisher_timer_LT   (camera publisher; the Pi switches it per run)
%   5. hil_run_supervisor              (listens on TCP 55556; the prompt stays free)
% A failing step stops with an error that names the step. The model file is not saved.
% Every variable starts with hs_: the base-workspace scripts run next to this file's variables.
%
% Approved by Amar Aly, 2026-10-17: model hil_closed_loop_unified (shared 20 m start,
% SIM_PARAMS.md), Fast DDS with the fastrtps_matlab.xml profile.
%
% It is a function so that it can take arguments. The steps that need the base workspace
% (clear all, the init and publisher scripts) run there with evalin.

% ── arguments ─────────────────────────────────────────────────────────────
hs_slam = true;
if nargin < 1 || isempty(mode), mode = 'mono'; end
if nargin < 2, baseline = []; end
if nargin < 3 || isempty(left_image), left_image = 'rgb'; end
if nargin < 4 || isempty(right_image), right_image = 'rgb'; end
hs_mode = lower(char(mode));
hs_baseline = baseline;
hs_left_image = lower(char(left_image));
hs_right_image = lower(char(right_image));
if ~any(strcmp(hs_mode, {'mono', 'stereo', 'rgbd'}))
    error('[session] mode must be mono, stereo or rgbd (got "%s")', hs_mode);
end
if strcmp(hs_mode, 'stereo') && ~any(abs(hs_baseline - [0.11 0.36 0.42 0.54 0.61]) < 1e-9)
    error('[session] stereo needs a baseline of 0.11, 0.36, 0.42, 0.54 or 0.61 m');
end
if ~strcmp(hs_mode, 'stereo') && ~isempty(hs_baseline)
    error('[session] a baseline is only for stereo');
end
if ~any(strcmp(hs_left_image, {'rgb', 'mono'})) || ~any(strcmp(hs_right_image, {'rgb', 'mono'}))
    error('[session] left_image and right_image must be rgb or mono');
end

hs_dir = fileparts(mfilename('fullpath'));
cd(hs_dir);
evalin('base', 'clear all');   % hil_ros_init_LT needs a clean workspace
clc;

% ── settings ──────────────────────────────────────────────────────────────
hs_model        = 'hil_closed_loop_unified';   % shared start: x 15.12, y 3.84, z 7.5, yaw 2*pi-0.0460
if hs_slam
    hs_model = hs_pick_model(hs_mode, hs_baseline, fullfile(hs_dir, '..'));
end
hs_matlab_ip    = '192.168.1.201';             % this Windows PC on the Wi-Fi
hs_pi_ip        = '192.168.1.60';              % the Raspberry Pi on the Wi-Fi
hs_expect_ic    = struct('x_integrator', 15.12, 'y_integrator', 3.84, 'z_integrator', 7.5, ...
                         'yaw_integrator', 2*pi - 0.0460);
hs_frame_wait_s = 180;
% ──────────────────────────────────────────────────────────────────────────

fprintf('[session] 1/5 environment\n');
setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp');
setenv('ROS_DOMAIN_ID', '0');
setenv('HIL_MODEL', hs_model);
if hs_slam
    setenv('CAM_LEFT_IMAGE', hs_left_image);
    setenv('CAM_RIGHT_IMAGE', hs_right_image);
    if strcmp(hs_mode, 'rgbd') && strcmp(hs_left_image, 'mono')
        setenv('RGBD_COLOUR_MONO', '1');       % the RGB-D publisher reads its own variable
    else
        setenv('RGBD_COLOUR_MONO', '');
    end
    fprintf('[session]     SLAM session: mode=%s baseline=%s left_image=%s right_image=%s\n', ...
            hs_mode, mat2str(hs_baseline), hs_left_image, hs_right_image);
end
hs_tmpl = fullfile(hs_dir, '..', 'config', 'hil', 'fastrtps_matlab.xml');
if ~isfile(hs_tmpl)
    error('[session] step 1: Fast DDS template not found: %s', hs_tmpl);
end
hs_xml = fileread(hs_tmpl);
hs_xml = strrep(hs_xml, '${MATLAB_HOST_IP}', hs_matlab_ip);
hs_xml = strrep(hs_xml, '${PI_ADDR}', hs_pi_ip);
if contains(hs_xml, '${')
    error('[session] step 1: the Fast DDS profile still has an unfilled ${...} placeholder');
end
hs_profile = fullfile(tempdir, 'fastrtps_matlab_resolved.xml');
hs_fid = fopen(hs_profile, 'w'); fwrite(hs_fid, hs_xml); fclose(hs_fid);
setenv('FASTRTPS_DEFAULT_PROFILES_FILE', hs_profile);
hs_gen_dir = fullfile(tempdir, 'hil_session_simgen');   % the tracked slprj is from another MATLAB release
Simulink.fileGenControl('set', 'CacheFolder', hs_gen_dir, 'CodeGenFolder', hs_gen_dir, 'createDir', true);
fprintf('[session]     RMW=%s  DOMAIN=%s  HIL_MODEL=%s\n', getenv('RMW_IMPLEMENTATION'), getenv('ROS_DOMAIN_ID'), hs_model);
fprintf('[session]     Fast DDS profile %s  (MATLAB %s, Pi %s)\n', hs_profile, hs_matlab_ip, hs_pi_ip);

fprintf('[session] 2/5 hil_ros_init_LT\n');
if hs_slam && strcmp(hs_mode, 'stereo')
    set_stereo(1);                        % before hil_ros_init_LT: creates the right-eye publisher
    evalin('base', 'hil_ros_init_LT');
elseif hs_slam && strcmp(hs_mode, 'rgbd')
    set_rgbd(1);                          % sets RGBD_ON in the base workspace
    evalin('base', 'hil_ros_init_LT_rgbd');
else
    evalin('base', 'hil_ros_init_LT');
end

fprintf('[session] 3/5 open + check + run %s\n', hs_model);
if ~isfile(fullfile(hs_dir, [hs_model '.slx']))
    error('[session] step 3: %s.slx not found in %s', hs_model, hs_dir);
end
open_system(fullfile(hs_dir, [hs_model '.slx']));
for hs_blk = fieldnames(hs_expect_ic)'
    hs_v = get_param([hs_model '/' hs_blk{1}], 'InitialCondition');
    fprintf('[session]     %-15s InitialCondition = %s\n', hs_blk{1}, hs_v);
    if abs(evalin('base', hs_v) - hs_expect_ic.(hs_blk{1})) > 1e-6
        error('[session] step 3: %s starts at %s, expected %.4f (shared start, SIM_PARAMS.md)', ...
              hs_blk{1}, hs_v, hs_expect_ic.(hs_blk{1}));
    end
end
set_param(hs_model, 'EnablePacing', 'on', 'PacingRate', '1');   % real time; the file is not saved
set_param(hs_model, 'SimulationCommand', 'start');
hs_t_frame = tic;
hs_have_frame = false;
while toc(hs_t_frame) < hs_frame_wait_s
    pause(0.5);
    if evalin('base', "exist('latest_frame','var')") && ~isempty(evalin('base', 'latest_frame'))
        hs_have_frame = true;
        break;
    end
end
if ~hs_have_frame
    error('[session] step 3: no camera frame after %d s — check the 3D window', hs_frame_wait_s);
end
fprintf('[session]     first frame after %.1f s\n', toc(hs_t_frame));

if hs_slam && strcmp(hs_mode, 'rgbd')
    fprintf('[session] 4/5 sim_camera_publisher_timer_LT_rgbd\n');
    evalin('base', 'sim_camera_publisher_timer_LT_rgbd');
else
    fprintf('[session] 4/5 sim_camera_publisher_timer_LT\n');
    evalin('base', 'sim_camera_publisher_timer_LT');
end

fprintf('[session] 5/5 hil_run_supervisor (model %s)\n', getenv('HIL_MODEL'));
evalin('base', 'hil_run_supervisor');

if ~hs_slam
    fprintf('[session] READY — start the run on the Pi:  benchmarks/ctrl_runner proportional\n');
    return;
end

% SLAM: wait on the main thread for probe requests from the Pi.
hs_probe = 'slam_traj_probe_unified';
if strcmp(hs_mode, 'rgbd'), hs_probe = 'slam_traj_probe_rgbd_unified'; end
if ~isfile(fullfile(hs_dir, [hs_probe '.m']))
    error('[session] %s.m not found. On the Pi run: python3 scripts/gen_probe.py, then copy matlab/%s.m here.', hs_probe, hs_probe);
end
fprintf('[session] READY — start the run on the Pi:  ./run_slam_benchmark.sh --tag <name>\n');
fprintf('[session] waiting for probe requests (probe: %s). Stop: Ctrl-C or stop_hil_supervisor\n', hs_probe);
hs_probe_loop(hs_probe);
end


function hs_probe_loop(hs_probe)
% The waiting loop. It runs the probe script ON THE MAIN THREAD, in this function's workspace,
% so the script finds its *_OVERRIDE variables here. The supervisor timer keeps firing during
% pause. hilprobe_req is set by the supervisor command probe_init / probe_mission / probe_scan / probe_straight.
    assignin('base', 'hilprobe_loop', true);
    assignin('base', 'hilprobe_state', 'idle');
    hs_cleanup = onCleanup(@() hs_loop_off()); %#ok<NASGU>
    while true
        pause(0.1);
        if isempty(timerfindall('Name', 'hil_supervisor_timer'))
            fprintf('[session] supervisor stopped: leaving the waiting loop\n');
            break;
        end
        hs_req = '';
        try
            if evalin('base', "exist('hilprobe_req','var')")
                hs_req = evalin('base', 'hilprobe_req');
            end
        catch
        end
        if isempty(hs_req), continue; end
        hs_args = evalin('base', 'hilprobe_args');
        evalin('base', 'clear hilprobe_req');
        % The five hooks the probe reads (slam_traj_probe.m lines 113 and 233-236).
        SIX_VX_OVERRIDE      = hs_args(1); %#ok<NASGU>
        SIX_CYCLES_OVERRIDE  = hs_args(2); %#ok<NASGU>
        SIX_LAMBDA_OVERRIDE  = hs_args(3); %#ok<NASGU>
        YAW_AMP_OVERRIDE     = hs_args(4); %#ok<NASGU>
        PROBE_PHASE_OVERRIDE = hs_req;     %#ok<NASGU>
        hs_run = hs_probe;
        if numel(hs_args) >= 6 && (hs_args(5) ~= 0 || hs_args(6) ~= 0 || numel(hs_args) >= 7 || strcmp(hs_req, 'turn'))
            % Wider view (2026-10-18): camera turned by hs_args(5) deg, init leg along the path if hs_args(6) = 1.
            YAW_OFFSET_OVERRIDE = hs_args(5); %#ok<NASGU>
            INIT_AXIS_OVERRIDE  = hs_args(6); %#ok<NASGU>
            if numel(hs_args) >= 7, YAW_RAMP_OVERRIDE = hs_args(7); end %#ok<NASGU>  % end-facing: 1 = ramp the camera to 0
            hs_run = 'slam_traj_probe_wide';
        elseif strcmp(hs_req, 'turn')
            error('probe_turn needs 6 numbers (... <cam_yaw_offset_deg> <init_along_path>)');
        end
        fprintf('[session] probe %s: vx=%g cycles=%g lambda=%g yaw_amp=%g\n', hs_req, hs_args(1:4));
        try
            if strcmp(hs_req, 'straight')
                % System-like approach (diagnostic): probe_straight <vmax_mps> <with_init> <standoff_m> <unused>
                SA_VMAX_OVERRIDE     = hs_args(1); %#ok<NASGU>
                SA_INIT_OVERRIDE     = hs_args(2); %#ok<NASGU>
                SA_STANDOFF_OVERRIDE = hs_args(3); %#ok<NASGU>
                eval('slam_straight_approach');
            elseif strcmp(hs_req, 'scan')
                % Look-around scan (diagnostic): probe_scan <step_deg> <settle_s> <speed_mps> <unused>
                SCAN_STEP_OVERRIDE   = hs_args(1); %#ok<NASGU>
                SCAN_SETTLE_OVERRIDE = hs_args(2); %#ok<NASGU>
                SCAN_SPEED_OVERRIDE  = hs_args(3); %#ok<NASGU>
                eval('slam_look_around');      % sets init_result and scan_dir in the base workspace
            else
                fprintf('[session] running %s\n', hs_run);
                eval(hs_run);                 % the probe script; it sets init_result in the base workspace
            end
        catch hs_err
            fprintf(2, '[session] probe %s failed: %s\n', hs_req, hs_err.message);
            assignin('base', 'init_result', struct('invalid', true, 'phase', hs_req, ...
                'end_pos', [NaN NaN NaN], 'dist_to_target_m', NaN, 'maxtime_hit', 0));
        end
        assignin('base', 'hilprobe_state', 'done');
        fprintf('[session] probe %s done\n', hs_req);
    end
end


function hs_loop_off()
    try, assignin('base', 'hilprobe_loop', false); catch, end
end


function name = hs_pick_model(mode, baseline, repo_dir)
% Model name (file stem) for a mode, by config/hil/models.yaml. If the file cannot be read or
% the entry is not found, the naming rule is used (same names as in models.yaml).
    name = '';
    f = fullfile(repo_dir, 'config', 'hil', 'models.yaml');
    try
        txt = fileread(f);
        if strcmp(mode, 'stereo')
            pat = ['"?' regexptranslate('escape', sprintf('%.2f', baseline)) '"?\s*:\s*\{[^}\n]*file:\s*(\S+?)[,}\s]'];
        else
            pat = ['(?m)^\s*' mode '\s*:\s*\{[^}\n]*file:\s*(\S+?)[,}\s]'];
        end
        tok = regexp(txt, pat, 'tokens', 'once');
        if ~isempty(tok)
            [~, name] = fileparts(strtrim(tok{1}));
        end
    catch
    end
    if isempty(name)
        switch mode
            case 'mono',   name = 'hil_closed_loop_unified';
            case 'rgbd',   name = 'hil_closed_loop_unified_rgbd';
            otherwise,     name = sprintf('hil_closed_loop_unified_stereo_%03d', round(baseline * 100));
        end
        fprintf('[session]     model by the naming rule: %s\n', name);
    else
        fprintf('[session]     model from %s: %s\n', f, name);
    end
end
