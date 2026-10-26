function hil_session_start_heldstart_mode(mode)
% hil_session_start_heldstart_mode.m  (prompt 25W, 2026-10-25)
% One sensing-mode session at our 20 m held start. Models and scripts load from .\matlab next to
% this file: copies of matlab\ at 28114bd (Amar's sim/unified-start-2026-10-04), byte-identical except
% that the three camera publishers keep sending a held frame at their timer rate (25W follow-up), and,
% since 26W, only while the supervisor reports the model held (hil_supervisor_state = 'held'); at any
% other time, in flight included, they behave as Amar's files:
%   hil_publish_frame.m                   IDLE_THROTTLE_SEC = Inf while held (else Amar's 120)
%   sim_camera_publisher_timer_LT.m       SKIP_REPEATS off while held (else on, as Amar's; stereo pair)
%   sim_camera_publisher_timer_LT_rgbd.m  SKIP_REPEATS off while held (else on, as Amar's; colour + depth)
% Edits nothing in the worktree or in HIL.
%
%   >> cd C:\Users\Ahmed\sim_25W
%   >> hil_session_start_heldstart_mode('mono')      % or 'stereo' (0.61 m) or 'rgbd'
%
%   mode    model (config/hil/models.yaml)    cameras
%   mono    hil_closed_loop_unified           left only; right eye commented out as stored
%   stereo  hil_closed_loop_baseline_061      left + right at y = -0.61 m (set_stereo(1))
%   rgbd    hil_closed_loop_baseline_RGBD     left colour + left depth (set_rgbd(1))
%
% Our session start (hil_ros_init_LT; hil_run_supervisor), then what the Pi sends at the start of a
% run, "pub_start 20" and "start_hold", sent to the supervisor on 127.0.0.1:55556. Returns when the
% supervisor reports held: the model paused at sim 0.25 s on its ICs (x 15.12, y 3.84, z 7.5,
% 20.0 m from the frozen sign). Release stays with the Pi ("resume"). There is no probe loop, so
% probe_* commands answer "error no_loop" and the scripted trajectory cannot start.
% Calling it again switches mode in this MATLAB: the session before is shut down first.

MODELS = struct('mono',   'hil_closed_loop_unified', ...
                'stereo', 'hil_closed_loop_baseline_061', ...
                'rgbd',   'hil_closed_loop_baseline_RGBD');
mode = lower(char(mode));
if ~isfield(MODELS, mode)
    error('[heldstart] mode must be mono, stereo or rgbd (got "%s")', mode);
end
model = MODELS.(mode);
here = fileparts(mfilename('fullpath'));
src = fullfile(here, 'matlab');
addpath(here);                                % callable again after cd(src)
fprintf('[heldstart] mode %s, model %s, from %s\n', mode, model, src);
hm_shutdown();                                % the session before, if any

% Environment of our session start. HIL's hil_ros_init_LT.m:19 hard-sets Fast DDS; the worktree
% copy keeps whatever is set, so it is set here. HIL_MODEL names the model for the supervisor and set_*.
setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp');
setenv('ROS_DOMAIN_ID', '0');
setenv('HIL_MODEL', model);
setenv('CAM_LEFT_IMAGE', 'rgb');  setenv('CAM_RIGHT_IMAGE', 'rgb');  setenv('RGBD_COLOUR_MONO', '');
setenv('RGBD_DROP_DUP_STAMPS', '');           % held RGB-D pairs share one stamp; never drop them
gen = fullfile(here, 'simgen');               % build cache, kept out of .\matlab
Simulink.fileGenControl('set', 'CacheFolder', gen, 'CodeGenFolder', gen, 'createDir', true);
cd(src);
evalin('base', 'clear all');                  % hil_ros_init_LT needs a clean workspace

switch mode
    case 'stereo', set_stereo(1);             % right eye on, STEREO_ON = 1
    case 'rgbd',   set_rgbd(1);               % checks the depth wiring, RGBD_ON = 1
end
if strcmp(mode, 'rgbd')
    evalin('base', 'hil_ros_init_LT_rgbd');
else
    evalin('base', 'hil_ros_init_LT');
end
evalin('base', 'hil_run_supervisor');
right_on = strcmp(get_param([model '/Simulation 3D Camera Right'], 'Commented'), 'off');
if right_on ~= strcmp(mode, 'stereo')
    error('[heldstart] right camera active = %d in %s mode', right_on, mode);
end

% The held start. Mono keeps the Pi's order: hil_publish_frame skips the image latest_frame holds at
% pub_start, so pub_start comes first. Stereo and RGB-D: pub_start runs Amar's publisher scripts,
% which stop with an error until latest_frame (and the right or depth frame) exists, so it comes once held.
if strcmp(mode, 'mono'), hm_send('pub_start 20'); end
st = hm_hold();
if strcmp(st, 'frozen')
    fprintf(2, '[heldstart] first hold frozen (first build?), sending start_hold once more\n');
    st = hm_hold();
end
if ~strcmp(st, 'held')
    error('[heldstart] not held (supervisor state "%s")', st);
end
if ~strcmp(mode, 'mono'), hm_send('pub_start 20'); end
p = evalin('base', 'sim_pose');
fprintf('[heldstart] %s\n', hm_send('status'));
fprintf('[heldstart] pose x %.2f y %.2f z %.2f yaw %.2f deg, %.2f m from the frozen sign (35.1, 2.92)\n', ...
        p(1), p(2), p(3), rad2deg(p(5)), hypot(35.1 - p(1), 2.92 - p(2)));
fprintf('[heldstart] READY: %s held at the 20 m start. The Pi releases it with "resume".\n', mode);
end


function st = hm_hold()
% "start_hold", then wait for the supervisor to report held or frozen.
    hm_send('start_hold');
    t0 = tic;
    st = '';
    while toc(t0) < 300 && ~any(strcmp(st, {'held', 'frozen'}))
        pause(0.5);
        if evalin('base', "exist('hil_supervisor_state','var')")
            st = evalin('base', 'hil_supervisor_state');
        end
    end
    fprintf('[heldstart] start_hold: %s after %.1f s\n', st, toc(t0));
end


function reply = hm_send(cmd)
% One supervisor command over TCP, as the Pi sends it. pause() lets the poll timer answer.
    s = java.net.Socket('127.0.0.1', 55556);
    c = onCleanup(@() s.close());
    out = java.io.PrintWriter(s.getOutputStream(), true);
    in = java.io.BufferedReader(java.io.InputStreamReader(s.getInputStream()));
    out.println(cmd);
    reply = '';
    t = tic;
    while isempty(reply) && toc(t) < 30
        pause(0.1);
        if in.ready(), reply = char(in.readLine()); end
    end
    if isempty(reply) || strncmp(reply, 'error', 5)
        error('[heldstart] "%s" got reply "%s"', cmd, reply);
    end
end


function hm_shutdown()
% Stops what a session before left in this MATLAB: supervisor socket and timers, publisher and
% state timers, the model (closed unsaved), so the next session starts as in a fresh MATLAB.
    try
        if evalin('base', "exist('hil_supervisor_sock','var')")
            evalin('base', 'hil_supervisor_sock.close();');
        end
    catch
    end
    for n = {'hil_supervisor_timer', 'sim_cam_pub_timer_LT', 'pitch_pub_timer_LT'}
        t = timerfindall('Name', n{1});
        if ~isempty(t), stop(t); delete(t); end
    end
    mdls = find_system('SearchDepth', 0, 'type', 'block_diagram', 'BlockDiagramType', 'model');
    for k = 1:numel(mdls)
        if ~strcmp(get_param(mdls{k}, 'SimulationStatus'), 'stopped')
            set_param(mdls{k}, 'SimulationCommand', 'stop');
            t0 = tic;
            while ~strcmp(get_param(mdls{k}, 'SimulationStatus'), 'stopped') && toc(t0) < 60
                pause(0.5);
            end
        end
        close_system(mdls{k}, 0);
        fprintf('[heldstart] closed %s (not saved)\n', mdls{k});
    end
end
