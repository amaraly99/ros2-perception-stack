function check_heldstart_25W(label, partB, teardown, secsA)
% check_heldstart_25W.m  (prompt 25W test harness, not part of the wrapper)
% Run right after hil_session_start_heldstart_mode returned held. Never sends "resume".
%   label     tag of the result file test_<label>.txt next to this file
%   secsA     length of part A in s (default 30); above 120 s it also reports the stretch after 120 s
%   partB     true: a second ROS node counts what arrives on each image topic, first in the
%             current hold, then through the Pi's per-run sequence ("stop", "pub_stop",
%             "pub_start 20", "start_hold") into a second hold
%   teardown  true: end with the Pi's end-of-run "stop" + "pub_stop", stop the supervisor and
%             the state timer, close the model unsaved
if nargin < 2, partB = true; end
if nargin < 3, teardown = true; end
if nargin < 4, secsA = 30; end
here = fileparts(mfilename('fullpath'));
fid = fopen(fullfile(here, ['test_' label '.txt']), 'w');
closer = onCleanup(@() fclose(fid));
model = getenv('HIL_MODEL');
START = [15.12 3.84 7.5];  YAW0 = 2*pi - 0.0460;  SIGN = [35.1 2.92];
T = tic;
TOPICS = {'/sim/camera/image_raw', '/sim/camera/right/image_raw', '/sim/camera/depth/image_raw'};
cnt = zeros(1, 3);  meta = {'', '', ''};

lg(fid, 'label %s  model %s  %s', label, model, char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')));
lg(fid, 'pwd %s', pwd);
lg(fid, 'env RMW=%s DOMAIN=%s FASTRTPS_DEFAULT_PROFILES_FILE=[%s] CAM_LEFT_IMAGE=%s CAM_RIGHT_IMAGE=%s RGBD_DROP_DUP_STAMPS=[%s]', ...
    getenv('RMW_IMPLEMENTATION'), getenv('ROS_DOMAIN_ID'), getenv('FASTRTPS_DEFAULT_PROFILES_FILE'), ...
    getenv('CAM_LEFT_IMAGE'), getenv('CAM_RIGHT_IMAGE'), getenv('RGBD_DROP_DUP_STAMPS'));
for f = {'hil_ros_init_LT', 'hil_ros_init_LT_rgbd', 'hil_run_supervisor', 'hil_pub_control', ...
         'hil_publish_frame', 'read_cmdvel_live_interp', 'sim_camera_publisher_timer_LT', ...
         'sim_camera_publisher_timer_LT_rgbd', 'set_stereo', 'set_rgbd', model}
    w = which('-all', f{1});
    lg(fid, 'which %s: %s', f{1}, strjoin(w(:)', ' | '));
end

% ── model as loaded ────────────────────────────────────────────────────────
for cam = {'Simulation 3D Camera', 'Simulation 3D Camera Right'}
    b = [model '/' cam{1}];
    lg(fid, '%s: Commented=%s FocalLength=%s ImageSize=%s OpticalCenter=%s tmountOffset=%s rmountOffset=%s DepthOutportEnabled=%s SampleTime=%s', ...
        cam{1}, gp(b, 'Commented'), gp(b, 'FocalLength'), gp(b, 'ImageSize'), gp(b, 'OpticalCenter'), ...
        gp(b, 'tmountOffset'), gp(b, 'rmountOffset'), gp(b, 'DepthOutportEnabled'), gp(b, 'SampleTime'));
end
for blk = {'write_depth_left', 'write_frame_right', 'write_depth_right'}
    lg(fid, '%s: Commented=%s', blk{1}, gp([model '/' blk{1}], 'Commented'));
end
for ic = {'x_integrator', 'y_integrator', 'z_integrator', 'yaw_integrator'}
    lg(fid, '%s InitialCondition=%s', ic{1}, gp([model '/' ic{1}], 'InitialCondition'));
end
lg(fid, 'SimulationMode=%s FixedStep=%s StopTime=%s EnablePacing=%s PacingRate=%s', gp(model, 'SimulationMode'), ...
    gp(model, 'FixedStep'), gp(model, 'StopTime'), gp(model, 'EnablePacing'), gp(model, 'PacingRate'));
lg(fid, 'model_info: %s', sendcmd('model_info'));
lg(fid, 'status: %s', sendcmd('status'));
lg(fid, 'probe_init 0.5 1 1 0: %s', sendcmd('probe_init 0.5 1 1 0'));   % must be refused: no probe loop
lg(fid, 'STEREO_ON=%s RGBD_ON=%s cam_pub_right=%d depth_pub=%d', mat2str(bv('STEREO_ON', [])), ...
    mat2str(bv('RGBD_ON', [])), ~isempty(bv('cam_pub_right', [])), ~isempty(bv('depth_pub', [])));
tm = timerfindall;
for k = 1:numel(tm)
    lg(fid, 'timer %s Running=%s Period=%.4f', tm(k).Name, tm(k).Running, tm(k).Period);
end
frames_info(fid, here, [label '_hold1']);

% ── part A: 30 s held, publisher counters only (nothing else subscribes) ────
lg(fid, '--- part A: %d s held, counters only', secsA);
mem0 = memory;
sA = snap(model, T);  lg(fid, '%s', fmt(sA));
for k = 1:secsA
    pause(1);
    sA(end + 1) = snap(model, T); %#ok<AGROW>
    lg(fid, '%s', fmt(sA(end)));
end
mem1 = memory;
summary(fid, 'A', sA, START, YAW0, SIGN);
if secsA > 120
    summary(fid, 'A after 120 s', sA(121:end), START, YAW0, SIGN);
end
lg(fid, 'A: MATLAB memory used %.0f MB at start, %.0f MB at end (%+.0f MB)', mem0.MemUsedMATLAB / 2^20, ...
    mem1.MemUsedMATLAB / 2^20, (mem1.MemUsedMATLAB - mem0.MemUsedMATLAB) / 2^20);
lg(fid, 'ros2 topic list:\n%s', evalc('ros2 topic list'));

% ── part B: what a second node receives ─────────────────────────────────────
if partB
    lg(fid, '--- part B1: node /check_25W subscribes (best effort) to the three image topics');
    node2 = ros2node('/check_25W');
    subs = cell(1, 3);
    for i = 1:3
        subs{i} = ros2subscriber(node2, TOPICS{i}, 'sensor_msgs/Image', @(m) onmsg(i, m), ...
            'Reliability', 'besteffort', 'Durability', 'volatile', 'Depth', 5);
    end
    pause(3);
    cnt(:) = 0;
    sB = snap(model, T);
    pause(15);
    sB(2) = snap(model, T);
    lg(fid, 'B1 15 s in the same hold: received %s, sent L+%d R+%d D+%d', mat2str(cnt), ...
        sB(2).L - sB(1).L, sB(2).R - sB(1).R, sB(2).D - sB(1).D);
    for i = 1:3, lg(fid, '  %s: %d msgs, last %s', TOPICS{i}, cnt(i), meta{i}); end

    lg(fid, '--- part B2: the Pi''s per-run sequence into a second hold');
    lg(fid, 'stop: %s', sendcmd('stop'));
    wait_status(model, 'stopped', 60);
    lg(fid, 'pub_stop: %s', sendcmd('pub_stop'));
    pause(1);
    cnt(:) = 0;
    sC = snap(model, T);
    lg(fid, 'pub_start 20: %s', sendcmd('pub_start 20'));
    lg(fid, 'start_hold: %s', sendcmd('start_hold'));
    t0 = tic;  st = '';
    while toc(t0) < 180 && ~any(strcmp(st, {'held', 'frozen'}))
        pause(0.5);
        st = bv('hil_supervisor_state', '?');
    end
    lg(fid, 'second hold: state %s after %.1f s, received so far %s', st, toc(t0), mat2str(cnt));
    sC(2) = snap(model, T);
    for k = 1:25
        pause(1);
        sC(end + 1) = snap(model, T); %#ok<AGROW>
    end
    lg(fid, '%s', fmt(sC(2)));  lg(fid, '%s', fmt(sC(end)));
    summary(fid, 'B2 (25 s after the second hold)', sC(2:end), START, YAW0, SIGN);
    lg(fid, 'B2 from pub_start to end: received %s, sent L+%d R+%d D+%d', mat2str(cnt), ...
        sC(end).L - sC(1).L, sC(end).R - sC(1).R, sC(end).D - sC(1).D);
    for i = 1:3, lg(fid, '  %s: %d msgs, last %s', TOPICS{i}, cnt(i), meta{i}); end
    frames_info(fid, here, [label '_hold2']);
    clear subs node2
end

% ── teardown, as the Pi ends a run ──────────────────────────────────────────
if teardown
    lg(fid, '--- teardown');
    lg(fid, 'stop: %s', sendcmd('stop'));
    lg(fid, 'pub_stop: %s', sendcmd('pub_stop'));
    wait_status(model, 'stopped', 60);
    stop_hil_supervisor();
    for n = {'pitch_pub_timer_LT', 'sim_cam_pub_timer_LT'}
        t = timerfindall('Name', n{1});
        if ~isempty(t), stop(t); delete(t); end
    end
    close_system(model, 0);
    free = false;
    try, s = java.net.ServerSocket(55556); s.close(); free = true; catch, end
    lg(fid, 'TEARDOWN: model loaded=%d, timers left=%d, port 55556 free=%d', bdIsLoaded(model), numel(timerfindall), free);
end
lg(fid, 'DONE %s after %.0f s', label, toc(T));

    function onmsg(i, m)
        cnt(i) = cnt(i) + 1;
        meta{i} = sprintf('%dx%d %s step %d frame_id "%s" stamp %d.%09d', m.width, m.height, m.encoding, ...
                          m.step, m.header.frame_id, m.header.stamp.sec, m.header.stamp.nanosec);
    end
end


function lg(fid, varargin)
    s = sprintf(varargin{:});
    fprintf(fid, '%s\n', s);
    fprintf('[25W] %s\n', s);
end

function v = gp(obj, p)
    try, v = get_param(obj, p); if ~ischar(v), v = mat2str(v); end
    catch e, v = ['n/a (' e.message ')']; end
end

function v = bv(name, default)
    v = default;
    try
        if evalin('base', sprintf("exist('%s','var')", name)), v = evalin('base', name); end
    catch
    end
end

function s = snap(model, T)
    s.t = toc(T);
    s.status = get_param(model, 'SimulationStatus');
    s.simt = get_param(model, 'SimulationTime');
    s.state = bv('hil_supervisor_state', '?');
    p = double(bv('sim_pose', nan(5, 1)));
    s.pose = p(:)';
    s.L = bv('sim_cam_pub_count_LT', 0) + bv('sim_cam_repeat_count_LT', 0);
    s.R = bv('sim_cam_pub_count_right_LT', 0);
    s.D = bv('sim_cam_pub_count_depth_LT', 0);
    s.drop = bv('sim_cam_drop_count_LT', 0);
end

function s = fmt(x)
    s = sprintf('t=%6.1f %-7s simt=%.2f state=%s x=%.3f y=%.3f z=%.3f yaw=%.2fdeg L=%d R=%d D=%d drop=%d', ...
                x.t, x.status, x.simt, x.state, x.pose(1), x.pose(2), x.pose(3), rad2deg(x.pose(5)), ...
                x.L, x.R, x.D, x.drop);
end

function summary(fid, tag, s, START, YAW0, SIGN)
    P = vertcat(s.pose);
    dt = s(end).t - s(1).t;
    dxyz = max(abs(P(:, 1:3) - START), [], 1);
    dyaw = max(abs(P(:, 5) - YAW0));
    d = hypot(SIGN(1) - P(:, 1), SIGN(2) - P(:, 2));
    simt = [s.simt];
    ok = all(strcmp({s.status}, 'paused')) && all(strcmp({s.state}, 'held')) && ...
         max(simt) - min(simt) < 1e-9 && all(dxyz < 0.05) && dyaw < 0.01;
    lg(fid, ['%s: %.1f s, status {%s}, state {%s}, simt %.2f..%.2f, max |dx dy dz| %s m, max |dyaw| %.4f rad, ' ...
             'sign distance %.3f..%.3f m, z %.3f'], tag, dt, strjoin(unique({s.status}), ','), ...
        strjoin(unique({s.state}), ','), min(simt), max(simt), mat2str(dxyz, 3), dyaw, min(d), max(d), P(end, 3));
    lg(fid, '%s: send rate L %.2f Hz, R %.2f Hz, D %.2f Hz (sends %d %d %d), drops +%d; HELD_AT_START=%d', tag, ...
        (s(end).L - s(1).L) / dt, (s(end).R - s(1).R) / dt, (s(end).D - s(1).D) / dt, ...
        s(end).L - s(1).L, s(end).R - s(1).R, s(end).D - s(1).D, s(end).drop - s(1).drop, ok);
end

function wait_status(model, want, tmax)
    t0 = tic;
    while ~strcmp(get_param(model, 'SimulationStatus'), want) && toc(t0) < tmax
        pause(0.5);
    end
end

function frames_info(fid, here, tag)
    f = bv('latest_frame', []);
    if ~isempty(f)
        lg(fid, '%s latest_frame %s %s sha256 %s', tag, class(f), mat2str(size(f)), sha(f));
        imwrite(f, fullfile(here, ['frame_' tag '_left.png']));
    end
    r = bv('latest_frame_right', []);
    if ~isempty(r)
        lg(fid, '%s latest_frame_right %s %s sha256 %s identical_to_left %d mean_abs_diff %.3f', tag, class(r), ...
            mat2str(size(r)), sha(r), isequal(r, f), mean(abs(double(r(:)) - double(f(:)))));
        imwrite(r, fullfile(here, ['frame_' tag '_right.png']));
    end
    d = bv('latest_depth_left', []);
    if ~isempty(d)
        v = double(d(:));  ok = isfinite(v) & v > 0 & v <= 65.535;
        lg(fid, '%s latest_depth_left %s %s valid %.1f%% min %.2f median %.2f max %.2f m, centre %.2f m, sky(1000) %.1f%%', ...
            tag, class(d), mat2str(size(d)), 100 * mean(ok), min(v(ok)), median(v(ok)), max(v(ok)), ...
            double(d(240, 320)), 100 * mean(v == 1000));
    end
end

function h = sha(b)
    md = java.security.MessageDigest.getInstance('SHA-256');
    md.update(typecast(b(:), 'uint8'));
    h = lower(reshape(dec2hex(typecast(md.digest(), 'uint8'), 2)', 1, []));
end

function reply = sendcmd(cmd)
% One supervisor command over TCP, as the Pi sends it; the reply is returned, not checked.
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
end
