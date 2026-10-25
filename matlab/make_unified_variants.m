function make_unified_variants(varargin)
%MAKE_UNIFIED_VARIANTS  Build the stereo and RGB-D models of the SLAM benchmark from the mono one.
%
%   >> cd <repo>\matlab
%   >> make_unified_variants                % all five stereo models and the RGB-D model
%   >> make_unified_variants('stereo', 0.36) % one stereo model
%   >> make_unified_variants('rgbd')         % the RGB-D model only
%
% Source: hil_closed_loop_unified.slx (shared 20 m start, SIM_PARAMS.md). It is opened, never saved.
% Results (names as in config/hil/models.yaml), saved next to the source:
%   hil_closed_loop_unified_stereo_011 / _036 / _042 / _054 / _061   baseline 0.11 ... 0.61 m
%   hil_closed_loop_unified_rgbd
%
% STEREO, per baseline B:
%   * 'Simulation 3D Camera Right', 'write_frame_right', 'write_depth_right': Commented = off
%     (the same three blocks that set_stereo(1) switches on)
%   * right camera mount y = -B in BOTH parameters 'tmountOffset' [0.2, -B, 0.2]
%     and 'mountPoint' [0.2 -B 0.2] (as in hil_closed_loop_baseline_036.slx)
% RGB-D:
%   * left camera 'DepthOutportEnabled' = on
%   * block 'write_depth_left' copied from hil_closed_loop_baseline_RGBD.slx, wired from output 2
%     of 'Simulation 3D Camera' (as in that file)
%   * the right camera stays commented out
%
% It prints every value it sets and reads each one back. NOT RUN IN MATLAB by the author:
% check the printed lines, then open one model and run a short simulation before a session.
% The window of the source model is left untouched; the new models are closed at the end.

    SRC = 'hil_closed_loop_unified';
    RGBD_BASELINE = 'hil_closed_loop_baseline_RGBD';
    BASELINES = [0.11 0.36 0.42 0.54 0.61];
    CAM_L = 'Simulation 3D Camera';
    CAM_R = 'Simulation 3D Camera Right';

    what = 'all';
    if nargin >= 1, what = lower(char(varargin{1})); end
    baselines = BASELINES;
    if strcmp(what, 'stereo') && nargin >= 2
        baselines = varargin{2};
        if ~all(ismember(round(baselines * 100), round(BASELINES * 100)))
            error('make_unified_variants: baseline must be one of 0.11 0.36 0.42 0.54 0.61');
        end
    end
    if ~any(strcmp(what, {'all', 'stereo', 'rgbd'}))
        error('make_unified_variants: use no argument, ''stereo'' [, baseline] or ''rgbd''');
    end

    here = fileparts(mfilename('fullpath'));
    cd(here);
    if ~isfile(fullfile(here, [SRC '.slx']))
        error('make_unified_variants: %s.slx not found in %s', SRC, here);
    end

    if any(strcmp(what, {'all', 'stereo'}))
        for B = baselines
            dst = sprintf('hil_closed_loop_unified_stereo_%03d', round(B * 100));
            fprintf('\n=== %s  (baseline %.2f m) ===\n', dst, B);
            start_copy(here, SRC, dst);
            % 1. the three right-eye blocks on
            blocks = {CAM_R, 'write_frame_right', 'write_depth_right'};
            optional = [false false true];
            for k = 1:numel(blocks)
                blk = [dst '/' blocks{k}];
                try
                    set_param(blk, 'Commented', 'off');
                    fprintf('  %-30s Commented = %s\n', blocks{k}, get_param(blk, 'Commented'));
                catch err
                    if ~optional(k)
                        error('make_unified_variants: block %s not found in %s (%s)', blocks{k}, SRC, err.message);
                    end
                    fprintf('  %-30s not found (optional, skipped)\n', blocks{k});
                end
            end
            % 2. right camera mount y = -B, in both parameters
            camR = [dst '/' CAM_R];
            set_param(camR, 'tmountOffset', sprintf('[0.2, %g, 0.2]', -B));
            fprintf('  tmountOffset = %s\n', get_param(camR, 'tmountOffset'));
            try
                set_param(camR, 'mountPoint', sprintf('[0.2 %g 0.2]', -B));
                fprintf('  mountPoint   = %s\n', get_param(camR, 'mountPoint'));
            catch err
                fprintf('  mountPoint could not be set (%s). Set it by hand to [0.2 %g 0.2] if the dialog shows it.\n', err.message, -B);
            end
            finish_copy(dst);
        end
    end

    if any(strcmp(what, {'all', 'rgbd'}))
        dst = 'hil_closed_loop_unified_rgbd';
        fprintf('\n=== %s ===\n', dst);
        load_system(fullfile(here, [RGBD_BASELINE '.slx']));
        cleanup = onCleanup(@() close_quiet(RGBD_BASELINE)); %#ok<NASGU>
        start_copy(here, SRC, dst);
        camL = [dst '/' CAM_L];
        set_param(camL, 'DepthOutportEnabled', 'on');
        fprintf('  left camera DepthOutportEnabled = %s\n', get_param(camL, 'DepthOutportEnabled'));
        if ~isempty(find_system(dst, 'SearchDepth', 1, 'Name', 'write_depth_left'))
            fprintf('  write_depth_left already present: kept\n');
        else
            add_block([RGBD_BASELINE '/write_depth_left'], [dst '/write_depth_left']);
            fprintf('  write_depth_left copied from %s\n', RGBD_BASELINE);
            % wire it as in the RGB-D baseline: camera output 2 -> write_depth_left input 1
            posL = get_param(camL, 'Position');
            set_param([dst '/write_depth_left'], 'Position', [posL(3) + 60, posL(4) + 20, posL(3) + 260, posL(4) + 80]);
            add_line(dst, [CAM_L '/2'], 'write_depth_left/1', 'autorouting', 'on');
            fprintf('  line: %s/2 -> write_depth_left/1\n', CAM_L);
        end
        camR = [dst '/' CAM_R];
        try
            if ~strcmp(get_param(camR, 'Commented'), 'on')
                set_param(camR, 'Commented', 'on');
                fprintf('  right camera was active: commented out\n');
            else
                fprintf('  right camera Commented = on (as required)\n');
            end
        catch
            fprintf('  no right camera block (fine for RGB-D)\n');
        end
        finish_copy(dst);
    end
    fprintf('\nmake_unified_variants: done. Not run in MATLAB when this file was written: check the lines above.\n');
end


function start_copy(here, src, dst)
% Load the source, save it under the new name (save-as renames the loaded model), keep it loaded.
    close_quiet(dst);
    close_quiet(src);
    load_system(fullfile(here, [src '.slx']));
    save_system(src, fullfile(here, [dst '.slx']));   % now the loaded model is called dst
    fprintf('  created %s.slx from %s.slx\n', dst, src);
end


function finish_copy(dst)
    save_system(dst);
    fprintf('  saved %s.slx\n', dst);
    close_quiet(dst);
end


function close_quiet(name)
    try
        if bdIsLoaded(name), close_system(name, 0); end
    catch
    end
end
