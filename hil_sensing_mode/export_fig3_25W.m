% export_fig3_25W.m  (prompt 25W step 5, run with matlab -batch)
% Prints the top level of Amar's extra-camera models from the worktree. Nothing is saved.
% fig3_model.png:      hil_closed_loop_baseline_061 with the right-eye blocks active, as set_stereo(1)
%                      leaves them in a stereo session (in memory only)
% fig3_model_rgbd.png: hil_closed_loop_baseline_RGBD as stored (left depth port on, write_depth_left)
wt  = 'C:\Users\Ahmed\sim_25A\matlab';
out = 'C:\Users\Ahmed\sim_25W';
Simulink.fileGenControl('set', 'CacheFolder', fullfile(out, 'simgen'), 'CodeGenFolder', fullfile(out, 'simgen'), 'createDir', true);
cd(wt);

m = 'hil_closed_loop_baseline_061';
load_system(m);
for b = {'Simulation 3D Camera Right', 'write_frame_right', 'write_depth_right'}
    set_param([m '/' b{1}], 'Commented', 'off');
end
print(['-s' m], '-dpng', '-r300', fullfile(out, 'fig3_model.png'));
fprintf('[fig3] %s printed, dirty=%s\n', m, get_param(m, 'Dirty'));
close_system(m, 0);

m = 'hil_closed_loop_baseline_RGBD';
load_system(m);
print(['-s' m], '-dpng', '-r300', fullfile(out, 'fig3_model_rgbd.png'));
fprintf('[fig3] %s printed, dirty=%s\n', m, get_param(m, 'Dirty'));
close_system(m, 0);
