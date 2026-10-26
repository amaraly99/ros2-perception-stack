% run_25W_rgbd_then_mono.m  (prompt 25W test session, launched with matlab -r)
% RGB-D test with teardown, then a switch to mono in the same MATLAB, left running and held.
try
    hil_session_start_heldstart_mode('rgbd');
    check_heldstart_25W('rgbd', true, true);
    hil_session_start_heldstart_mode('mono');
    check_heldstart_25W('mono_after_switch', false, false);
    fprintf('[25W] SESSION END OK (mono left running and held)\n');
catch err_25W
    fprintf(2, '[25W] FAILED: %s\n', getReport(err_25W, 'extended', 'hyperlinks', 'off'));
    fprintf('[25W] SESSION END FAILED\n');
end
