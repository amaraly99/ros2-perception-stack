% run_25W_ab.m  (25W follow-up A/B, launched with matlab -r): mono 60 s held with the unchanged 28114bd
% publisher (ab_control\), then mono 60 s held with the changed copy (.\matlab), back to back in one MATLAB.
try
    cd('C:\Users\Ahmed\sim_25W\ab_control');
    hil_session_start_heldstart_mode('mono');
    check_heldstart_25W('ab_original_mono', false, true, 60);
    cd('C:\Users\Ahmed\sim_25W');
    rmpath('C:\Users\Ahmed\sim_25W\ab_control');
    clear functions
    hil_session_start_heldstart_mode('mono');
    check_heldstart_25W('ab_changed_mono', false, true, 60);
    fprintf('[25W] SESSION END OK\n');
    exit
catch err_25W
    fprintf(2, '[25W] FAILED: %s\n', getReport(err_25W, 'extended', 'hyperlinks', 'off'));
    fprintf('[25W] SESSION END FAILED\n');
end
