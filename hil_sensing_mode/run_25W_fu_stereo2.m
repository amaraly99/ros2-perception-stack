% run_25W_fu_stereo2.m  (25W follow-up re-test, launched with matlab -r): 3 minutes held, then teardown and exit.
try
    hil_session_start_heldstart_mode('stereo');
    check_heldstart_25W('fu_stereo2', true, true, 180);
    fprintf('[25W] SESSION END OK\n');
    exit
catch err_25W
    fprintf(2, '[25W] FAILED: %s\n', getReport(err_25W, 'extended', 'hyperlinks', 'off'));
    fprintf('[25W] SESSION END FAILED\n');
end
