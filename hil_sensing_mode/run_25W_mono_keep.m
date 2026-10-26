% run_25W_mono_keep.m  (prompt 25W step 6, launched with matlab -r): mono in a fresh MATLAB, left running and held.
try
    hil_session_start_heldstart_mode('mono');
    check_heldstart_25W('mono_final', false, false);
    fprintf('[25W] SESSION END OK (mono left running and held)\n');
catch err_25W
    fprintf(2, '[25W] FAILED: %s\n', getReport(err_25W, 'extended', 'hyperlinks', 'off'));
    fprintf('[25W] SESSION END FAILED\n');
end
