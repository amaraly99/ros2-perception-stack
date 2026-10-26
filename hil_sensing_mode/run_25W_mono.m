% run_25W_mono.m  (prompt 25W test session, launched with matlab -r). Exits MATLAB only on success.
try
    hil_session_start_heldstart_mode('mono');
    check_heldstart_25W('mono', true, true);
    fprintf('[25W] SESSION END OK\n');
    exit
catch err_25W
    fprintf(2, '[25W] FAILED: %s\n', getReport(err_25W, 'extended', 'hyperlinks', 'off'));
    fprintf('[25W] SESSION END FAILED\n');
end
