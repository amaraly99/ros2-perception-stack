# keep16.ps1 (prompt 26W): keeps one MATLAB session in the 16 Hz publisher state by keeping its windows minimized.
# Started by hil_session_start_heldstart_mode(mode, 'rate16'); ends when that MATLAB exits or <StopFile> exists.
# Why (26W step 1, VERIFIED on this host): the publisher's 50 ms MATLAB timer fires every 62.5 ms (16 Hz) while
# every MATLAB.exe window of the session is minimized or fully hidden, and every 50 ms (20 Hz) while one is visible.
# Minimizing uses ShowWindow(SW_SHOWMINNOACTIVE) only: it never takes the keyboard focus. No install.
#   powershell -NoProfile -ExecutionPolicy Bypass -WindowStyle Hidden -File keep16.ps1 -MatlabPid <feature('getpid')>
param([Parameter(Mandatory = $true)][int]$MatlabPid, [string]$Log = '', [string]$StopFile = '')
Add-Type -TypeDefinition @'
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
public static class K16 {
    public delegate bool EnumProc(IntPtr h, IntPtr l);
    [DllImport("user32.dll")] public static extern bool EnumWindows(EnumProc f, IntPtr l);
    [DllImport("user32.dll")] public static extern uint GetWindowThreadProcessId(IntPtr h, out uint pid);
    [DllImport("user32.dll")] public static extern bool IsWindowVisible(IntPtr h);
    [DllImport("user32.dll")] public static extern bool IsIconic(IntPtr h);
    [DllImport("user32.dll")] public static extern int GetWindowTextLength(IntPtr h);
    [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr h, int cmd);
    public static List<IntPtr> Shown(HashSet<uint> pids) {
        var r = new List<IntPtr>();
        EnumWindows((h, l) => { uint p; GetWindowThreadProcessId(h, out p);
            if (pids.Contains(p) && IsWindowVisible(h) && !IsIconic(h) && GetWindowTextLength(h) > 0) r.Add(h); return true; }, IntPtr.Zero);
        return r;
    }
}
'@
function Say([string]$t) { $line = '{0} keep16: {1}' -f (Get-Date -Format 'yyyy-MM-dd HH:mm:ss'), $t; if ($Log) { Add-Content -LiteralPath $Log -Value $line } }
# The session's MATLAB.exe processes: the engine, its MATLAB.exe parent and its MATLAB.exe children.
function Session-Pids {
    $all = @(Get-CimInstance Win32_Process -Filter "Name = 'MATLAB.exe'")
    $set = New-Object 'System.Collections.Generic.HashSet[uint32]'
    $me = $all | Where-Object { $_.ProcessId -eq $MatlabPid }
    if (-not $me) { return $set }
    $null = $set.Add([uint32]$MatlabPid)
    $all | Where-Object { $_.ProcessId -eq $me.ParentProcessId -or $_.ParentProcessId -eq $MatlabPid } | ForEach-Object { $null = $set.Add([uint32]$_.ProcessId) }
    return $set
}
Say "start, MATLAB pid $MatlabPid"
$n = 0
while ($true) {
    if ($StopFile -and (Test-Path -LiteralPath $StopFile)) { Say 'stop file, exiting'; break }
    $pids = Session-Pids
    if ($pids.Count -eq 0) { Say 'MATLAB exited, exiting'; break }
    foreach ($h in [K16]::Shown($pids)) { $null = [K16]::ShowWindow($h, 7); $n++; Say "minimized window $h ($n so far)" }
    Start-Sleep -Milliseconds 1000
}
