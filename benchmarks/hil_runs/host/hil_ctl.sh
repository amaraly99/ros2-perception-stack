#!/usr/bin/env bash
# hil_ctl.sh <command...> — send one line to the MATLAB supervisor and print its one-line reply.
#   hil_ctl.sh status | pub_start [hz] | pub_stop | start | stop
# Host/port: HIL_CTL_HOST (default 192.168.56.1), HIL_CTL_PORT (default 55556). Exit 1 if unreachable / empty reply.
H="${HIL_CTL_HOST:-192.168.56.1}"; P="${HIL_CTL_PORT:-55556}"
# MATLAB (Java println on Windows) ends replies with \r\n: strip the \r so exact comparisons ("ok") work
r=$(echo "$*" | timeout 5 nc -w 3 "$H" "$P" 2>/dev/null | tr -d '\r') || true
[ -n "$r" ] || { echo "NO_REPLY"; exit 1; }
echo "$r"
