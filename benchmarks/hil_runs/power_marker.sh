#!/usr/bin/env bash
# power_marker.sh: run by Ahmed once after a FULL power removal (supply unplugged), never after a soft reboot (8A).
# Writes $HIL_POWER_DIR/power_cycled_<YYYYMMDD_HHMM> (local time). batch.sh starts only with an unconsumed marker newer than
# the current boot (/proc/stat btime) and consumes it, so the NPU open budget restarts only after a confirmed power-on.
# Refuses if the clock is not NTP-synchronised (the marker time is compared with btime) or if the marker already exists.
set -u; set -o noclobber
source "$(dirname "$(readlink -f "$0")")/hil_env.sh"
[ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" = yes ] || { echo "power_marker: REFUSE: clock not NTP-synchronised yet; wait and run again"; exit 1; }
mkdir -p "$HIL_POWER_DIR" || exit 1
F="$HIL_POWER_DIR/power_cycled_$(date +%Y%m%d_%H%M)"
BT=$(awk '/^btime/ {print $2}' /proc/stat)
echo "full power removal confirmed by Ahmed; written $(date -u +%FT%TZ); boot $(date -d "@$BT" '+%F %T') (btime $BT); $(vcgencmd get_throttled)" > "$F" \
  || { echo "power_marker: REFUSE: $F exists"; exit 1; }
echo "power_marker: wrote $F"; cat "$F"
