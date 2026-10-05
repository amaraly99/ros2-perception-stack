#!/usr/bin/env bash
# Logs temperature, ARM clock and throttle state every few seconds while an experiment runs.
#   bash pi_telemetry.sh run1.csv 1 &      (start before the run; 2nd arg is the interval in seconds)
#   kill %1                                (stop after the run)
# Columns: utc, temp_c, arm_mhz, throttled_hex, cur_flags, load1
#   throttled_hex  = sticky bits, they stay set until the next reboot (bits 16-19 mean "has happened")
#   cur_flags      = live bits only (0x1 under-voltage, 0x2 freq capped, 0x4 throttled, 0x8 soft temp limit)
# A run is void if cur_flags is ever non-zero, or arm_mhz drops below 2400 while the detector is busy.
# arm_mhz at 1500 while idle is normal (ondemand governor).

OUT="${1:-telemetry_$(date +%Y%m%d_%H%M%S).csv}"
INT="${2:-5}"
echo "utc,temp_c,arm_mhz,throttled_hex,cur_flags,load1" > "$OUT"
while true; do
  t=$(vcgencmd measure_temp | sed 's/[^0-9.]//g')
  f=$(( $(vcgencmd measure_clock arm | cut -d= -f2) / 1000000 ))
  th=$(vcgencmd get_throttled | cut -d= -f2)
  cur=$(( th & 0xF ))
  l=$(cut -d' ' -f1 /proc/loadavg)
  echo "$(date -u +%FT%TZ),$t,$f,$th,$cur,$l" >> "$OUT"
  sleep "$INT"
done
