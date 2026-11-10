#!/bin/bash
# Records what test/fit_from_bag.py and test/fit_osc_from_bag.py need. Run on
# the vehicle PC with the driving stack up (auto mode); Ctrl-C to stop.
#   bash waypoint_follower/test/record_fit_bag.sh [output_dir]
# The parameters the node is running with are saved into the bag directory as
# wf_params.yaml, so every bag stays tied to the gains it was driven with.
out=${1:-erp42_fit_$(date +%m%d_%H%M)}
params=$(mktemp)
if ! ros2 param dump /waypoint_follower > "$params" 2>/dev/null || [ ! -s "$params" ]; then
  echo "[record_fit_bag] /waypoint_follower is not running: no wf_params.yaml for this bag" >&2
  rm -f "$params"; params=
fi
# Ctrl-C goes to ros2 bag as well; ignoring it here lets the lines below run.
trap '' INT
ros2 bag record -o "$out" \
  /erp42_racing/control_command /erp42_racing/control_command_raw \
  /erp42_racing/feedback /odom /odom_back /imu/data /pva/ins_status \
  /final_waypoint /gps_state /ego_speed /change_dist \
  /wf_debug/lookahead /wf_debug/cte /wf_debug/terms
[ -n "$params" ] && [ -d "$out" ] && mv "$params" "$out/wf_params.yaml"
echo "[record_fit_bag] saved $out"
