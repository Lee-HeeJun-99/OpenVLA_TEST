#!/usr/bin/env bash
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT="${1:-${ROOT}/reports/live_readonly_${STAMP}.txt}"
ROBOT_IP="${ROBOT_IP:-192.168.0.110}"
JOINT_TOPIC="${JOINT_TOPIC:-/dsr01/joint_states}"

mkdir -p "$(dirname "${OUTPUT}")"
if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
fi
if [[ -f /home/ubuntu/robot_ws/install/setup.bash ]]; then
  set +u
  # shellcheck disable=SC1091
  source /home/ubuntu/robot_ws/install/setup.bash
  set -u
fi

section() { printf '\n===== %s =====\n' "$1"; }
run() { printf '$'; printf ' %q' "$@"; printf '\n'; timeout 12s "$@" 2>&1 || true; }

{
  section "metadata"
  date --iso-8601=seconds
  printf 'ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID:-UNSET}"
  printf 'ROS_LOCALHOST_ONLY=%s\n' "${ROS_LOCALHOST_ONLY:-UNSET}"

  section "network_readonly"
  run ip -brief address
  run ping -c 10 -W 1 "${ROBOT_IP}"

  section "topic_list_and_types"
  run ros2 topic list -t

  section "servoj_stream_publishers_qos"
  run ros2 topic info /dsr01/servoj_stream --verbose

  section "joint_state_qos_and_rate"
  run ros2 topic info "${JOINT_TOPIC}" --verbose
  run ros2 topic hz "${JOINT_TOPIC}" --window 20

  section "joint_state_one_sample"
  run ros2 topic echo "${JOINT_TOPIC}" sensor_msgs/msg/JointState --once

  section "read_only_get_services"
  run ros2 service call /dsr01/system/get_robot_state dsr_msgs2/srv/GetRobotState '{}'
  run ros2 service call /dsr01/system/get_robot_mode dsr_msgs2/srv/GetRobotMode '{}'
  run ros2 service call /dsr01/system/get_robot_system dsr_msgs2/srv/GetRobotSystem '{}'
  run ros2 service call /dsr01/aux_control/get_current_posj dsr_msgs2/srv/GetCurrentPosj '{}'
  run ros2 service call /dsr01/aux_control/get_current_tool_flange_posx dsr_msgs2/srv/GetCurrentToolFlangePosx '{ref: 0}'
  run ros2 service call /dsr01/aux_control/get_current_rotm dsr_msgs2/srv/GetCurrentRotm '{ref: 0}'
  run ros2 service call /dsr01/tool/get_current_tool dsr_msgs2/srv/GetCurrentTool '{}'
  run ros2 service call /dsr01/tcp/get_current_tcp dsr_msgs2/srv/GetCurrentTcp '{}'
  run ros2 service call /dsr01/system/get_last_alarm dsr_msgs2/srv/GetLastAlarm '{}'

  section "stop_state_io_tool_services"
  run bash -lc "ros2 service list -t | grep -E '/dsr01/(motion/move_stop|system/|io/|tool/)'"

  section "controller_manager_update_rate"
  run ros2 param get /dsr01/controller_manager update_rate
  run ros2 param get /dsr01/dsr_controller2 update_rate

  section "safety_statement"
  printf '%s\n' 'READ_ONLY: subscriptions and get_* service calls only; no topic pub, set_*/move_*/servo_off, or action send.'
} | tee "${OUTPUT}"

printf 'Saved: %s\n' "${OUTPUT}"
