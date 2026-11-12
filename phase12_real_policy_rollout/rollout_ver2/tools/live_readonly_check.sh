#!/usr/bin/env bash
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT="${1:-${ROOT}/reports/live_readonly_${STAMP}.txt}"
ROBOT_IP="${ROBOT_IP:-192.168.0.110}"
ROBOT_STATE_TOPIC="${ROBOT_STATE_TOPIC:-/dsr01/robot_state}"
JOINT_TOPIC="${JOINT_TOPIC:-/dsr01/joint_states}"

mkdir -p "$(dirname "${OUTPUT}")"
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
if [[ -f /home/ubuntu/robot_ws/install/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /home/ubuntu/robot_ws/install/setup.bash
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

  section "robot_state_qos_and_rate"
  run ros2 topic info "${ROBOT_STATE_TOPIC}" --verbose
  run ros2 topic hz "${ROBOT_STATE_TOPIC}" --window 20

  section "robot_state_one_sample"
  run ros2 topic echo "${ROBOT_STATE_TOPIC}" dsr_msgs2/msg/RobotState --once --field robot_state
  run ros2 topic echo "${ROBOT_STATE_TOPIC}" dsr_msgs2/msg/RobotState --once --field current_posj
  run ros2 topic echo "${ROBOT_STATE_TOPIC}" dsr_msgs2/msg/RobotState --once --field current_posx
  run ros2 topic echo "${ROBOT_STATE_TOPIC}" dsr_msgs2/msg/RobotState --once --field current_tool_posx
  run ros2 topic echo "${ROBOT_STATE_TOPIC}" dsr_msgs2/msg/RobotState --once --field f_current_pos_world

  section "stop_state_io_tool_services"
  run bash -lc "ros2 service list -t | grep -E '/dsr01/(motion/move_stop|system/|io/|tool/)'"

  section "controller_manager_update_rate"
  run ros2 param get /dsr01/controller_manager update_rate
  run ros2 param get /dsr01/dsr_controller2 update_rate

  section "safety_statement"
  printf '%s\n' 'READ_ONLY: no ros2 topic pub, no ros2 service call, no action send.'
} | tee "${OUTPUT}"

printf 'Saved: %s\n' "${OUTPUT}"
