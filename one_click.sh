#!/usr/bin/env bash

# One-click launcher for:
# 1) Cleanup stale processes
# 2) Launch UR simulation + MoveIt
# 3) Force-activate controllers
# 4) Start EE trace marker visualization

set -e

ROOT_DIR="/home/woody/simple_fmp_v1"
UR_WS="/home/woody/ws_ur_sim"
ROS_SETUP="/opt/ros/humble/setup.bash"
UR_TYPE="${UR_TYPE:-ur7e}"
ACTION_NAME="/joint_trajectory_controller/follow_joint_trajectory"
ONE_CLICK_LOCALHOST_ONLY="${ONE_CLICK_LOCALHOST_ONLY:-${ROS_LOCALHOST_ONLY:-1}}"
ONE_CLICK_DOMAIN_ID="${ONE_CLICK_DOMAIN_ID:-${ROS_DOMAIN_ID:-0}}"
PREALIGN_TARGET="${PREALIGN_TARGET:-joint}"
PREALIGN_DURATION_SEC="${PREALIGN_DURATION_SEC:-3}"
EE_PLANE_START_Q="${EE_PLANE_START_Q:-[0.0, -1.57, 1.57, -1.57, -1.57, 0.0]}"
EE_TRACE_MAX_POINTS="${EE_TRACE_MAX_POINTS:-20000}"
EE_TRACE_EE_LINK="${EE_TRACE_EE_LINK:-tool0}"
RUNTIME_BACKEND="${RUNTIME_BACKEND:-cpp_bridge}"
ENABLE_OBSTACLES="${ENABLE_OBSTACLES:-0}"
OBSTACLE_CONFIG_FILE="${OBSTACLE_CONFIG_FILE:-}"
OBSTACLE_WORLD_NAME="${OBSTACLE_WORLD_NAME:-}"
GZ_WORLD_READY_RETRIES="${GZ_WORLD_READY_RETRIES:-10}"
GZ_WORLD_READY_INTERVAL_SEC="${GZ_WORLD_READY_INTERVAL_SEC:-1}"

OBSTACLE_STATUS="disabled"
OBSTACLE_LOG_FILE=""
OBSTACLE_ANALYTIC_JSON=""
MOVE_GROUP_STATUS="unchecked"

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="/tmp/one_click_${TIMESTAMP}"
mkdir -p "${LOG_DIR}"

log() {
  echo "[one_click] $*"
}

usage() {
  cat <<'EOF'
Usage:
  ./one_click.sh start [--prealign]
  ./one_click.sh stop

Environment:
  ONE_CLICK_LOCALHOST_ONLY (default: 1)
  ONE_CLICK_DOMAIN_ID      (default: 0)
  PREALIGN_TARGET          (joint|ee_plane|[q1,...,q6], default: joint)
  EE_PLANE_START_Q         (only used when PREALIGN_TARGET=ee_plane)
  PREALIGN_DURATION_SEC    (default: 3)
  EE_TRACE_MAX_POINTS      (default: 20000)
  EE_TRACE_EE_LINK         (default: tool0)
  RUNTIME_BACKEND          (python|cpp_bridge, default: cpp_bridge)
  ENABLE_OBSTACLES         (0|1, default: 0)
  OBSTACLE_CONFIG_FILE     (default: <pkg_share>/config/obstacles_default.json)
  OBSTACLE_WORLD_NAME      (default: from config world_name, fallback empty)
  GZ_WORLD_READY_RETRIES   (default: 10)
  GZ_WORLD_READY_INTERVAL_SEC (default: 1)
EOF
}

cleanup_stale() {
  log "Cleaning stale processes..."
  pkill -9 -f "ur_sim_moveit.launch.py" || true
  pkill -9 -f "moveit_ros_move_group/move_group" || true
  pkill -9 -f "robot_state_publisher/robot_state_publisher" || true
  pkill -9 -f "/opt/ros/humble/lib/rviz2/rviz2" || true
  pkill -9 -f "ign gazebo" || true
  pkill -9 -f "gzserver" || true
  pkill -9 -f "gzclient" || true
  pkill -9 -f "/usr/bin/gazebo" || true
  pkill -9 -f "gz sim" || true
  pkill -9 -f "gz_ros2_control" || true
  pkill -9 -f "ros_gz_bridge/parameter_bridge" || true
  pkill -9 -f "moveit_servo/servo_node_main" || true
  pkill -9 -f "intent_hybrid_planner/ee_trace_marker" || true
  pkill -9 -f "intent_hybrid_runtime_cpp/intent_runtime_bridge" || true
  pkill -9 -f "intent_runtime_bridge" || true
  sleep 2
}

source_env() {
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"
  # shellcheck disable=SC1091
  source "${UR_WS}/install/setup.bash"
  # shellcheck disable=SC1091
  source "${ROOT_DIR}/install/setup.bash"
}

prepend_unique_path_var() {
  local var_name="$1"
  local new_path="$2"
  local current="${!var_name:-}"

  if [[ -z "${new_path}" || ! -d "${new_path}" ]]; then
    return
  fi
  current="${current#:}"
  current="${current%:}"
  case ":${current}:" in
    *":${new_path}:"*) ;;
    "")
      export "${var_name}=${new_path}"
      ;;
    *)
      export "${var_name}=${new_path}:${current}"
      ;;
  esac
}

setup_gz_resource_paths() {
  local ur_desc_prefix ur_sim_prefix
  ur_desc_prefix="$(ros2 pkg prefix ur_description 2>/dev/null || true)"
  ur_sim_prefix="$(ros2 pkg prefix ur_simulation_gz 2>/dev/null || true)"

  prepend_unique_path_var "GZ_SIM_RESOURCE_PATH" "${ur_desc_prefix}/share"
  prepend_unique_path_var "GZ_SIM_RESOURCE_PATH" "${ur_sim_prefix}/share"
  prepend_unique_path_var "GZ_SIM_RESOURCE_PATH" "/opt/ros/humble/share"

  prepend_unique_path_var "IGN_GAZEBO_RESOURCE_PATH" "${ur_desc_prefix}/share"
  prepend_unique_path_var "IGN_GAZEBO_RESOURCE_PATH" "${ur_sim_prefix}/share"
  prepend_unique_path_var "IGN_GAZEBO_RESOURCE_PATH" "/opt/ros/humble/share"

  log "GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH:-<unset>}"
  log "IGN_GAZEBO_RESOURCE_PATH=${IGN_GAZEBO_RESOURCE_PATH:-<unset>}"
}

wait_for_service() {
  local service_name="$1"
  local timeout_sec="${2:-60}"
  local start_ts
  start_ts="$(date +%s)"
  while true; do
    if ros2 service list 2>/dev/null | grep -q "^${service_name}$"; then
      return 0
    fi
    if (( "$(date +%s)" - start_ts >= timeout_sec )); then
      return 1
    fi
    sleep 1
  done
}

controller_active() {
  local name="$1"
  ros2 control list_controllers -c /controller_manager 2>/dev/null \
    | sed -r 's/\x1B\[[0-9;]*[A-Za-z]//g' \
    | awk -v n="$name" '$1==n && $3=="active"{ok=1} END{exit(ok?0:1)}'
}

ensure_controller_active() {
  local ctrl="$1"
  local i=0
  while (( i < 8 )); do
    if controller_active "${ctrl}"; then
      log "Controller active: ${ctrl}"
      return 0
    fi
    if (( i == 0 )); then
      log "Activating controller: ${ctrl}"
    fi
    ros2 control load_controller -c /controller_manager --set-state active "${ctrl}" >/dev/null 2>&1 || true
    ros2 run controller_manager spawner "${ctrl}" --controller-manager /controller_manager >/dev/null 2>&1 || true
    sleep 1
    i=$((i + 1))
  done

  log "ERROR: Failed to activate controller ${ctrl}"
  return 1
}

wait_for_action_server() {
  local action_name="$1"
  local timeout_sec="${2:-30}"
  local start_ts
  start_ts="$(date +%s)"
  while true; do
    if ros2 action info "${action_name}" 2>/dev/null | grep -q "Action servers: 1"; then
      return 0
    fi
    if (( "$(date +%s)" - start_ts >= timeout_sec )); then
      return 1
    fi
    sleep 1
  done
}

check_move_group_alive() {
  local timeout_sec="${1:-20}"
  local start_ts
  start_ts="$(date +%s)"
  while true; do
    if ros2 node list 2>/dev/null | grep -q "^/move_group$"; then
      MOVE_GROUP_STATUS="ready"
      return 0
    fi
    if (( "$(date +%s)" - start_ts >= timeout_sec )); then
      MOVE_GROUP_STATUS="missing"
      return 1
    fi
    sleep 1
  done
}

start_simulation() {
  log "Launching UR simulation (ur_type=${UR_TYPE})..."
  # Keep the ROS2 launch parent alive in its own session. A plain
  # nohup/background launch can leave Gazebo orphaned while move_group/rviz2
  # disappear, which makes /check_state_validity and /compute_fk show up in the
  # graph but remain unusable.
  nohup setsid bash -lc "
    source '${ROS_SETUP}'
    source '${UR_WS}/install/setup.bash'
    source '${ROOT_DIR}/install/setup.bash'
    export ROS_LOCALHOST_ONLY='${ROS_LOCALHOST_ONLY}'
    export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}'
    export GZ_SIM_RESOURCE_PATH='${GZ_SIM_RESOURCE_PATH:-}'
    export IGN_GAZEBO_RESOURCE_PATH='${IGN_GAZEBO_RESOURCE_PATH:-}'
    exec ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:='${UR_TYPE}'
  " > "${LOG_DIR}/ur_sim_moveit.log" 2>&1 &
  echo $! > "${LOG_DIR}/launch.pid"
  log "Launch PID: $(cat "${LOG_DIR}/launch.pid")"
}

start_trace_marker() {
  log "Starting EE trace marker..."
  nohup ros2 run intent_hybrid_planner ee_trace_marker \
    --max-points "${EE_TRACE_MAX_POINTS}" \
    --ee-link "${EE_TRACE_EE_LINK}" \
    > "${LOG_DIR}/ee_trace_marker.log" 2>&1 &
  echo $! > "${LOG_DIR}/ee_trace.pid"
}

start_runtime_bridge_if_enabled() {
  local pkg_prefix=""
  local bridge_bin=""
  local bridge_pid=""

  if [[ "${RUNTIME_BACKEND}" != "cpp_bridge" ]]; then
    return 0
  fi

  pkg_prefix="$(ros2 pkg prefix intent_hybrid_runtime_cpp 2>/dev/null || true)"
  bridge_bin="${pkg_prefix}/lib/intent_hybrid_runtime_cpp/intent_runtime_bridge"
  if [[ -z "${pkg_prefix}" || ! -x "${bridge_bin}" ]]; then
    log "ERROR: intent_runtime_bridge binary not found: ${bridge_bin}"
    log "Please run: colcon build --packages-select intent_hybrid_runtime_cpp"
    return 1
  fi

  log "Starting C++ runtime bridge: ${bridge_bin}"
  # Use setsid to detach bridge into a new session; this avoids parent-shell
  # lifecycle interference and keeps bridge alive across launcher exit.
  nohup setsid "${bridge_bin}" \
    > "${LOG_DIR}/intent_runtime_bridge.log" 2>&1 &
  bridge_pid="$!"
  echo "${bridge_pid}" > "${LOG_DIR}/intent_runtime_bridge.pid"
  sleep 1
  if ! kill -0 "${bridge_pid}" 2>/dev/null; then
    log "ERROR: intent_runtime_bridge exited right after launch."
    log "Check ${LOG_DIR}/intent_runtime_bridge.log"
    return 1
  fi

  if ! wait_for_service "/intent_runtime/check_states_batch" 20; then
    log "ERROR: /intent_runtime/check_states_batch is not ready."
    return 1
  fi
  if ! wait_for_service "/intent_runtime/dispatch_joint_trajectory" 20; then
    log "ERROR: /intent_runtime/dispatch_joint_trajectory is not ready."
    return 1
  fi
  if ! wait_for_service "/intent_runtime/publish_planning_markers" 20; then
    log "ERROR: /intent_runtime/publish_planning_markers is not ready."
    return 1
  fi
  if ! kill -0 "${bridge_pid}" 2>/dev/null; then
    log "ERROR: intent_runtime_bridge exited unexpectedly after advertising services."
    log "Check ${LOG_DIR}/intent_runtime_bridge.log"
    return 1
  fi

  log "C++ runtime bridge services are ready (pid=${bridge_pid})."
  return 0
}

resolve_prealign_target_q() {
  if [[ -z "${PREALIGN_TARGET}" || "${PREALIGN_TARGET}" == "joint" ]]; then
    echo "[-0.8, -1.57, 1.57, -1.57, -1.57, 0.0]"
    return
  fi
  if [[ "${PREALIGN_TARGET}" == "ee_plane" ]]; then
    echo "${EE_PLANE_START_Q}"
    return
  fi
  echo "${PREALIGN_TARGET}"
}

run_prealign() {
  local target_q="$1"
  local goal_text
  goal_text="$(cat <<EOF
{
  trajectory: {
    joint_names: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
    points: [{positions: ${target_q}, time_from_start: {sec: ${PREALIGN_DURATION_SEC}, nanosec: 0}}]
  }
}
EOF
)"

  log "Sending pre-alignment goal (target=${PREALIGN_TARGET}, duration=${PREALIGN_DURATION_SEC}s)..."
  if ros2 action send_goal "${ACTION_NAME}" control_msgs/action/FollowJointTrajectory "${goal_text}" \
    2>&1 | tee "${LOG_DIR}/prealign_action.log" | grep -q "SUCCEEDED"; then
    log "Pre-alignment successful."
    return 0
  fi
  log "ERROR: Pre-alignment failed or aborted. Check ${LOG_DIR}/prealign_action.log"
  return 1
}

is_true() {
  local value="${1:-}"
  value="$(echo "${value}" | tr '[:upper:]' '[:lower:]')"
  [[ "${value}" == "1" || "${value}" == "true" || "${value}" == "yes" ]]
}

resolve_obstacle_config_path() {
  if [[ -n "${OBSTACLE_CONFIG_FILE}" ]]; then
    echo "${OBSTACLE_CONFIG_FILE}"
    return
  fi
  local pkg_prefix=""
  pkg_prefix="$(ros2 pkg prefix intent_hybrid_planner 2>/dev/null || true)"
  if [[ -n "${pkg_prefix}" ]]; then
    echo "${pkg_prefix}/share/intent_hybrid_planner/config/obstacles_default.json"
    return
  fi
  echo ""
}

read_world_name_from_config() {
  local config_file="$1"
  if [[ -z "${config_file}" || ! -f "${config_file}" ]]; then
    echo ""
    return
  fi
  python3 - "${config_file}" <<'PY'
import json
import sys

path = sys.argv[1]
try:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    world = str(data.get("world_name", "")).strip()
    print(world)
except Exception:
    print("")
PY
}

config_to_analytic_obstacles_json() {
  local config_file="$1"
  if [[ -z "${config_file}" || ! -f "${config_file}" ]]; then
    echo ""
    return
  fi
  python3 - "${config_file}" <<'PY'
import json
import sys

path = sys.argv[1]
try:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    out = []
    for item in data.get("obstacles", []):
        if not isinstance(item, dict):
            continue
        if str(item.get("type", "cylinder")).strip().lower() != "cylinder":
            continue
        x = float(item.get("x", 0.45))
        y = float(item.get("y", 0.0))
        z = float(item.get("z", 0.4))
        r = float(item.get("radius", 0.08))
        out.append([x, y, z, r])
    print(json.dumps(out, separators=(",", ":")))
except Exception:
    print("")
PY
}

wait_for_gz_world_ready() {
  local world_name="$1"
  local retries="${2:-10}"
  local interval_sec="${3:-1}"
  local i=0

  if ! command -v ign >/dev/null 2>&1; then
    log "ERROR: 'ign' command is not available, cannot check Gazebo world readiness."
    return 1
  fi

  while (( i < retries )); do
    local services
    services="$(ign service -l 2>/dev/null || true)"
    if echo "${services}" | grep -q "^/world/${world_name}/create$" \
      && echo "${services}" | grep -q "^/world/${world_name}/control$"; then
      return 0
    fi
    sleep "${interval_sec}"
    i=$((i + 1))
  done
  return 1
}

spawn_obstacles_if_enabled() {
  local config_file world_name
  OBSTACLE_LOG_FILE="${LOG_DIR}/spawn_obstacles.log"
  config_file="$(resolve_obstacle_config_path)"
  world_name="${OBSTACLE_WORLD_NAME}"

  if [[ -z "${config_file}" || ! -f "${config_file}" ]]; then
    log "ERROR: obstacle config file not found: ${config_file}"
    OBSTACLE_STATUS="failed_config_missing"
    return 1
  fi
  if [[ -z "${world_name}" ]]; then
    world_name="$(read_world_name_from_config "${config_file}")"
  fi
  world_name="${world_name:-empty}"
  OBSTACLE_WORLD_NAME="${world_name}"
  OBSTACLE_CONFIG_FILE="${config_file}"
  OBSTACLE_ANALYTIC_JSON="$(config_to_analytic_obstacles_json "${OBSTACLE_CONFIG_FILE}")"

  log "Obstacle injection enabled."
  log "Obstacle config: ${OBSTACLE_CONFIG_FILE}"
  log "Obstacle world: ${OBSTACLE_WORLD_NAME}"
  if [[ -n "${OBSTACLE_ANALYTIC_JSON}" ]]; then
    log "Obstacle analytic json prepared from config."
  fi

  log "Waiting for Gazebo world services: /world/${OBSTACLE_WORLD_NAME}/create and /world/${OBSTACLE_WORLD_NAME}/control ..."
  if ! wait_for_gz_world_ready "${OBSTACLE_WORLD_NAME}" "${GZ_WORLD_READY_RETRIES}" "${GZ_WORLD_READY_INTERVAL_SEC}"; then
    log "ERROR: Gazebo world services are not ready for obstacle injection."
    OBSTACLE_STATUS="failed_gz_world_not_ready"
    return 1
  fi

  if ros2 run intent_hybrid_planner spawn_obstacles \
    --mode both \
    --config "${OBSTACLE_CONFIG_FILE}" \
    --world-name "${OBSTACLE_WORLD_NAME}" \
    --publish-count 3 \
    > "${OBSTACLE_LOG_FILE}" 2>&1; then
    log "Obstacle injection succeeded."
    OBSTACLE_STATUS="success"
    return 0
  fi

  log "ERROR: obstacle injection failed. Check ${OBSTACLE_LOG_FILE}"
  OBSTACLE_STATUS="failed_spawn_command"
  return 1
}

print_summary() {
  local did_prealign="${1:-false}"
  echo
  log "Done."
  log "Logs:"
  log "  ${LOG_DIR}/ur_sim_moveit.log"
  log "  ${LOG_DIR}/ee_trace_marker.log"
  if [[ -f "${LOG_DIR}/prealign_action.log" ]]; then
    log "  ${LOG_DIR}/prealign_action.log"
  fi
  if [[ -f "${LOG_DIR}/intent_runtime_bridge.log" ]]; then
    log "  ${LOG_DIR}/intent_runtime_bridge.log"
  fi
  if [[ -n "${OBSTACLE_LOG_FILE}" && -f "${OBSTACLE_LOG_FILE}" ]]; then
    log "  ${OBSTACLE_LOG_FILE}"
  fi
  echo
  log "Controller status:"
  ros2 control list_controllers -c /controller_manager || true
  echo
  log "Action status:"
  ros2 action info "${ACTION_NAME}" || true
  echo
  log "MoveIt status: /move_group=${MOVE_GROUP_STATUS}"
  if is_true "${ENABLE_OBSTACLES}"; then
    echo
    log "Obstacle injection status: ${OBSTACLE_STATUS}"
    log "Obstacle config: ${OBSTACLE_CONFIG_FILE:-<unset>}"
    log "Obstacle world: ${OBSTACLE_WORLD_NAME:-<unset>}"
  fi
  echo
  log "Terminal 2/3 setup:"
  cat <<EOF
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
source /opt/ros/humble/setup.bash
source ~/ws_ur_sim/install/setup.bash
source ~/simple_fmp_v1/install/setup.bash
EOF
  echo
  if [[ "${did_prealign}" == "true" ]]; then
    log "Robot is pre-aligned. Run planner directly (full command with source):"
    cat <<EOF
source /opt/ros/humble/setup.bash
source ~/ws_ur_sim/install/setup.bash
source ${ROOT_DIR}/install/setup.bash
ros2 run intent_hybrid_planner intent_hybrid_planner_node --ros-args \
  -p runtime_backend:=${RUNTIME_BACKEND} \
  -p cpp_bridge_collision_required:=true \
  -p moveit_group_name:=ur_manipulator \
  -p use_sim_time:=true \
  -p execution_mode:=offline \
  -p hybrid_mode:=matlab_compat \
  -p trajectory_action_name:=/joint_trajectory_controller/follow_joint_trajectory \
  -p action_path_tolerance_rad:=0.5 \
  -p action_goal_tolerance_rad:=0.2 \
  -p action_goal_time_tolerance_sec:=5.0 \
  -p nominal_dt:=0.12
EOF
  else
    log "Robot is NOT pre-aligned."
    log "Please pre-align manually or run: ./one_click.sh start --prealign"
  fi
}

main() {
  local mode="start"
  local mode_set="false"
  local enable_prealign="false"
  local target_q=""

  while (( $# > 0 )); do
    case "$1" in
      start|stop)
        if [[ "${mode_set}" == "true" ]]; then
          log "ERROR: Multiple modes specified."
          usage
          exit 1
        fi
        mode="$1"
        mode_set="true"
        ;;
      --prealign)
        enable_prealign="true"
        ;;
      -h|--help)
        usage
        exit 0
        ;;
      *)
        log "ERROR: Unknown argument: $1"
        usage
        exit 1
        ;;
    esac
    shift
  done

  if [[ "${mode}" == "stop" ]]; then
    cleanup_stale
    log "Stopped."
    exit 0
  fi
  if [[ "${mode}" != "start" ]]; then
    log "ERROR: Unknown mode: ${mode}"
    usage
    exit 1
  fi

  cleanup_stale
  source_env
  export ROS_LOCALHOST_ONLY="${ONE_CLICK_LOCALHOST_ONLY}"
  export ROS_DOMAIN_ID="${ONE_CLICK_DOMAIN_ID}"
  log "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
  log "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
  log "RUNTIME_BACKEND=${RUNTIME_BACKEND}"
  setup_gz_resource_paths

  ros2 daemon stop >/dev/null 2>&1 || true
  ros2 daemon start >/dev/null 2>&1 || true
  sleep 1

  start_simulation

  log "Waiting for /controller_manager/list_controllers..."
  if ! wait_for_service "/controller_manager/list_controllers" 90; then
    log "ERROR: controller_manager service not ready. Check ${LOG_DIR}/ur_sim_moveit.log"
    exit 1
  fi

  ensure_controller_active "joint_state_broadcaster"
  ensure_controller_active "joint_trajectory_controller"

  log "Waiting for action server ${ACTION_NAME}..."
  if ! wait_for_action_server "${ACTION_NAME}" 30; then
    log "ERROR: action server not ready. Check ${LOG_DIR}/ur_sim_moveit.log"
    exit 1
  fi

  if ! check_move_group_alive 20; then
    log "WARNING: /move_group is not visible. cpp_bridge collision/FK services may fallback."
  fi

  if [[ "${enable_prealign}" == "true" ]]; then
    target_q="$(resolve_prealign_target_q)"
    log "Pre-align target q: ${target_q}"
    run_prealign "${target_q}"
  fi

  start_runtime_bridge_if_enabled

  if is_true "${ENABLE_OBSTACLES}"; then
    if ! spawn_obstacles_if_enabled; then
      log "WARNING: obstacle injection failed; simulation continues without guaranteed obstacle consistency."
    fi
  fi

  start_trace_marker
  print_summary "${enable_prealign}"
}

main "$@"
