#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="${ROOT_DIR:-/home/woody/simple_fmp_v1}"
UR_WS="${UR_WS:-/home/woody/ws_ur_sim}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"

export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
RUNTIME_BACKEND="${RUNTIME_BACKEND:-cpp_bridge}"
PLANNER_TIMEOUT_SEC="${PLANNER_TIMEOUT_SEC:-45}"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${OUT_DIR:-${ROOT_DIR}/debug/offline_debug_${TIMESTAMP}}"
mkdir -p "${OUT_DIR}"

set +u
source "${ROS_SETUP}"
source "${UR_WS}/install/setup.bash"
source "${ROOT_DIR}/install/setup.bash"
set -u

echo "[debug] OUT_DIR=${OUT_DIR}"
echo "[debug] ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
echo "[debug] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

REC_PID=""
cleanup() {
  if [[ -n "${REC_PID}" ]]; then
    pkill -INT -P "${REC_PID}" >/dev/null 2>&1 || true
    kill -INT "${REC_PID}" >/dev/null 2>&1 || true
    wait "${REC_PID}" >/dev/null 2>&1 || true
    pkill -f "offline_debug_recorder --out-dir ${OUT_DIR}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

echo "[debug] starting offline_debug_recorder ..."
ros2 run intent_hybrid_planner offline_debug_recorder \
  --out-dir "${OUT_DIR}" \
  --frame-id base_link \
  --ee-link tool0 \
  > "${OUT_DIR}/offline_debug_recorder.log" 2>&1 &
REC_PID="$!"
sleep 1

echo "[debug] running planner ..."
set +e
timeout "${PLANNER_TIMEOUT_SEC}" ros2 run intent_hybrid_planner intent_hybrid_planner_node --ros-args \
  -p runtime_backend:="${RUNTIME_BACKEND}" \
  -p use_sim_time:=true \
  -p execution_mode:=offline \
  -p offline_start_delay_sec:=1.0 \
  -p hybrid_mode:=matlab_compat \
  -p trajectory_action_name:=/joint_trajectory_controller/follow_joint_trajectory 2>&1 \
  | tee "${OUT_DIR}/planner.log"
planner_rc=${PIPESTATUS[0]}
set -e
if [[ "${planner_rc}" -ne 0 && "${planner_rc}" -ne 124 ]]; then
  echo "[debug] planner exited with error code ${planner_rc}"
fi

echo "[debug] planner finished, stopping recorder ..."
cleanup
REC_PID=""

LATEST_ONE_CLICK="$(ls -dt /tmp/one_click_* 2>/dev/null | head -n1 || true)"
if [[ -n "${LATEST_ONE_CLICK}" ]]; then
  cp -f "${LATEST_ONE_CLICK}/intent_runtime_bridge.log" "${OUT_DIR}/intent_runtime_bridge.log" 2>/dev/null || true
  cp -f "${LATEST_ONE_CLICK}/ur_sim_moveit.log" "${OUT_DIR}/ur_sim_moveit.log" 2>/dev/null || true
  cp -f "${LATEST_ONE_CLICK}/spawn_obstacles.log" "${OUT_DIR}/spawn_obstacles.log" 2>/dev/null || true
fi

echo "[debug] done. artifacts:"
echo "  ${OUT_DIR}/planner.log"
echo "  ${OUT_DIR}/ee_trace.csv"
echo "  ${OUT_DIR}/controller_state.csv"
echo "  ${OUT_DIR}/rosout_filtered.csv"
echo "  ${OUT_DIR}/action_status.csv"
echo "  ${OUT_DIR}/meta.json"
