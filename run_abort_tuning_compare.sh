#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="${ROOT_DIR:-/home/woody/simple_fmp_v1}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
UR_WS="${UR_WS:-/home/woody/ws_ur_sim}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RUNTIME_BACKEND="${RUNTIME_BACKEND:-cpp_bridge}"
export ENABLE_OBSTACLES="${ENABLE_OBSTACLES:-1}"

TS="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${ROOT_DIR}/debug/abort_compare_${TS}"
mkdir -p "${OUT_DIR}"
SUMMARY_CSV="${OUT_DIR}/summary.csv"
echo "case_name,dispatch_result,abort_count,tolerance_fail_count,bridge_summary" > "${SUMMARY_CSV}"

set +u
source "${ROS_SETUP}"
source "${UR_WS}/install/setup.bash"
source "${ROOT_DIR}/install/setup.bash"
set -u

run_case() {
  local case_name="$1"
  local planner_args="$2"
  echo "[compare] ===== case: ${case_name} ====="
  cd "${ROOT_DIR}"
  ./one_click.sh stop >/dev/null 2>&1 || true
  ./one_click.sh start --prealign > "${OUT_DIR}/${case_name}_one_click.log" 2>&1
  local one_click_dir
  one_click_dir="$(ls -dt /tmp/one_click_* 2>/dev/null | head -n1)"
  echo "[compare] one_click_dir=${one_click_dir}"

  set +e
  timeout 70 ros2 run intent_hybrid_planner intent_hybrid_planner_node --ros-args \
    -p runtime_backend:=cpp_bridge \
    -p use_sim_time:=true \
    -p execution_mode:=offline \
    -p offline_start_delay_sec:=1.0 \
    -p hybrid_mode:=matlab_compat \
    -p trajectory_action_name:=/joint_trajectory_controller/follow_joint_trajectory \
    ${planner_args} \
    > "${OUT_DIR}/${case_name}_planner.log" 2>&1
  local rc=$?
  set -e
  if [[ "${rc}" -ne 0 && "${rc}" -ne 124 ]]; then
    echo "[compare] planner exited with rc=${rc}"
  fi

  local dispatch_result
  dispatch_result="$(rg -N -n \"Offline execution dispatch result:\" "${OUT_DIR}/${case_name}_planner.log" | tail -n1 | sed -E 's/.*result: ([^ ]+).*/\1/' || true)"
  dispatch_result="${dispatch_result:-unknown}"

  local abort_count tol_count bridge_summary
  abort_count="$(rg -N -c \"Aborted due to state tolerance violation\" "${one_click_dir}/ur_sim_moveit.log" || true)"
  tol_count="$(rg -N -c \"State tolerances failed\" "${one_click_dir}/ur_sim_moveit.log" || true)"
  bridge_summary="$(rg -N -n \"dispatch trajectory summary:\" "${one_click_dir}/intent_runtime_bridge.log" | tail -n1 | sed 's/\"/\"\"/g' || true)"
  bridge_summary="${bridge_summary:-none}"

  cp -f "${one_click_dir}/ur_sim_moveit.log" "${OUT_DIR}/${case_name}_ur_sim_moveit.log" 2>/dev/null || true
  cp -f "${one_click_dir}/intent_runtime_bridge.log" "${OUT_DIR}/${case_name}_intent_runtime_bridge.log" 2>/dev/null || true

  echo "${case_name},${dispatch_result},${abort_count},${tol_count},\"${bridge_summary}\"" >> "${SUMMARY_CSV}"
  echo "[compare] case=${case_name} dispatch=${dispatch_result} abort=${abort_count} tol_fail=${tol_count}"
}

echo "[compare] building required packages ..."
cd "${ROOT_DIR}"
colcon build --packages-select intent_hybrid_interfaces intent_hybrid_runtime_cpp intent_hybrid_planner --symlink-install > "${OUT_DIR}/build.log" 2>&1

# 方案A：降速
run_case "slowdown" "-p nominal_dt:=0.16"

# 方案B：放宽Action容差
run_case "relaxed_tolerance" "-p action_path_tolerance_rad:=0.30 -p action_goal_tolerance_rad:=0.30 -p action_goal_time_tolerance_sec:=1.0"

./one_click.sh stop >/dev/null 2>&1 || true
echo "[compare] done: ${SUMMARY_CSV}"
cat "${SUMMARY_CSV}"
