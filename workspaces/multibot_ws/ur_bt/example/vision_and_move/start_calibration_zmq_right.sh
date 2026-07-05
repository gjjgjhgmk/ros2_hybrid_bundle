#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
CALIBRATION_DOCKER_DIR="${WORKSPACE_DIR}/ws_vision/app/vision_calibration/docker"

CONTAINER_NAME="vision_calibration_local"
CONTAINER_CONFIG="/home/yw/workspace/config/vision_calibration/config_right.yaml"
CONTAINER_SOURCE_DIR="/home/yw/workspace/src/vision_calibration"
CONTAINER_LOG="/tmp/log/zmq_server.log"

cd "${CALIBRATION_DOCKER_DIR}"

docker compose up -d "${CONTAINER_NAME}"

docker compose exec -d "${CONTAINER_NAME}" bash -lc "
source /home/yw/.env_noninteractive
export ROS_DOMAIN_ID=27
cd ${CONTAINER_SOURCE_DIR}
python3 -m vision_calibration.zmq_server \
  --config ${CONTAINER_CONFIG} \
  > ${CONTAINER_LOG} 2>&1
"

echo "Calibration ZMQ server start requested."
echo "Container: ${CONTAINER_NAME}"
echo "Config: ${CONTAINER_CONFIG}"
echo "Log: ${CONTAINER_LOG}"
echo "Check with: docker compose exec ${CONTAINER_NAME} bash -lc 'tail -n 100 ${CONTAINER_LOG}'"
