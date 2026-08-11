#!/bin/bash

set -euo pipefail

echo -e "[SYSTEM] Creating the \"base\" image...\n"

sudo rm -f /tmp/*_dpdk
sudo docker build -t base ./microservices/base

echo -e "\n[SYSTEM] Removing any existing microservices...\n"

sudo docker rm -f camera sff1 encoder sff2 decoder sff3 user 2>/dev/null || true

# Function to launch containers in detached mode, with the specified name and folder
start_node() {
  local NODE_NAME=$1
  local FOLDER_NAME=$2
  local CORE_SET=$3
  local USE_GPU=$4
  local DPDK_CORE=$5

  local ENTRYPOINT="entrypoint.sh"
  local GPU_FLAG=""

  if [ "$USE_GPU" == "true" ]; then
    GPU_FLAG="--gpus all"
  fi

  sudo docker run -itd \
    --name "$NODE_NAME" \
    --privileged \
    $GPU_FLAG \
    --env DPDK_CORE="$DPDK_CORE" \
    --entrypoint "" \
    --net none \
    --cpuset-cpus="$CORE_SET" \
    -v /dev/hugepages:/dev/hugepages \
    -v /tmp/:/tmp/ \
    -v "$(pwd)/shared:/shared" \
    -v "$(pwd)/microservices/$FOLDER_NAME:/app" \
    base bash -c "if [ -f /app/$ENTRYPOINT ]; then chmod +x /app/$ENTRYPOINT && /app/$ENTRYPOINT; else tail -f /dev/null; fi"
}

wait_for_dpdk_init() {
  local SOCKET_PATH=$1

  while [ ! -S "$SOCKET_PATH" ]; do
    sleep 0.5

  done
}

start_node "user" "user" "7" "false" "7" # + "rendering"
# wait_for_dpdk_init "/tmp/vh-usr"

start_node "decoder" "decoder" "0,6" "true" "6"
# wait_for_dpdk_init "/tmp/vh-dec"

start_node "encoder" "encoder" "0,5" "true" "5"
wait_for_dpdk_init "/tmp/vh-enc"

start_node "sff2" "sff2" "4" "false" "4"
wait_for_dpdk_init "/tmp/vh-sff2-sff1"
wait_for_dpdk_init "/tmp/vh-sff2-enc"
wait_for_dpdk_init "/tmp/vh-sff2-dec"
wait_for_dpdk_init "/tmp/vh-sff2-sff3"

start_node "sff3" "sff3" "3" "false" "3"
# wait_for_dpdk_init "/tmp/vh-sff3-in"
# wait_for_dpdk_init "/tmp/vh-sff3-eg"

start_node "sff1" "sff1" "3" "false" "3"
wait_for_dpdk_init "/tmp/vh-sff1-in"
wait_for_dpdk_init "/tmp/vh-sff1-eg"

start_node "camera" "camera" "2" "false" "2"
wait_for_dpdk_init "/tmp/vh-cam"

echo -e "\n[SYSTEM] All components are up! Element status:\n"

sudo docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Image}}"