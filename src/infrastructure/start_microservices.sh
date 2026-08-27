set -euo pipefail

QUALITY_CAPTURE="0"

if [ "$QUALITY_CAPTURE" != "0" ] && [ "$QUALITY_CAPTURE" != "1" ]; then
  echo "[SYSTEM] Error: \"QUALITY_CAPTURE\" must be either \"0\" or \"1\"..."
  exit 1
fi

echo -e "[SYSTEM] Creating the \"base\" image...\n"

sudo rm -f /tmp/*_dpdk
sudo docker build -t base ./microservices/base

echo -e "\n[SYSTEM] Removing any existing microservices...\n"

sudo rm -f /tmp/sfc-encoder-ready /tmp/sfc-decoder-ready /tmp/sfc-user-ready
sudo docker rm -f camera sff1 encoder sff2 decoder sff3 user 2>/dev/null || true

# Launches containers sequentially in detached mode, configuring explicit naming, working directories, "CPU" affinity, optional "GPU" access, shared "HugePages" & designed "DPDK" "lcores"
start_node() {
  local NODE_NAME=$1
  local FOLDER_NAME=$2
  local CORE_SET=$3
  local USE_GPU=$4
  local DPDK_CORE=$5

  local ENTRYPOINT="entrypoint.sh"
  local GPU_FLAG=""
  local NETWORK_FLAG="--net none"
  local PORTS=""
  local ENVIRONMENT=""

  if [ "$USE_GPU" == "true" ]; then
    GPU_FLAG="--gpus all"
  fi

  if [ "$NODE_NAME" == "encoder" ]; then
    ENVIRONMENT="--env QUALITY_CAPTURE=$QUALITY_CAPTURE"
  fi

  if [ "$NODE_NAME" == "user" ]; then
    ENVIRONMENT="--env QUALITY_CAPTURE=$QUALITY_CAPTURE"
  
    if [ "$QUALITY_CAPTURE" = "0" ]; then
      NETWORK_FLAG="--network bridge"
      PORTS="-p 8080:8080 -p 9999:9999"
    fi
  fi

  sudo docker run -itd \
    --name "$NODE_NAME" \
    --privileged \
    $GPU_FLAG \
    --env DPDK_CORE="$DPDK_CORE" \
    $ENVIRONMENT \
    --entrypoint "" \
    $NETWORK_FLAG \
    $PORTS \
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

wait_for_node_init() {
  local READY_PATH=$1

  while [ ! -f "$READY_PATH" ]; do
    sleep 0.05

  done
}

# Reference physical topology: { 0, 4 }, { 1, 5 }, { 2, 6 }, { 3, 7 }

start_node "sff2" "sff2" "6" "false" "6"
wait_for_dpdk_init "/tmp/sfc-sff1-sff2"
wait_for_dpdk_init "/tmp/sfc-sff2-enc"
wait_for_dpdk_init "/tmp/sfc-sff2-dec"
wait_for_dpdk_init "/tmp/sfc-sff2-sff3"

start_node "decoder" "decoder" "2,4" "true" "4"
start_node "encoder" "encoder" "5,7" "true" "5"

wait_for_node_init "/tmp/sfc-encoder-ready"
wait_for_node_init "/tmp/sfc-decoder-ready"

start_node "sff3" "sff3" "7" "false" "7"
wait_for_dpdk_init "/tmp/sfc-sff3-usr"

if [ "$QUALITY_CAPTURE" = "1" ]; then
  USER_GROUP="0"
else
  USER_GROUP="0,2"
fi

start_node "user" "user" "$USER_GROUP" "false" "0" 
wait_for_node_init "/tmp/sfc-user-ready"

start_node "sff1" "sff1" "3" "false" "3"
wait_for_dpdk_init "/tmp/sfc-cam-sff1"

if [ "$QUALITY_CAPTURE" = "0" ]; then
  echo ""
  read -r -p "[SYSTEM] Press \"ENTER\" once the remote viewer is linked up..."
  echo ""
fi

start_node "camera" "camera" "1" "false" "1"

echo -e "\n[SYSTEM] All components are up! Element status:\n"

sudo docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Image}}"