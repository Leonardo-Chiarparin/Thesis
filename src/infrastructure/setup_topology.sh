#!/bin/bash

set -euo pipefail

echo "[SYSTEM] Deleting the past topology..."

sudo ovs-vsctl --if-exists del-br br-sfc

sudo rm -f /tmp/vh-*

echo "[SYSTEM] Crafting a single bridge ( \"RFC 7665\" )..."

sudo ovs-vsctl add-br br-sfc -- set bridge br-sfc datapath_type=netdev

echo "[SYSTEM] Switching on \"vhost-user\" ports..."

# Enumerates the complete set of adjacent attachment points connected to the unified service-chain bridge
PORTS=( "vh-cam" "vh-sff1-cam" "vh-sff1-sff2" "vh-sff2-sff1" "vh-sff2-enc" "vh-enc" "vh-dec" "vh-sff2-dec" "vh-sff2-sff3" "vh-sff3-sff2" "vh-sff3-usr" "vh-usr" )

for port in "${PORTS[@]}"; do
  sudo ovs-vsctl add-port br-sfc $port -- set Interface $port type=dpdkvhostuserclient options:vhost-server-path=/tmp/$port
done

echo -e "[SYSTEM] Defining \"OpenFlow\" network rules..."

sudo ovs-ofctl del-flows br-sfc

# Camera <-> SFF1
sudo ovs-ofctl add-flow br-sfc "priority=100, in_port=vh-cam, actions=output:vh-sff1-cam"
sudo ovs-ofctl add-flow br-sfc "priority=100, in_port=vh-sff1-cam, actions=output:vh-cam"

# SFF1 <-> SFF2
sudo ovs-ofctl add-flow br-sfc "priority=100, in_port=vh-sff1-sff2, actions=output:vh-sff2-sff1"
sudo ovs-ofctl add-flow br-sfc "priority=100, in_port=vh-sff2-sff1, actions=output:vh-sff1-sff2"

# SFF2 <-> Encoder
sudo ovs-ofctl add-flow br-sfc "priority=100, in_port=vh-sff2-enc, actions=output:vh-enc"
sudo ovs-ofctl add-flow br-sfc "priority=100, in_port=vh-enc, actions=output:vh-sff2-enc"

# SFF2 <-> Decoder 
sudo ovs-ofctl add-flow br-sfc "priority=100, in_port=vh-sff2-dec, actions=output:vh-dec"
sudo ovs-ofctl add-flow br-sfc "priority=100, in_port=vh-dec, actions=output:vh-sff2-dec"

# SFF2 <-> SFF3
sudo ovs-ofctl add-flow br-sfc "priority=100, in_port=vh-sff2-sff3, actions=output:vh-sff3-sff2"
sudo ovs-ofctl add-flow br-sfc "priority=100, in_port=vh-sff3-sff2, actions=output:vh-sff2-sff3"

# SFF3 <-> User
sudo ovs-ofctl add-flow br-sfc "priority=100, in_port=vh-sff3-usr, actions=output:vh-usr"
sudo ovs-ofctl add-flow br-sfc "priority=100, in_port=vh-usr, actions=output:vh-sff3-usr"

# Drops any packets not explicitly matched by the preceding rules, enforcing a strict "Default-Deny" policy
sudo ovs-ofctl add-flow br-sfc "priority=0, actions=drop"

if [[ "${1:-}" == "--debug" ]]; then
  echo -e "[SYSTEM] Enabling interfaces for \"tcpdump\" ( \"mirroring\" )...\n"

  # Camera -> SFF1
  sudo ovs-vsctl add-port br-sfc cam-eg -- set Interface cam-eg type=internal
  sudo ip link set dev cam-eg up

  sudo ovs-vsctl -- \
    --id=@snp_cam_eg get Port cam-eg -- \
    --id=@prt_cam_eg get Port vh-cam -- \
    --id=@mir_cam_eg create Mirror name=mir_cam_eg select-src-port=@prt_cam_eg output-port=@snp_cam_eg -- \
    add Bridge br-sfc mirrors @mir_cam_eg

  # SFF1 -> Camera
  sudo ovs-vsctl add-port br-sfc sff1-eg-cam -- set Interface sff1-eg-cam type=internal
  sudo ip link set dev sff1-eg-cam up

  sudo ovs-vsctl -- \
    --id=@snp_sff1_eg_cam get Port sff1-eg-cam -- \
    --id=@prt_sff1_eg_cam get Port vh-sff1-cam -- \
    --id=@mir_sff1_eg_cam create Mirror name=mir_sff1_eg_cam select-src-port=@prt_sff1_eg_cam output-port=@snp_sff1_eg_cam -- \
    add Bridge br-sfc mirrors @mir_sff1_eg_cam

  # SFF1 -> SFF2
  sudo ovs-vsctl add-port br-sfc sff1-eg-sff2 -- set Interface sff1-eg-sff2 type=internal
  sudo ip link set dev sff1-eg-sff2 up

  sudo ovs-vsctl -- \
    --id=@snp_sff1_eg_sff2 get Port sff1-eg-sff2 -- \
    --id=@prt_sff1_eg_sff2 get Port vh-sff1-sff2 -- \
    --id=@mir_sff1_eg_sff2 create Mirror name=mir_sff1_eg_sff2 select-src-port=@prt_sff1_eg_sff2 output-port=@snp_sff1_eg_sff2 -- \
    add Bridge br-sfc mirrors @mir_sff1_eg_sff2

  # SFF2 -> SFF1
  sudo ovs-vsctl add-port br-sfc sff2-eg-sff1 -- set Interface sff2-eg-sff1 type=internal
  sudo ip link set dev sff2-eg-sff1 up

  sudo ovs-vsctl -- \
    --id=@snp_sff2_eg_sff1 get Port sff2-eg-sff1 -- \
    --id=@prt_sff2_eg_sff1 get Port vh-sff2-sff1 -- \
    --id=@mir_sff2_eg_sff1 create Mirror name=mir_sff2_eg_sff1 select-src-port=@prt_sff2_eg_sff1 output-port=@snp_sff2_eg_sff1 -- \
    add Bridge br-sfc mirrors @mir_sff2_eg_sff1

  # SFF2 -> Encoder
  sudo ovs-vsctl add-port br-sfc sff2-eg-enc -- set Interface sff2-eg-enc type=internal
  sudo ip link set dev sff2-eg-enc up

  sudo ovs-vsctl -- \
    --id=@snp_sff2_eg_enc get Port sff2-eg-enc -- \
    --id=@prt_sff2_eg_enc get Port vh-sff2-enc -- \
    --id=@mir_sff2_eg_enc create Mirror name=mir_sff2_eg_enc select-src-port=@prt_sff2_eg_enc output-port=@snp_sff2_eg_enc -- \
    add Bridge br-sfc mirrors @mir_sff2_eg_enc

  # Encoder -> SFF2
  sudo ovs-vsctl add-port br-sfc enc-eg -- set Interface enc-eg type=internal
  sudo ip link set dev enc-eg up

  sudo ovs-vsctl -- \
    --id=@snp_enc_eg get Port enc-eg -- \
    --id=@prt_enc_eg get Port vh-enc -- \
    --id=@mir_enc_eg create Mirror name=mir_enc_eg select-src-port=@prt_enc_eg output-port=@snp_enc_eg -- \
    add Bridge br-sfc mirrors @mir_enc_eg

  # SFF2 -> Decoder
  sudo ovs-vsctl add-port br-sfc sff2-eg-dec -- set Interface sff2-eg-dec type=internal
  sudo ip link set dev sff2-eg-dec up

  sudo ovs-vsctl -- \
    --id=@snp_sff2_eg_dec get Port sff2-eg-dec -- \
    --id=@prt_sff2_eg_dec get Port vh-sff2-dec -- \
    --id=@mir_sff2_eg_dec create Mirror name=mir_sff2_eg_dec select-src-port=@prt_sff2_eg_dec output-port=@snp_sff2_eg_dec -- \
    add Bridge br-sfc mirrors @mir_sff2_eg_dec

  # Decoder -> SFF2
  sudo ovs-vsctl add-port br-sfc dec-eg -- set Interface dec-eg type=internal
  sudo ip link set dev dec-eg up

  sudo ovs-vsctl -- \
    --id=@snp_dec_eg get Port dec-eg -- \
    --id=@prt_dec_eg get Port vh-dec -- \
    --id=@mir_dec_eg create Mirror name=mir_dec_eg select-src-port=@prt_dec_eg output-port=@snp_dec_eg -- \
    add Bridge br-sfc mirrors @mir_dec_eg
fi

echo -e "[SYSTEM] Topology successfully made!\\n"

sudo ovs-vsctl show