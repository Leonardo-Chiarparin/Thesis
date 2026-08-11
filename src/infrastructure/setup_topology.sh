#!/bin/bash

set -euo pipefail

echo "[SYSTEM] Deleting the past topology..."

sudo ovs-vsctl --if-exists del-br br-sfc

sudo rm -f /tmp/vh-*

echo "[SYSTEM] Crafting a single bridge ( \"RFC\" 7665 )..."

sudo ovs-vsctl add-br br-sfc -- set bridge br-sfc datapath_type=netdev

echo "[SYSTEM] Switching on \"vhost-user\" ports..."

# Listing all ports attached to the single bridge
PORTS=( "vh-cam" "vh-sff1-in" "vh-sff1-eg" "vh-sff2-sff1" "vh-sff2-enc" "vh-enc" "vh-dec" "vh-sff2-dec" "vh-sff2-sff3" "vh-sff3-in" "vh-sff3-eg" "vh-usr" )

for port in "${PORTS[@]}"; do
  sudo ovs-vsctl add-port br-sfc $port -- set Interface $port type=dpdkvhostuserclient options:vhost-server-path=/tmp/$port
done

echo -e "[SYSTEM] Defining \"OpenFlow\" network rules..."

sudo ovs-ofctl del-flows br-sfc

# Primary chain ( "SPI" = 100, "SI" defined according to "RFC" 8300 ): Camera -> SFF1 -> SFF2 -> Encoder -> SFF2 -> Decoder -> SFF2 -> SFF3 -> User
# Flow control + Skip ( "Feedback" ) interaction ( "SPI" = 200 ): User -> SFF3 -> SFF2 -> Encoder -> SFF2 -> SFF1 -> Camera

# Camera -> SFF1
sudo ovs-ofctl add-flow br-sfc "priority=100, in_port=vh-cam, actions=output:vh-sff1-in"

# SFF1 <-> SFF2
sudo ovs-ofctl add-flow br-sfc "priority=100, in_port=vh-sff1-eg, actions=output:vh-sff2-sff1"
sudo ovs-ofctl add-flow br-sfc "priority=100, in_port=vh-sff2-sff1, actions=output:vh-sff1-eg"

# SFF2 <-> Encoder
sudo ovs-ofctl add-flow br-sfc "priority=100, in_port=vh-sff2-enc, actions=output:vh-enc"
sudo ovs-ofctl add-flow br-sfc "priority=100, in_port=vh-enc, actions=output:vh-sff2-enc"

# SFF2 <-> Decoder 
sudo ovs-ofctl add-flow br-sfc "priority=100, in_port=vh-sff2-dec, actions=output:vh-dec"
sudo ovs-ofctl add-flow br-sfc "priority=100, in_port=vh-dec, actions=output:vh-sff2-dec"

# SFF2 <-> SFF3
sudo ovs-ofctl add-flow br-sfc "priority=100, in_port=vh-sff2-sff3, actions=output:vh-sff3-in"
sudo ovs-ofctl add-flow br-sfc "priority=100, in_port=vh-sff3-in, actions=output:vh-sff2-sff3"

# SFF3 <-> User
sudo ovs-ofctl add-flow br-sfc "priority=100, in_port=vh-sff3-eg, actions=output:vh-usr"
sudo ovs-ofctl add-flow br-sfc "priority=100, in_port=vh-usr, actions=output:vh-sff3-eg"

# Dropping any other packets that is not explicitly defined in the above statements
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

  # SFF1 -> SFF2
  sudo ovs-vsctl add-port br-sfc sff1-eg -- set Interface sff1-eg type=internal
  sudo ip link set dev sff1-eg up

  sudo ovs-vsctl -- \
    --id=@snp_sff1_eg get Port sff1-eg -- \
    --id=@prt_sff1_eg get Port vh-sff1-eg -- \
    --id=@mir_sff1_eg create Mirror name=mir_sff1_eg select-src-port=@prt_sff1_eg output-port=@snp_sff1_eg -- \
    add Bridge br-sfc mirrors @mir_sff1_eg

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
fi

echo -e "[SYSTEM] Topology successfully made!\\n"

sudo ovs-vsctl show