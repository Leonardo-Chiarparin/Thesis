#!/bin/bash

set -euo pipefail

echo -e "[SYSTEM] Stopping active microservices...\n"
sudo docker rm -f camera sff1 encoder sff2 decoder sff3 user 2>/dev/null || true

echo -e "\n[SYSTEM] Erasing current \"OVS\"-\"DPDK\" topology..."

sudo ovs-vsctl --if-exists del-br br-sfc

sudo rm -f /tmp/vh-*
sudo rm -f /tmp/*_dpdk

echo -e "\n[SYSTEM] Disabling the \"OVS\"-\"DPDK\" engine..."

sudo ovs-vsctl --if-exists remove Open_vSwitch . other_config dpdk-init
sudo ovs-vsctl --if-exists remove Open_vSwitch . other_config dpdk-lcore-mask
sudo ovs-vsctl --if-exists remove Open_vSwitch . other_config pmd-cpu-mask
sudo ovs-vsctl --if-exists remove Open_vSwitch . other_config dpdk-socket-mem

sudo systemctl stop openvswitch-switch

echo -e "\n[SYSTEM] Releasing allocated \"Linux\" \"HugePages\"..."
echo 0 | sudo tee /sys/kernel/mm/hugepages/hugepages-2048kB/nr_hugepages

sync

echo 3 | sudo tee /proc/sys/vm/drop_caches > /dev/null

echo -e "\n[SYSTEM] Environment correctly shut down!"