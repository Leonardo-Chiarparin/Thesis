#!/bin/bash

set -euo pipefail

sync

echo 3 | sudo tee /proc/sys/vm/drop_caches > /dev/null

echo "[SYSTEM] Configuring \"Linux\" \"HugePages\" size ( 4 GB total )..."
echo 2048 | sudo tee /sys/kernel/mm/hugepages/hugepages-2048kB/nr_hugepages

echo -e "\n[SYSTEM] Booting the \"OVS\"-\"DPDK\" engine..."

sudo systemctl daemon-reload
sudo systemctl start openvswitch-switch

sudo ovs-vsctl --no-wait set Open_vSwitch . other_config:dpdk-init=true

# "OVS"-"DPDK" core allocation. Logical "CPU 0" hosts auxiliary "lcore" processes, whereas "CPU 1" is dedicated to the "PMD" data path
sudo ovs-vsctl --no-wait set Open_vSwitch . other_config:dpdk-lcore-mask=0x1
sudo ovs-vsctl --no-wait set Open_vSwitch . other_config:pmd-cpu-mask=0x2
sudo ovs-vsctl --no-wait set Open_vSwitch . other_config:dpdk-socket-mem="1024"

sudo systemctl restart openvswitch-switch

echo "[SYSTEM] Ready to go!"

echo -e "\n[SYSTEM] Building the virtual network infrastructure..."
./infrastructure/setup_topology.sh # use "--debug" to enable "tcpdump" packet capture on the virtual interfaces

echo -e "\n[SYSTEM] Launching \"Docker\" containers..."
./infrastructure/start_microservices.sh

echo -e "\n[SYSTEM] Scenario fully loaded!"