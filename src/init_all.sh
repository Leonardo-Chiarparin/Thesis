#!/bin/bash

set -euo pipefail

echo "[SYSTEM] Configuring \"Linux\" \"HugePages\" size ( 8 GB total )..."
echo 4096 | sudo tee /sys/kernel/mm/hugepages/hugepages-2048kB/nr_hugepages

echo -e "\n[SYSTEM] Booting the \"OVS\"-\"DPDK\" engine..."

sudo systemctl daemon-reload
sudo ovs-vsctl --no-wait set Open_vSwitch . other_config:dpdk-init=true

# Core assignment: 0 for "Control Plane", 1 for "PMD" ( processing )
sudo ovs-vsctl --no-wait set Open_vSwitch . other_config:dpdk-lcore-mask=0x1
sudo ovs-vsctl --no-wait set Open_vSwitch . other_config:pmd-cpu-mask=0x2
sudo ovs-vsctl --no-wait set Open_vSwitch . other_config:dpdk-socket-mem="1024"
sudo systemctl restart openvswitch-switch

echo "[SYSTEM] Ready to go!"

echo -e "\n[SYSTEM] Building the virtual network infrastructure..."
./infrastructure/setup_topology.sh # use "--debug" to enable "tcpdump" sniffing on the virtual interfaces

echo -e "\n[SYSTEM] Launching \"Docker\" containers..."
./infrastructure/start_microservices.sh

echo -e "\n[SYSTEM] Scenario fully loaded!"