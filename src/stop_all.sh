#!/bin/bash

set -euo pipefail

echo -e "[SYSTEM] Stopping active microservices...\n"
sudo docker rm -f camera sff1 encoder sff2 decoder sff3 user 2>/dev/null || true

echo -e "\n[SYSTEM] Erasing current \"OVS\"-\"DPDK\" topology..."

sudo ovs-vsctl --if-exists del-br br-sfc

sudo rm -f /tmp/vh-*

echo -e "\n[SYSTEM] Releasing allocated \"Linux\" \"HugePages\"..."
echo 0 | sudo tee /sys/kernel/mm/hugepages/hugepages-2048kB/nr_hugepages

echo -e "\n[SYSTEM] Environment correctly shut down!"