set -euo pipefail

sync

echo 3 | sudo tee /proc/sys/vm/drop_caches > /dev/null

echo "[SYSTEM] Configuring \"Linux\" \"HugePages\" size ( 2 GB total )..."
echo 1024 | sudo tee /sys/kernel/mm/hugepages/hugepages-2048kB/nr_hugepages

echo -e "\n[SYSTEM] Building the network infrastructure..."
./infrastructure/setup_topology.sh

echo -e "\n[SYSTEM] Launching \"Docker\" containers..."
./infrastructure/start_microservices.sh 

echo -e "\n[SYSTEM] Scenario fully loaded!"