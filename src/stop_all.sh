set -euo pipefail

echo -e "[SYSTEM] Stopping active microservices...\n"

if sudo docker ps -a --format '{{.Names}}' | grep -qE "^(camera|sff1|encoder|sff2|decoder|sff3|user)$"; then
  sudo docker rm -f camera sff1 encoder sff2 decoder sff3 user 2>/dev/null || true
  echo -e "\n[SYSTEM] Erasing current \"DPDK\" topology..."
else
  echo "[SYSTEM] Erasing current \"DPDK\" topology..."
fi

sudo rm -f /tmp/sfc-*
sudo rm -f /tmp/vh-*
sudo rm -f /tmp/*_dpdk

sudo rm -f /dev/hugepages/cameramap_* /dev/hugepages/sff1map_* /dev/hugepages/sff2map_*  /dev/hugepages/encodermap_* /dev/hugepages/decodermap_* /dev/hugepages/sff3map_* /dev/hugepages/usermap_*

echo -e "\n[SYSTEM] Releasing allocated \"Linux\" \"HugePages\"..."
echo 0 | sudo tee /sys/kernel/mm/hugepages/hugepages-2048kB/nr_hugepages

sync

echo 3 | sudo tee /proc/sys/vm/drop_caches > /dev/null

echo -e "\n[SYSTEM] Environment correctly shut down!"