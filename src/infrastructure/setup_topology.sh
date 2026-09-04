set -euo pipefail

echo "[SYSTEM] Deleting the past instance..."

sudo rm -f /tmp/sfc-*
sudo rm -f /tmp/vh-*

echo -e "[SYSTEM] Crafting a single line topology ( \"RFC 7665\" )...\n"

echo "Camera <-> SFF1  /tmp/sfc-cam-sff1"
echo "SFF1 <-> SFF2    /tmp/sfc-sff1-sff2"
echo "SFF2 <-> Encoder /tmp/sfc-sff2-enc"
echo "SFF2 <-> Decoder /tmp/sfc-sff2-dec"
echo "SFF2 <-> SFF3    /tmp/sfc-sff2-sff3"
echo "SFF3 <-> User    /tmp/sfc-sff3-usr"

echo -e "\n[SYSTEM] Ready to go!"