set -euo pipefail

echo "[SYSTEM] Deleting the past instance..."

sudo rm -f /tmp/sfc-*
sudo rm -f /tmp/vh-*

echo -e "[SYSTEM] Crafting a single line topology ( \"RFC 7665\" )...\n"

echo -e "Camera <-> SFF1 \"/tmp/sfc-cam-sff1\""
echo -e "SFF1 <-> SFF2 \"/tmp/sfc-sff1-sff2\""
echo -e "SFF2 <-> Encoder \"/tmp/sfc-sff2-enc\""
echo -e "SFF2 <-> Decoder \"/tmp/sfc-sff2-dec\""
echo -e "SFF2 <-> SFF3 \"/tmp/sfc-sff2-sff3\""
echo -e "SFF3 <-> User \"/tmp/sfc-sff3-usr\""

echo -e "\n[SYSTEM] Ready to go!"