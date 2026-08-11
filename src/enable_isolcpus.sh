#!/bin/bash

set -euo pipefail

if [ "$EUID" -ne 0 ]; then
  echo "[SYSTEM] Error: Please run the script as \"root\" user ( \"sudo\" )..."
  exit 1
fi

GRUB_FILE="/etc/default/grub"

echo "[SYSTEM] Configuring \"isolcpus=1-7\" within the \"GRUB\" file..."

if grep -q "isolcpus=" "$GRUB_FILE"; then
    sed -i 's/isolcpus=[0-9,-]*/isolcpus=1-7/g' "$GRUB_FILE"
else
    sed -i 's/GRUB_CMDLINE_LINUX_DEFAULT="/GRUB_CMDLINE_LINUX_DEFAULT="isolcpus=1-7 /' "$GRUB_FILE"
fi

echo -e "[SYSTEM] Updating the environment...\n"
update-grub

echo -e "\n[SYSTEM] Operation completed. Reload the system to apply core isolation..."