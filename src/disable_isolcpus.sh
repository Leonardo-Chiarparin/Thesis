#!/bin/bash

set -euo pipefail

if [ "$EUID" -ne 0 ]; then
  echo "[SYSTEM] Error: Please run the script as \"root\" user ( \"sudo\" )..."
  exit 1
fi

GRUB_FILE="/etc/default/grub"

echo "[SYSTEM] Removing \"isolcpus=\" parameter from the \"GRUB\" file..."

sed -i 's/ \?isolcpus=[0-9,-]* \?//g' "$GRUB_FILE"

echo -e "[SYSTEM] Updating the environment...\n"
update-grub

echo -e "\n[SYSTEM] Operation completed. Reload the system to apply default scheduling..."