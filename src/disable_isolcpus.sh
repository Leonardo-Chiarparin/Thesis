set -euo pipefail

if [ "$EUID" -ne 0 ]; then
    echo "[SYSTEM] Error: Please run the script as \"root\" user..."
    exit 1
fi

GRUB_FILE="/etc/default/grub"

echo "[SYSTEM] Removing \"isolcpus=\" parameter from the \"GRUB\" file..."

sed -i -E 's/isolcpus=[0-9,-]+ ?//g' "$GRUB_FILE"

echo -e "[SYSTEM] Updating the environment...\n"

update-grub 2> >(grep -v -E 'os-prober will not be executed|Systems on them will not be added to the GRUB boot configuration|GRUB_DISABLE_OS_PROBER documentation entry' >&2)

echo -e "\n[SYSTEM] Operation completed. Reload the system to apply default scheduling..."