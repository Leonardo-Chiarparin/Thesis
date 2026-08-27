set -euo pipefail

if [ "$EUID" -ne 0 ]; then
    echo "[SYSTEM] Error: Please run the script as \"root\" user..."
    exit 1
fi

GRUB_FILE="/etc/default/grub"
ISOLATED_CPUS="1-7"

echo "[SYSTEM] Configuring \"isolcpus=$ISOLATED_CPUS\" within the \"GRUB\" file..."

if grep -q "isolcpus=" "$GRUB_FILE"; then
    sed -i -E "s/isolcpus=[0-9,-]+/isolcpus=$ISOLATED_CPUS/g" "$GRUB_FILE"
else
    sed -i "s/GRUB_CMDLINE_LINUX_DEFAULT=\"/GRUB_CMDLINE_LINUX_DEFAULT=\"isolcpus=$ISOLATED_CPUS /" "$GRUB_FILE"
fi

echo -e "[SYSTEM] Updating the environment...\n"

update-grub 2> >(grep -v -E 'os-prober will not be executed|Systems on them will not be added to the GRUB boot configuration|GRUB_DISABLE_OS_PROBER documentation entry' >&2)

echo -e "\n[SYSTEM] Operation completed. Reload the system to apply core isolation..."