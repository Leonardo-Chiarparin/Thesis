#!/bin/bash

echo "[SYSTEM] Evaluating the \"C\" application for \"Camera\"..."

# Compiling the "DPDK" application with maximum optimization flag ( -O3 ), while "gcc" & "pkg-config" permits to link libraries natively
gcc -O3 /app/c/camera.c -o /tmp/camera_dpdk $(pkg-config --cflags --libs libdpdk)

if [ $? -ne 0 ]; then
    echo "[SYSTEM] Error: Compilation failed... Exiting!"
    exit 1
fi

echo -e "[SYSTEM] Communicating directly with \"OVS\"-\"DPDK\"...\n"

/tmp/camera_dpdk -l $DPDK_CORE -m 4096 --file-prefix=camera --single-file-segments --no-pci \
  --vdev=net_virtio_user0,path=/tmp/vh-cam,server=1,queue_size=4096,mac=00:00:00:00:01:01 \
  --log-level=*:alert