#!/bin/bash

echo "[SYSTEM] Evaluating the \"C\" application for \"SFF1\"..."

# Compiling the "DPDK" application with maximum optimization flag ( -O3 ), while "gcc" & "pkg-config" permits to link libraries natively
gcc -O3 /app/c/sff1.c -o /tmp/sff1_dpdk $(pkg-config --cflags --libs libdpdk)

if [ $? -ne 0 ]; then
    echo "[SYSTEM] Error: Compilation failed... Exiting!"
    exit 1
fi

echo -e "[SYSTEM] Communicating directly with \"OVS\"-\"DPDK\"...\n"

/tmp/sff1_dpdk -l $DPDK_CORE -m 512 --file-prefix=sff1 --single-file-segments --no-pci \
  --vdev=net_virtio_user0,path=/tmp/vh-sff1-in,server=1,queue_size=4096,mac=00:00:00:00:01:02 \
  --vdev=net_virtio_user1,path=/tmp/vh-sff1-eg,server=1,queue_size=1024,mac=00:00:00:00:02:01 \
  --log-level=*:alert