#!/bin/bash

echo "[SYSTEM] Evaluating the \"C\" application for \"SFF2\"..."

# Compiling the "DPDK" application with maximum optimization flag ( -O3 ), while "gcc" & "pkg-config" permits to link libraries natively
gcc -O3 /app/c/sff2.c -o /tmp/sff2_dpdk $(pkg-config --cflags --libs libdpdk)

if [ $? -ne 0 ]; then
    echo "[SYSTEM] Error: Compilation failed... Exiting!"
    exit 1
fi

echo -e "[SYSTEM] Communicating directly with \"OVS\"-\"DPDK\"...\n"

/tmp/sff2_dpdk -l $DPDK_CORE -m 512 --file-prefix=sff2 --single-file-segments --no-pci \
  --vdev=net_virtio_user0,path=/tmp/vh-sff2-sff1,server=1,queue_size=1024,mac=00:00:00:00:02:02 \
  --vdev=net_virtio_user1,path=/tmp/vh-sff2-enc,server=1,queue_size=4096,mac=00:00:00:00:03:01 \
  --vdev=net_virtio_user2,path=/tmp/vh-sff2-dec,server=1,queue_size=4096,mac=00:00:00:00:04:01 \
  --vdev=net_virtio_user3,path=/tmp/vh-sff2-sff3,server=1,queue_size=1024,mac=00:00:00:00:05:01 \
  --log-level=*:alert