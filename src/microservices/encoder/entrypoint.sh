#!/bin/bash

echo "[SYSTEM] Evaluating the \"C++\" application for \"Encoder\"..."

cd /app

make clean
make all

if [ $? -ne 0 ]; then
    echo "[SYSTEM] Error: Compilation failed... Exiting!"
    exit 1
fi

echo -e "[SYSTEM] Communicating directly with \"OVS\"-\"DPDK\"...\n"

/tmp/encoder_dpdk -l $DPDK_CORE -m 512 --file-prefix=encoder --single-file-segments --no-pci \
  --vdev=net_virtio_user0,path=/tmp/vh-enc,server=1,queue_size=4096,mac=00:00:00:00:03:02 \
  --log-level=*:alert