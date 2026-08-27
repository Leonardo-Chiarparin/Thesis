echo "[SYSTEM] Evaluating the \"C\" application for \"SFF3\"..."

# Compiles the "DPDK" program applying maximum optimization flags ( "-03" ) & linking modules natively via "gcc" / "pkg-config"
gcc -O3 /app/c/sff3.c -o /tmp/sff3_dpdk $(pkg-config --cflags --libs libdpdk)

if [ $? -ne 0 ]; then
    echo "[SYSTEM] Error: Compilation failed... Exiting!"
    exit 1
fi

echo -e "[SYSTEM] Communicating directly with \"DPDK\"...\n"

/tmp/sff3_dpdk -l $DPDK_CORE -m 256 --file-prefix=sff3 --single-file-segments --no-pci \
  --vdev=net_virtio_user0,path=/tmp/sfc-sff2-sff3,queue_size=4096,mac=00:00:00:00:05:01 \
  --vdev=net_vhost1,iface=/tmp/sfc-sff3-usr,queues=1 \
  --log-level=*:alert