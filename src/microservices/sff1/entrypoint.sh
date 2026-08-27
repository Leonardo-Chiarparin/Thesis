echo "[SYSTEM] Evaluating the \"C\" application for \"SFF1\"..."

# Compiles the "DPDK" program applying maximum optimization flags ( "-03" ) & linking modules natively via "gcc" / "pkg-config"
gcc -O3 /app/c/sff1.c -o /tmp/sff1_dpdk $(pkg-config --cflags --libs libdpdk) -lm

if [ $? -ne 0 ]; then
    echo "[SYSTEM] Error: Compilation failed... Exiting!"
    exit 1
fi

echo -e "[SYSTEM] Communicating directly with \"DPDK\"...\n"

/tmp/sff1_dpdk -l $DPDK_CORE -m 256 --file-prefix=sff1 --single-file-segments --no-pci \
  --vdev=net_vhost0,iface=/tmp/sfc-cam-sff1,queues=1 \
  --vdev=net_virtio_user1,path=/tmp/sfc-sff1-sff2,queue_size=4096,mac=00:00:00:00:02:01 \
  --log-level=*:alert