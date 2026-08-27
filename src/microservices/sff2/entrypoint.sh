echo "[SYSTEM] Evaluating the \"C\" application for \"SFF2\"..."

# Compiles the "DPDK" program applying maximum optimization flags ( "-03" ) & linking modules natively via "gcc" / "pkg-config"
gcc -O3 /app/c/sff2.c -o /tmp/sff2_dpdk $(pkg-config --cflags --libs libdpdk)

if [ $? -ne 0 ]; then
    echo "[SYSTEM] Error: Compilation failed... Exiting!"
    exit 1
fi

echo -e "[SYSTEM] Communicating directly \"DPDK\"...\n"

/tmp/sff2_dpdk -l $DPDK_CORE -m 256 --file-prefix=sff2 --single-file-segments --no-pci \
  --vdev=net_vhost0,iface=/tmp/sfc-sff1-sff2,queues=1 \
  --vdev=net_vhost1,iface=/tmp/sfc-sff2-enc,queues=1 \
  --vdev=net_vhost2,iface=/tmp/sfc-sff2-dec,queues=1 \
  --vdev=net_vhost3,iface=/tmp/sfc-sff2-sff3,queues=1 \
  --log-level=*:alert