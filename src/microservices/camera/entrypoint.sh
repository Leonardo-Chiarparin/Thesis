echo "[SYSTEM] Evaluating the \"C\" application for \"Camera\"..."

# Compiles the "DPDK" program applying maximum optimization flags ( "-03" ) & linking modules natively via "gcc" / "pkg-config"
gcc -O3 /app/c/camera.c -o /tmp/camera_dpdk $(pkg-config --cflags --libs libdpdk)

if [ $? -ne 0 ]; then
    echo "[SYSTEM] Error: Compilation failed... Exiting!"
    exit 1
fi

echo -e "[SYSTEM] Communicating directly with \"DPDK\"...\n"

/tmp/camera_dpdk -l $DPDK_CORE -m 256 --file-prefix=camera --single-file-segments --no-pci \
  --vdev=net_virtio_user0,path=/tmp/sfc-cam-sff1,queue_size=4096,mac=00:00:00:00:01:01 \
  --log-level=*:alert