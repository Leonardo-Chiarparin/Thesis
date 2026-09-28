echo "[SYSTEM] Evaluating the \"C++\" application for \"Encoder\"..."

cd /app

# Compiles the "DPDK" program applying maximum optimisation flags ( "-O3" ) & linking modules natively via "g++" / "pkg-config"
make clean
make all

if [ $? -ne 0 ]; then
    echo "[SYSTEM] Error: Compilation failed... Exiting!"
    exit 1
fi

echo -e "[SYSTEM] Communicating directly with \"DPDK\"...\n"

/tmp/encoder_dpdk -l $DPDK_CORE -m 256 --file-prefix=encoder --single-file-segments --no-pci \
  --vdev=net_virtio_user0,path=/tmp/sfc-sff2-enc,queue_size=4096,mac=00:00:00:00:03:01 \
  --log-level=*:alert