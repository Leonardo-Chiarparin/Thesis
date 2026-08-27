set -e

QUALITY_READY="/tmp/sfc-user-quality"
QUALITY_DONE="/tmp/sfc-user-done"

QUALITY_CAPTURE="${QUALITY_CAPTURE:-0}"

WEB_PID=""
QUALITY_PID=""

rm -f "$QUALITY_READY" "$QUALITY_DONE"

echo -e "[SYSTEM] Evaluating the \"C\" application for \"User\"..."

# Compiles the "DPDK" program applying maximum optimization flags ( "-O3" ) & linking modules natively via "gcc" / "pkg-config"
gcc -O3 /app/c/user.c -o /tmp/user_dpdk $(pkg-config --cflags --libs libdpdk) -lm

if [ "$QUALITY_CAPTURE" = "0" ]; then
  echo -e "\n[SYSTEM] Opening the asynchronous web bridge...\n"
  nice -n 5 taskset -c 2 python3 /app/py/user.py --http "${HTTP_PORT:-8080}" --ws "${WS_PORT:-9999}" --root /app/html &
  WEB_PID=$!
fi

if [ "$QUALITY_CAPTURE" = "1" ]; then
  echo -e "\n[SYSTEM] Planning geometric analysis...\n"
  (
    while [ ! -f "$QUALITY_READY" ]; do
      sleep 0.5
      
    done

    OMP_NUM_THREADS=1 \
    OPENBLAS_NUM_THREADS=1 \
    MKL_NUM_THREADS=1 \
    NUMEXPR_NUM_THREADS=1 \
    taskset -c $DPDK_CORE python3 /shared/py/gauge/gauge.py \
      --telemetry /shared/log/user/telemetry_user.csv \
      --capture /shared/data/loot/made/results.bin \
      --reference "${REFERENCE_DIR:-/shared/data/loot/bin}" \
      --ready "$QUALITY_READY" \
      --done "$QUALITY_DONE"
  ) &
  QUALITY_PID=$!
fi

cleanup() {
  if [ -n "$WEB_PID" ] && kill -0 "$WEB_PID" 2>/dev/null; then
    kill "$WEB_PID" 2>/dev/null || true
    wait "$WEB_PID" 2>/dev/null || true
  fi

  if [ -n "$QUALITY_PID" ] && kill -0 "$QUALITY_PID" 2>/dev/null; then
    kill "$QUALITY_PID" 2>/dev/null || true
    wait "$QUALITY_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo -e "[SYSTEM] Communicating directly with \"DPDK\"...\n"

/tmp/user_dpdk -l $DPDK_CORE -m 256 --file-prefix=user --single-file-segments --no-pci \
  --vdev=net_virtio_user0,path=/tmp/sfc-sff3-usr,queue_size=4096,mac=00:00:00:00:06:01 \
  --log-level=*:alert