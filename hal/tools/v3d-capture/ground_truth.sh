#!/bin/sh
# Confirms the hardware/firmware constants src/v3d.rs hardcodes -- the
# V3D register base and the ENABLE_QPU mailbox tag -- against a Debian
# install running on a real Pi 3, rather than against documentation.
# Run on the Pi, not in this repo's dev environment. Prints best-effort
# findings; several steps are allowed to come up empty and say so rather
# than guessing.
set -e

echo "== V3D register range (live, from /proc/iomem) =="
if ! grep -i v3d /proc/iomem; then
    echo "not found -- try 'sudo modprobe vc4' then re-run this section"
fi

echo
echo "== V3D device-tree node (fallback if /proc/iomem was empty) =="
# Match the node's *directory name* (v3d@<addr>), not file content --
# grepping content also matches unrelated nodes that merely mention
# "v3d" (e.g. a watchdog node listing it as one of its clock names).
node=$(find /sys/firmware/devicetree/base -type d -iname 'v3d@*' 2>/dev/null | head -1 || true)
if [ -n "$node" ]; then
    echo "node: $node"
    # `printf`, not `echo -n`: this script runs under /bin/sh, where
    # echo's flags are not portable.
    printf 'reg property (raw bytes): '
    xxd "$node/reg" 2>/dev/null || od -An -tx1 "$node/reg"
else
    echo "no v3d device-tree node found under /sys/firmware/devicetree/base"
fi

echo
echo "== ENABLE_QPU mailbox tag (installed kernel headers) =="
hdr=$(find /usr/src -iname raspberrypi-firmware.h 2>/dev/null | head -1 || true)
if [ -n "$hdr" ]; then
    echo "found: $hdr"
    grep -i enable_qpu "$hdr" || echo "ENABLE_QPU not in this header -- check a cloned kernel tree instead"
else
    echo "not found under /usr/src -- clone the kernel tree instead:"
    echo "  git clone --depth 1 https://github.com/raspberrypi/linux rpi-linux"
    echo "  grep -rn ENABLE_QPU rpi-linux/include/soc/bcm2835/raspberrypi-firmware.h"
fi

echo
echo "== IDENT/vc4 debugfs (if available) =="
if [ -d /sys/kernel/debug/dri ]; then
    grep -rl v3d /sys/kernel/debug/dri 2>/dev/null || echo "no v3d debugfs entries found (may need sudo)"
else
    echo "/sys/kernel/debug/dri not mounted -- try: sudo mount -t debugfs none /sys/kernel/debug"
fi

echo
echo "== glxinfo/es2_info renderer string (confirms vc4 driver is live) =="
if command -v es2_info >/dev/null 2>&1; then
    es2_info 2>&1 | grep -i "GL_RENDERER\|GL_VERSION" || true
else
    echo "es2_info not installed -- 'sudo apt install mesa-utils-extra' or rely on shader_dump's own GL_RENDERER print"
fi
