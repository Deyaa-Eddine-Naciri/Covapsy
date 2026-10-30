#!/usr/bin/env bash
# Launch Webots with NVIDIA PRIME GPU offload enabled

export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD="${1:-${ROOT_DIR}/simulation/worlds/standalone.wbt}"

echo "[COVAPSY] Launching Webots with NVIDIA dGPU offload..."
echo "[COVAPSY] World: ${WORLD}"
exec webots "${WORLD}" "${@:2}"

