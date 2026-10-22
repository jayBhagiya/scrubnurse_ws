#!/usr/bin/env bash
# Create (or update) the workspace Python virtual environment in .venv.
#
# Follows the ROS 2 guide "Using Python packages with ROS 2": ROS and apt
# packages stay system-wide (install them with rosdep), the venv only adds the
# pip-only packages from requirements.txt on top via --system-site-packages.
# A venv contains absolute paths and is never copied between machines; run this
# script again on each machine instead.
#
# PyTorch is installed in the build that fits this machine's GPU:
#   cu128  Blackwell GPUs (RTX 50xx), driver >= 570.26
#   cu126  older NVIDIA GPUs, driver >= 560.28
#   cpu    no usable NVIDIA GPU (YOLO and Whisper still run, only slower)
# Override with TORCH_BUILD=cu128|cu126|cpu ./scripts/setup_venv.sh
#
# The Azure Kinect camera node needs pyk4a, which builds against the Azure
# Kinect SDK; it is installed only where the SDK headers are present.
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${WS_DIR}/.venv"
PIP=("${VENV_DIR}/bin/python" -m pip)

version_ge() {  # version_ge A B: true if A >= B
  [[ "$(printf '%s\n%s\n' "$1" "$2" | sort -V | head -1)" == "$2" ]]
}

detect_torch_build() {
  if ! nvidia-smi > /dev/null 2>&1; then
    echo cpu
    return
  fi
  local driver compute
  driver="$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -1)"
  compute="$(nvidia-smi --query-gpu=compute_cap --format=csv,noheader | head -1)"
  if version_ge "${compute}" 10.0; then
    # Blackwell only runs with the CUDA 12.8 builds.
    if version_ge "${driver}" 570.26; then echo cu128; else echo cpu; fi
  elif version_ge "${driver}" 560.28; then
    echo cu126
  else
    echo cpu
  fi
}

if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
  echo "Creating ${VENV_DIR}"
  python3 -m venv --system-site-packages "${VENV_DIR}"
fi
# Keep colcon from crawling the venv's site-packages.
touch "${VENV_DIR}/COLCON_IGNORE"
"${PIP[@]}" install --upgrade pip

# PyTorch first, from the index of the chosen build; requirements.txt pins the version.
TORCH_BUILD="${TORCH_BUILD:-$(detect_torch_build)}"
TORCH_PINS=($(grep -E '^(torch|torchvision)==' "${WS_DIR}/requirements.txt"))
TORCH_VERSION="${TORCH_PINS[0]#torch==}"
INSTALLED="$("${VENV_DIR}/bin/python" -c 'import torch; print(torch.__version__)' 2>/dev/null || true)"
echo "PyTorch build: ${TORCH_BUILD} (installed: ${INSTALLED:-none})"
if [[ -n "${INSTALLED}" && "${INSTALLED}" != "${TORCH_VERSION}+${TORCH_BUILD}" ]]; then
  "${PIP[@]}" uninstall -y torch torchvision
fi
"${PIP[@]}" install "${TORCH_PINS[@]}" --index-url "https://download.pytorch.org/whl/${TORCH_BUILD}"

"${PIP[@]}" install -r "${WS_DIR}/requirements.txt"

if [[ -f /usr/include/k4a/k4a.h ]]; then
  "${PIP[@]}" install -r "${WS_DIR}/requirements-camera.txt"
else
  echo "Azure Kinect SDK not found: skipping pyk4a (only needed on the PC with the camera)."
fi
echo "Done. Activate with: source ${WS_DIR}/scripts/activate.sh"
