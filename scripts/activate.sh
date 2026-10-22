# Source this file (bash or zsh) to work in the Scrubnurse workspace:
#   source scripts/activate.sh
# Order matters: ROS first, then the venv (so `python3` is the venv's), then the overlay.

if [ -n "${ZSH_VERSION:-}" ]; then
  _sn_shell=zsh
  _sn_ws="$(cd "$(dirname "${(%):-%x}")/.." && pwd)"
else
  _sn_shell=bash
  _sn_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
fi

source "/opt/ros/${ROS_DISTRO:-humble}/setup.${_sn_shell}"
if [ -f "${_sn_ws}/.venv/bin/activate" ]; then
  source "${_sn_ws}/.venv/bin/activate"
else
  echo "No venv yet: run ${_sn_ws}/scripts/setup_venv.sh" >&2
fi
if [ -f "${_sn_ws}/install/setup.${_sn_shell}" ]; then
  source "${_sn_ws}/install/setup.${_sn_shell}"
fi
unset _sn_shell _sn_ws
