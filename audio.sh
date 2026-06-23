#!/usr/bin/env bash
#
# audio.sh - Route PC-B's (172.16.10.80) speaker/mic to PC-A
#
# Usage:
#   bash audio.sh start    # switch PC-A defaults to PC-B's audio devices
#   bash audio.sh stop     # revert to PC-A's local devices
#   bash audio.sh status   # show current state
#
# Notes:
#   - SSH to PC-B (rg-sobit-home@172.16.10.80) still uses password auth
#   - No sshpass; plain ssh is used, password prompt appears once during start
#
set -euo pipefail

# ===== Config (fixed) =====
PCB_USER="rg-sobit-home"
PCB_HOST="172.16.10.80"
PCB_SSH="${PCB_USER}@${PCB_HOST}"

PCB_SINK="alsa_output.usb-0b0e_Jabra_Speak_710_08C8C23A2C5F-00.analog-stereo"
PCB_SOURCE="alsa_input.usb-R__DE_R__DE_VideoMic_GO_II_7503A158-00.mono-fallback"

LOCAL_SINK_NAME="jabra_tunnel_sink"
LOCAL_SOURCE_NAME="rode_tunnel_source"

PCA_FALLBACK_SINK="alsa_output.pci-0000_00_1f.3-platform-skl_hda_dsp_generic.HiFi__hw_sofhdadsp__sink"
PCA_FALLBACK_SOURCE="alsa_input.pci-0000_00_1f.3-platform-skl_hda_dsp_generic.HiFi__hw_sofhdadsp_6__source"

# ===== Colors =====
COLOR_OK="\033[32m"    # green
COLOR_FAIL="\033[31m"  # red
COLOR_WARN="\033[33m"  # yellow
COLOR_RESET="\033[0m"

ok()   { echo -e "${COLOR_OK}OK${COLOR_RESET} $1"; }
fail() { echo -e "${COLOR_FAIL}FAIL${COLOR_RESET} $1"; }
warn() { echo -e "${COLOR_WARN}WARN${COLOR_RESET} $1"; }

# ===== Internal functions =====

pcb_ssh() {
  local remote_cmd="$1"
  ssh -o StrictHostKeyChecking=accept-new "$PCB_SSH" "$remote_cmd"
}

unload_existing_tunnels() {
  local mod_id

  while read -r mod_id; do
    [ -n "$mod_id" ] && pactl unload-module "$mod_id" 2>/dev/null || true
  done < <(pactl list modules short | grep "module-tunnel-sink" | awk '{print $1}')

  while read -r mod_id; do
    [ -n "$mod_id" ] && pactl unload-module "$mod_id" 2>/dev/null || true
  done < <(pactl list modules short | grep "module-tunnel-source" | awk '{print $1}')
}

# ===== start =====
do_start() {
  if pcb_ssh '
    if ! pactl list modules short | grep -q module-native-protocol-tcp; then
      pactl load-module module-native-protocol-tcp auth-anonymous=1
    fi
  '; then
    ok "PC-B network module ready"
  else
    fail "could not reach or configure PC-B over SSH"
    exit 1
  fi

  if nc -z -w3 "$PCB_HOST" 4713; then
    ok "PC-B reachable on port 4713"
  else
    fail "cannot connect to ${PCB_HOST}:4713 (check firewall)"
    exit 1
  fi

  unload_existing_tunnels

  if pactl load-module module-tunnel-sink \
      server="tcp:${PCB_HOST}" \
      sink="${PCB_SINK}" \
      sink_name="${LOCAL_SINK_NAME}" >/dev/null; then
    ok "tunnel sink loaded (${LOCAL_SINK_NAME})"
  else
    fail "failed to load tunnel sink"
    exit 1
  fi

  if pactl load-module module-tunnel-source \
      server="tcp:${PCB_HOST}" \
      source="${PCB_SOURCE}" \
      source_name="${LOCAL_SOURCE_NAME}" >/dev/null; then
    ok "tunnel source loaded (${LOCAL_SOURCE_NAME})"
  else
    fail "failed to load tunnel source"
    exit 1
  fi

  if pactl set-default-sink "${LOCAL_SINK_NAME}" && pactl set-default-source "${LOCAL_SOURCE_NAME}"; then
    ok "default sink/source set to PC-B devices"
  else
    fail "failed to set default sink/source"
    exit 1
  fi
}

# ===== stop =====
do_stop() {
  unload_existing_tunnels
  ok "tunnel sink/source unloaded"

  if pactl list short sinks | grep -q "${PCA_FALLBACK_SINK}"; then
    pactl set-default-sink "${PCA_FALLBACK_SINK}"
    ok "default sink reverted to PC-A local device"
  else
    warn "fallback sink (${PCA_FALLBACK_SINK}) not found, check manually"
  fi

  if pactl list short sources | grep -q "${PCA_FALLBACK_SOURCE}"; then
    pactl set-default-source "${PCA_FALLBACK_SOURCE}"
    ok "default source reverted to PC-A local device"
  else
    warn "fallback source (${PCA_FALLBACK_SOURCE}) not found, check manually"
  fi
}

# ===== status =====
do_status() {
  echo "default sink:   $(pactl get-default-sink)"
  echo "default source: $(pactl get-default-source)"
  echo
  if pactl list modules short | grep -q "module-tunnel-"; then
    pactl list modules short | grep "module-tunnel-"
  else
    echo "no tunnel modules loaded"
  fi
}

# ===== main =====
case "${1:-}" in
  start)
    do_start
    ;;
  stop)
    do_stop
    ;;
  status)
    do_status
    ;;
  *)
    echo "usage: bash audio.sh {start|stop|status}" >&2
    exit 1
    ;;
esac
