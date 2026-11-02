#!/bin/bash
# Holding /dev/cpu_dma_latency open at 0 blocks deep C-states, cutting wake-up latency.
set -euo pipefail

exec 3<>/dev/cpu_dma_latency
printf '\x00\x00\x00\x00' >&3
exec sleep infinity
