#!/bin/bash
# isolcpus=managed_irq (see /proc/cmdline) only auto-excludes isolated CPUs from
# kernel-managed multi-queue IRQs when a driver requests fewer queues than CPUs.
# xhci_hcd's legacy vector and drivers that request one queue per CPU regardless
# (igc, iwlwifi, i915) aren't covered and were found pinned to isolated cores.
# Sweep every IRQ and force it onto the housekeeping cores; the kernel itself
# rejects this for genuinely managed IRQs (e.g. NVMe), so those are skipped safely.
set -uo pipefail

HOUSEKEEPING_CPUS="0-1"

for irq in $(awk -F: '{gsub(/ /, "", $1); print $1}' /proc/interrupts); do
    path="/proc/irq/${irq}/smp_affinity_list"
    [ -f "$path" ] || continue
    before=$(cat "$path")
    echo "${HOUSEKEEPING_CPUS}" > "$path" 2>/dev/null || continue
    after=$(cat "$path")
    [ "$before" = "$after" ] || echo "IRQ ${irq}: ${before} -> ${after}"
done
