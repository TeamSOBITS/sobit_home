#!/bin/bash
# One-time host RT setup for the NUC. See ../REALTIME_SETUP.md. Usage: sudo bash setup.sh
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
    echo "Run this with sudo: sudo bash $0" >&2
    exit 1
fi

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "==> Installing scripts to /usr/local/sbin"
install -m 755 "$DIR/sobit-irq-affinity.sh"    /usr/local/sbin/sobit-irq-affinity.sh
install -m 755 "$DIR/sobit-cpu-dma-latency.sh" /usr/local/sbin/sobit-cpu-dma-latency.sh

echo "==> Installing systemd units"
install -m 644 "$DIR/sobit-irq-affinity.service"    /etc/systemd/system/sobit-irq-affinity.service
install -m 644 "$DIR/sobit-cpu-performance.service" /etc/systemd/system/sobit-cpu-performance.service
install -m 644 "$DIR/sobit-cpu-dma-latency.service" /etc/systemd/system/sobit-cpu-dma-latency.service

echo "==> Installing sysctl config (DDS UDP buffers)"
install -m 644 "$DIR/60-sobit-dds.conf" /etc/sysctl.d/60-sobit-dds.conf
sysctl --system > /dev/null

echo "==> Enabling and starting services"
systemctl daemon-reload
systemctl enable --now sobit-irq-affinity.service
systemctl enable --now sobit-cpu-performance.service
systemctl enable --now sobit-cpu-dma-latency.service

echo
echo "==> Done. Current state:"
echo "--- xhci_hcd IRQ affinity ---"
awk -F: '/xhci_hcd/ {gsub(/ /, "", $1); print $1}' /proc/interrupts | sort -u | while read -r irq; do
    printf 'IRQ %s -> %s\n' "$irq" "$(cat "/proc/irq/${irq}/smp_affinity_list")"
done
echo "--- power profile ---"
powerprofilesctl get
echo "--- cpu_dma_latency holder ---"
systemctl is-active sobit-cpu-dma-latency.service
echo "--- DDS UDP buffers ---"
sysctl net.core.rmem_max net.core.wmem_max
