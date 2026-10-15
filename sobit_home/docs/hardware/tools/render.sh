#!/usr/bin/env bash
# Render every harness file to SVG with the shared device library.
# Needs graphviz (apt) and wireviz==0.4.1 (pip); WIREVIZ overrides the binary.
set -euo pipefail

cd "$(dirname "$0")/.."
WIREVIZ="${WIREVIZ:-wireviz}"
mkdir -p generated

for harness in harness/*.yml; do
  "$WIREVIZ" -f s -p library.yml -o generated "$harness"
done
