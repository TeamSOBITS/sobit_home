#!/usr/bin/env bash
# Render every harness file, plus all of them merged into generated/all.svg.
# Needs graphviz (apt) and wireviz==0.4.1 (pip); WIREVIZ overrides the binary.
set -euo pipefail

cd "$(dirname "$0")/.."
WIREVIZ="${WIREVIZ:-wireviz}"
mkdir -p generated

for harness in harness/*.yml; do
  "$WIREVIZ" -f s -p library.yml -o generated "$harness"
done

merged="$(mktemp -d)/all.yml"
python3 - "$merged" harness/*.yml <<'EOF'
import sys
cables, connections = [], []
for path in sys.argv[2:]:
    head, body = open(path).read().split("\nconnections:\n")
    cables += [l for l in head.split("\ncables:\n")[1].splitlines() if l.strip()]
    connections += [l for l in body.splitlines() if l.strip()]
open(sys.argv[1], "w").write("cables:\n" + "\n".join(cables) + "\n\nconnections:\n" + "\n".join(connections) + "\n")
EOF
"$WIREVIZ" -f s -p library.yml -o generated "$merged"
rm -r "$(dirname "$merged")"
