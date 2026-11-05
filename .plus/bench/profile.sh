#!/usr/bin/env bash
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BENCH="${BENCH:-$HOME/comaps-bench}"
BUILD="${BUILD:-$REPO/../omim-build-release}"
ROUTE="${ROUTE:-23.2599 77.4126 18.5308 73.8475}"

out="$BENCH/profile-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$out"
echo "$ROUTE" > "$out/route.txt"

perf record -F 999 -o "$out/perf.data" -- "$BUILD/routes_builder_tool" \
  --routes_file="$out/route.txt" \
  --dump_path="$out/dump" \
  --data_path="$BENCH/maps" \
  --resources_path="$REPO/data" \
  --threads=1 \
  --timeout=3600 \
  --vehicle_type=car

perf report -i "$out/perf.data" --no-children --sort symbol --percent-limit 0.5 --stdio 2>/dev/null \
  | grep -v '^$' | head -80 > "$out/profile.txt"
echo "profile: $out/profile.txt"
