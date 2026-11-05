#!/usr/bin/env bash
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BENCH="${BENCH:-$HOME/comaps-bench}"
BUILD="${BUILD:-$REPO/../omim-build-release}"
ROUTES="${ROUTES:-$REPO/.plus/bench/routes.txt}"
LAUNCHES="${LAUNCHES:-1}"
STATES="${STATES:-Uttar Pradesh|Madhya Pradesh|Maharashtra|Rajasthan|Gujarat|Chhattisgarh|Telangana|Delhi|Haryana}"
SERVER="${SERVER:-https://mapgen-fi-1.comaps.app}"

cd "$REPO"
version=$(awk -F'[:,]' '/"v":/{ gsub(/ /, "", $2); print $2; exit }' data/countries.txt)
series=$(awk -F'"' '/"map_series":/{ print $4; exit }' data/countries.txt)
maps="$BENCH/maps/$version"
mkdir -p "$maps"

IFS='|' read -ra names <<< "$STATES"
for n in "${names[@]}"; do
  f="India_$n.mwm"
  if [ -s "$maps/$f" ]; then
    continue
  fi
  echo "downloading $f"
  curl -fL --retry 3 -o "$maps/$f.part" "$SERVER/maps/$series/$version/${f// /%20}"
  mv "$maps/$f.part" "$maps/$f"
done

if [ ! -f data/types.txt ]; then
  tools/unix/generate_drules.sh
fi

if [ ! -x "$BUILD/routes_builder_tool" ] || [ -n "${REBUILD:-}" ]; then
  tools/unix/build_omim.sh -r routes_builder_tool
fi

out="$BENCH/out-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$out"
ulimit -n 4096 2>/dev/null || true

TIME=()
if [ -x /usr/bin/time ]; then
  TIME=(/usr/bin/time -v -o "$out/time.txt")
fi

"${TIME[@]}" "$BUILD/routes_builder_tool" \
  --routes_file="$ROUTES" \
  --dump_path="$out/dump" \
  --data_path="$BENCH/maps" \
  --resources_path="$REPO/data" \
  --threads=1 \
  --timeout=3600 \
  --launches_number="$LAUNCHES" \
  --vehicle_type=car \
  --verbose 2>&1 | tee "$out/log.txt"

python3 "$REPO/.plus/bench/summary.py" "$out/log.txt" | tee "$out/summary.txt"
if [ -f "$out/time.txt" ]; then
  grep -E "Maximum resident|Elapsed" "$out/time.txt" | tee -a "$out/summary.txt"
fi
echo "log: $out/log.txt"
echo "summary: $out/summary.txt"
