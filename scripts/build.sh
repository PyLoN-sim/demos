#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
pylon_root="${PYLON_DIR:-$repo_root/../PyLoN}"
demo="${1:?Usage: build.sh DEMO [sync options]}"
shift
[[ -x "$pylon_root/sync.sh" ]] || { echo "Clone PyLoN-sim/PyLoN beside demos or set PYLON_DIR." >&2; exit 1; }
export PYLON_DEMOS_DIR="$repo_root"
exec "$pylon_root/sync.sh" --skip-ksp-build --skip-ksp-sync --demo "$demo" "$@"
