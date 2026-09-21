#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$repo_root/scripts/run.sh" pylon_demo_debris_orbit pylon_demo_debris_orbit.launch.py "$@"
