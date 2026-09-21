#!/usr/bin/env bash
# Run PyLoN against a separate, pinned Space ROS installation on Linux.
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
pylon_root="${PYLON_DIR:-$repo_root/../PyLoN}"
image="${PYLON_SPACEROS_IMAGE:-pylon-spaceros-reusable:jazzy-2026.07.0}"
container="${PYLON_SPACEROS_CONTAINER:-pylon-reusable}"
usage() {
    cat <<'USAGE'
Usage: ./pylon_demo_reusable/spaceros.sh COMMAND [arguments]
  build             Build the core image and the reusable demo overlay.
  run [bridge args] Start the bridge (Ctrl+C stops it).
  demo [args]       Run reusable launch demo and bridge; configure then activate via lifecycle.
  shell             Open a new Space ROS shell.
  exec COMMAND ...  Run a command in the running bridge container.
  test              Run bridge, vehicle control and reusable mission tests in Space ROS.
  stop              Stop the running bridge container.
Environment: PYLON_SPACEROS_IMAGE, PYLON_SPACEROS_CONTAINER, ROS_DOMAIN_ID,
             ROS_AUTOMATIC_DISCOVERY_RANGE (default LOCALHOST), PYLON_DIR, PYLON_CORE_IMAGE.
Rebuild after changing ROS sources. Host ~/ros2_ws is not used by the image.
USAGE
}
action="${1:-help}"
if (($#)); then shift; fi
case "$action" in
    help|-h|--help) usage; exit 0;;
    build|run|demo|shell|exec|test|stop) ;;
    *) usage >&2; exit 2;;
esac
command -v docker >/dev/null || { echo 'Docker is required. See https://github.com/PyLoN-sim/docs/blob/main/guide/space-ros.md.' >&2; exit 1; }
interactive=()
if [[ -t 0 && -t 1 ]]; then interactive=(-it); fi
runtime=(--rm --init --network host
    -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
    -e "ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}")
case "$action" in
    build)
        PYLON_SPACEROS_IMAGE="${PYLON_CORE_IMAGE:-pylon-spaceros-core:jazzy-2026.07.0}" "$pylon_root/spaceros.sh" build
        docker build -f "$repo_root/pylon_demo_reusable/Dockerfile" \
            --build-arg "PYLON_CORE_IMAGE=${PYLON_CORE_IMAGE:-pylon-spaceros-core:jazzy-2026.07.0}" -t "$image" "$@" "$repo_root";;
    run)
        exec docker run "${runtime[@]}" "${interactive[@]}" --name "$container" "$image" \
            ros2 run pylon_bridge udp_bridge --host 127.0.0.1 "$@";;
    shell)
        exec docker run "${runtime[@]}" "${interactive[@]}" "$image" bash --norc;;
    demo)
        exec docker run "${runtime[@]}" "${interactive[@]}" --name "$container" "$image" \
            ros2 launch pylon_demo_reusable demo.launch.py "$@";;
    exec)
        (($#)) || { echo 'exec requires a command' >&2; exit 2; }
        exec docker exec "${interactive[@]}" "$container" /pylon-demo-entrypoint.sh "$@";;
    test)
        # Use a private container network so test nodes cannot reach a running game.
        exec docker run --rm --init "$image" bash -ec '
            python3 -m pytest -q /home/spaceros-user/pylon_ws/src/pylon_bridge/test
            python3 -m pytest -q /home/spaceros-user/pylon_ws/src/pylon_vehicle_control/test
            python3 -m pytest -q src/pylon_demo_reusable/test
        ';;
    stop) exec docker stop "$container";;
esac
