#!/usr/bin/env bash
# Build the amd64 Jazzy navigation image.
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
exec docker compose -f "$root/docker/compose.yaml" build "$@" closed-loop
