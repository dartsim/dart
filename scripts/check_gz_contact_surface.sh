#!/usr/bin/env bash
# Fails unless a configured gz-physics build detected
# <dart/constraint/ContactSurface.hpp>. When the check fails, gz-physics's
# dartsim plugin silently drops SetContactPropertiesCallbackFeature, which
# breaks gz-sim's TrackController.
set -euo pipefail

build_dir="${1:?usage: check_gz_contact_surface.sh <gz-physics-build-dir>}"

if grep -qx 'DART_HAS_CONTACT_SURFACE_HEADER:INTERNAL=1' \
  "$build_dir/CMakeCache.txt" 2> /dev/null; then
  exit 0
fi

echo "gz-physics did not detect dart/constraint/ContactSurface.hpp" \
  "(DART_HAS_CONTACT_SURFACE_HEADER is not 1 in $build_dir/CMakeCache.txt)." >&2
echo "Its dartsim plugin would drop SetContactPropertiesCallbackFeature." >&2
log="$build_dir/CMakeFiles/CMakeConfigureLog.yaml"
if [ -f "$log" ]; then
  echo "Probe log from $log:" >&2
  # `|| true`: no match must not hide the message above under pipefail.
  grep -n -B2 -A16 'DART_HAS_CONTACT_SURFACE_HEADER' "$log" >&2 || true
fi
exit 1
