#!/usr/bin/env bash
# Standalone Linux SDK verification. A missing GTest/test target must fail CI.
set -euo pipefail

luchs_tree="${1:-}"
case "$luchs_tree" in
  ros1|ros2) ;;
  *) printf 'Usage: bash scripts/ci_sdk.sh ros1|ros2\n' >&2; exit 2 ;;
esac

luchs_repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
luchs_build="$(mktemp -d "${RUNNER_TEMP:-/tmp}/luchs-sdk-${luchs_tree}.XXXXXX")"
printf 'Standalone SDK source: %s; build: %s\n' "$luchs_tree" "$luchs_build"
cmake --version
ctest --version
cmake -S "$luchs_repo/$luchs_tree/luchs_sdk" -B "$luchs_build" \
  -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "$luchs_build" --parallel 2
ctest --test-dir "$luchs_build" --output-on-failure --no-tests=error
printf 'Standalone %s SDK build and CTest completed successfully.\n' "$luchs_tree"
