#!/usr/bin/env bash
# Build in ros:noetic-ros-base and execute both supplied simulator rostests.
set -euo pipefail

if [[ "${ROS_DISTRO:-}" != noetic ]]; then
  printf 'This check requires the official ROS Noetic image.\n' >&2
  exit 2
fi

luchs_repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
luchs_workspace="$(mktemp -d /tmp/luchs-ros1-noetic.XXXXXX)"
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --no-install-recommends \
  build-essential cmake libgtest-dev python3-rosdep ros-noetic-rostest ros-noetic-rostopic
set +u
source /opt/ros/noetic/setup.bash
set -u
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  rosdep init
fi
# Noetic is an EOL distribution; include it rather than silently omitting its rules.
rosdep update --rosdistro noetic --include-eol-distros
mkdir -p "$luchs_workspace/src"
cp -a "$luchs_repo/ros1" "$luchs_workspace/src/luchs_ros1"
cd "$luchs_workspace"
rosdep install --from-paths src --ignore-src --rosdistro noetic -y
printf 'Building ROS 1 Noetic in independent workspace %s\n' "$luchs_workspace"
catkin_make_isolated --install --cmake-args \
  -DCATKIN_ENABLE_TESTING=ON -DBUILD_TESTING=ON
set +u
source install_isolated/setup.bash
set -u

# The upstream driver does not install its test helper. Stage the unchanged helper
# in this disposable workspace's install prefix so roslaunch can resolve it.
luchs_test_source="$luchs_workspace/src/luchs_ros1/luchs_driver/test"
chmod +x "$luchs_test_source/drive_check.py"
mkdir -p "$luchs_workspace/install_isolated/lib/luchs_driver"
install -m 755 "$luchs_test_source/drive_check.py" \
  "$luchs_workspace/install_isolated/lib/luchs_driver/drive_check.py"
luchs_results="$luchs_workspace/test_results"
mkdir -p "$luchs_results"
export ROS_TEST_RESULTS_DIR="$luchs_results"
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost

luchs_test_exit=0
for luchs_model in a b; do
  printf 'Running supplied Luchs %s simulator rostest\n' "$luchs_model"
  if rostest --pkgdir "$luchs_workspace/src/luchs_ros1/luchs_driver" \
    --package luchs_driver --results-base-dir "$luchs_results" \
    "$luchs_test_source/dry_run_${luchs_model}.test"; then
    :
  else
    luchs_test_exit=1
  fi
done
catkin_test_results --all --verbose "$luchs_results"

# Require the actual drive tests for both models, rather than only launch XML.
python3 - "$luchs_results" <<'PY'
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

results = Path(sys.argv[1])
for model in ('a', 'b'):
    candidates = list(results.rglob(f'rosunit-drive_{model}.xml'))
    cases = sum(
        1 for result in candidates for case in ET.parse(result).getroot().iter('testcase')
        if case.find('skipped') is None and case.get('status') != 'notrun'
    )
    if cases == 0:
        raise SystemExit(f'Luchs {model}: no executed drive_check result found')
    print(f'Luchs {model}: {cases} supplied drive test cases recorded')
PY

if [[ "$luchs_test_exit" -ne 0 ]]; then
  exit "$luchs_test_exit"
fi
printf 'ROS 1 Noetic build and both supplied simulator rostests completed successfully.\n'
