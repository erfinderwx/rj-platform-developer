#!/usr/bin/env bash
# Run in ros:humble-ros-base or ros:jazzy-ros-base with /repo mounted read-only.
set -euo pipefail

case "${ROS_DISTRO:-}" in
  humble|jazzy) ;;
  *) printf 'This check requires the Humble or Jazzy official ROS image.\n' >&2; exit 2 ;;
esac

luchs_repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
luchs_workspace="$(mktemp -d "/tmp/luchs-ros2-${ROS_DISTRO}.XXXXXX")"
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --no-install-recommends \
  build-essential cmake libgtest-dev python3-colcon-common-extensions python3-rosdep

# ROS setup scripts may legitimately inspect unset variables.
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  rosdep init
fi
rosdep update --rosdistro "$ROS_DISTRO"
mkdir -p "$luchs_workspace/src"
cp -a "$luchs_repo/ros2" "$luchs_workspace/src/luchs_ros2"
cd "$luchs_workspace"
printf 'Building ROS 2 %s in independent workspace %s\n' "$ROS_DISTRO" "$luchs_workspace"
rosdep install --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" -y
colcon list
colcon build --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=ON
set +u
source install/setup.bash
set -u

luchs_test_exit=0
if colcon test --packages-select luchs_sdk luchs_driver \
  --event-handlers console_direct+ --return-code-on-test-failure; then
  :
else
  luchs_test_exit=$?
fi
colcon test-result --verbose

# A nominal success with no executed SDK/driver test cases is not a pass.
python3 - <<'PY'
from pathlib import Path
import xml.etree.ElementTree as ET

for package in ('luchs_sdk', 'luchs_driver'):
    result_dir = Path('build') / package / 'test_results'
    cases = 0
    for result in result_dir.rglob('*.xml'):
        root = ET.parse(result).getroot()
        for case in root.iter('testcase'):
            if case.find('skipped') is None and case.get('status') != 'notrun':
                cases += 1
    if cases == 0:
        raise SystemExit(f'{package}: no executed test cases found in {result_dir}')
    print(f'{package}: {cases} executed test cases recorded')
PY

if [[ "$luchs_test_exit" -ne 0 ]]; then
  exit "$luchs_test_exit"
fi
printf 'ROS 2 %s build and supplied SDK/driver tests completed successfully.\n' "$ROS_DISTRO"
