# Luchs Version Development Manual

**Release: 1.1.0-preview.1. Date: 2026-10-11. Upstream software baseline: 1.1.0.**

This public preview provides readable sources and simulator development for Luchs A and B in separate ROS 1 and ROS 2 trees. A formal release for vehicle operation requires the firmware, parameter and physical test acceptance below.

## Version and scope

| Item | Version or location | Meaning |
| --- | --- | --- |
| Developer preview | `1.1.0-preview.1` | Curated public import, preview documentation and simulator defaults |
| Frozen release snapshot branch | `luchs-v1.1.0-preview.1` | Website links and Git acquisition refer to this snapshot; future revisions get a new preview name |
| ROS package metadata | `1.1.0` | Original source package version retained for provenance |
| ROS 1 tree | `ros1/` | Noetic and catkin workspace with a plain CMake SDK |
| ROS 2 tree | `ros2/` | Humble and Jazzy with colcon and ament |
| Website | V2.4 | Resource portal version, separate from the software version |

Record the release commit with `git rev-parse HEAD`. Build one ROS tree per workspace. Each contains seven packages: SDK, driver, messages, bringup, descriptions, teleoperation and the metapackage. Licenses and original import hashes are in the [public source manifest](luchs-public-sources.json).

Preview changes include working Git acquisition, an independent ROS 1 driver guide, clearer B draft status and `dry_run=true` defaults in B YAML and launch files. CAN identifiers, steering polarity, geometry and unresolved current encoding retain their source values; they have not been recalibrated for a vehicle.

## Software and protocol references

| Developer task | Public reference | Interpretation |
| --- | --- | --- |
| Build and start ROS 2 | [ROS 2 README](../ros2/README.md), [driver guide](../ros2/luchs_driver/README.md) | Use the matching ROS distribution and begin in the simulator |
| Build and start ROS 1 | [ROS 1 README](../ros1/README.md), [driver guide](../ros1/luchs_driver/README.md) | Use isolated or catkin tools build; ordinary catkin_make cannot build the plain CMake SDK |
| Integrate C++ | [SDK README](../ros2/luchs_sdk/README.md) | Codec, kinematics, SocketCAN client, CLI and simulators |
| Understand A frames | [A protocol](../ros2/docs/protocol_luchs_a.md) | Build and firmware details must match the vehicle |
| Understand B interfaces | [B protocol](../ros2/docs/protocol_luchs_b.md) | Draft LCI and legacy VCU V2.6 are distinct paths |
| Decode CAN in development | [DBC directory](../ros2/dbc/) | A DBC defines frame layouts and signals; it does not prove firmware implements them |
| Understand the robot model | [Description guide](../ros2/luchs_description/README.md) | B geometry, inertia and IMU placement are illustrative |

LCI means **Luchs Chassis Interface**, a proposed common chassis interface with heartbeat, commands, feedback and services. Its SDK and simulator implement development behavior. Firmware availability, accepted frames and physical response need separate confirmation. Legacy B VCU uses different commands and a host-side speed loop; it must not be substituted silently for LCI.

## Test record for this preview

Status is tied to the tested commit and environment. A supplied test is not a completed test. CI runs software checks with read-only repository permissions and no vehicle connection.

| Check | Current status | Evidence or remaining work |
| --- | --- | --- |
| Public import and original ZIP hashes | Checked locally | Manifest includes only the two already public ROS packages; private archive excluded |
| Python syntax, package XML and preview defaults | Passed locally | Python, XML, YAML, package metadata and preview defaults checked on 2026-10-11 |
| LCI DBC byte vectors | Passed locally | All 10 vectors pass in each tree with cantools 44.2.1 on 2026-10-11; draft software consistency only |
| Standalone C++ SDK tests for both trees | Pending CI | CMake with GTest, then ctest --output-on-failure --no-tests=error |
| ROS 2 Humble build and unit tests | Pending CI | Clean Linux container, rosdep, colcon build/test and result summary |
| ROS 2 Jazzy build and unit tests | Pending CI | Repeat in the matching Jazzy environment |
| ROS 1 Noetic build and A/B dry-run rostests | Pending CI | Noetic isolated build; record odometry, state and service results |
| ROS 2 launch, topics, services and RViz smoke tests | Pending integration testing | Start A/B separately with dry_run:=true; inspect runtime outputs and transforms |
| Physical SocketCAN and vehicle acceptance | Pending hardware testing | No physical A/B verification is claimed |

The workflow is [Luchs preview checks](../.github/workflows/luchs-preview.yml). Results appear in [GitHub Actions](https://github.com/erfinderwx/rj-platform-developer/actions). Passing CI changes only the software checks it actually covers, not vehicle acceptance.

## Remaining software and simulator tests

| Test group | Procedure and acceptance evidence |
| --- | --- |
| Clean installation | Clone the frozen ref into an empty workspace on each target OS/ROS pair; record dependency versions and build results |
| Package isolation | Build ros1/ and ros2/ independently; confirm seven intended packages without duplicate names in one workspace |
| A build selection | Run supported A configurations in dry-run; inspect model, topics, odometry, joint states and transforms |
| B interface selection | Exercise lci and legacy_b separately in dry-run; reject invalid names and detect model mismatches |
| Command and state behavior | Confirm units, axes, limits, state freshness, cmd_vel, odometry, battery, diagnostics and services |
| Simulator stop behavior | Test command silence, feedback loss, shutdown and e-stop requests; record timestamps and expected response |
| Robot model | Expand xacro variants; inspect RViz frames, joint directions, wheel motion and namespace/prefix combinations |
| Transport integration | Use Linux virtual CAN for send/receive, reconnect and malformed/stale frames before physical CAN |
| Documentation reproduction | A second developer follows the public README and quickstart in a clean environment; record missing steps |

Each result needs commit, OS, ROS distribution, architecture, command, result and sanitized logs. Resolve failures or list explicit release limitations.

## Luchs B parameters and protocol acceptance

These unresolved details are visible in the public source and must be closed for the hardware and firmware configuration of a formal release.

| Topic | Required confirmation and test |
| --- | --- |
| Firmware and interface | Record chassis/controller/firmware revisions; establish whether that unit implements draft LCI, VCU V2.6 or a revised interface; obtain a matching public specification |
| Geometry and steering | Validate wheelbase, track, effective wheel radius and mechanical steering range. B YAML example values 1.430 m, 0.970 m, 0.2794 m and 30 degrees are provisional |
| Legacy B frames | Verify 0x701 and 0x703 identifiers, standard/extended format, payload, byte order, scale, timing and example steer_sign=-1; verify the legacy 42 degree assumption |
| Speed loop | Validate command/feedback units, gearing, wheel direction, speed estimate and gains under the agreed load; set accepted speed/acceleration limits |
| LCI implementation | Confirm command/feedback IDs and DLC, checksum/alive counter, heartbeat timing, version/model fields and unsupported-frame behavior against vehicle captures |
| Stop and arbitration | Confirm host watchdog, command/feedback timeouts, remote-control priority, physical e-stop, brake/park and restart policy. The 200 ms draft heartbeat-loss detection threshold requires firmware measurement; total stopping time is a separate measured result |
| Limits and services | Confirm speed caps, steering/acceleration limits, payload power and e-stop requests; document rejection/reporting of unsupported commands |
| Current and battery | Resolve VCU DBC Current: unsigned 10-bit data with factor 1/offset 0 conflicts with a negative stated range. SDK does not decode this current field; it is not verified telemetry |
| IMU and model | Confirm sensor presence, frame, units, rate, orientation, pose and calibration; replace illustrative B mass/inertia/dimensions and placeholder IMU pose with approved values |
| Variant differences | Keep standard/cooling and other variants distinct where mechanical, electrical, protocol or performance values differ; record a compatibility matrix |

## Physical test sequence

1. **Bench and stationary:** confirm wiring, bitrate, termination, frame definitions and direction/units with motion constrained. Verify feedback, faults and each stop path.
2. **Low speed:** check forward/reverse travel, steering direction/limits, odometry, speed limits, command loss, remote-control handover and braking. Record response and agreed thresholds.
3. **Operating envelope:** test accepted loads, slopes, speed, turning, voltage/temperature and sustained operation using manufacturer-approved limits.
4. **Regression and recovery:** repeat stop/fault tests, bus interruption and power-cycle/restart behavior; confirm motion requires the intended deliberate re-enable.

Keep detailed vehicle logs internal when they contain identifiers or private information. Publish only approved, sanitized conclusions.

## Conditions for a formal release

| Requirement | Completion evidence |
| --- | --- |
| Fixed implementation | Immutable commit/tag and reproducible dependency versions, with no unresolved critical defect |
| Software checks | Required SDK, ROS builds, unit, integration and simulator tests pass for each supported combination |
| Accepted interface | Manufacturer-confirmed public protocol revision, accepted frame captures and documented unsupported functions |
| Accepted configuration | Approved A/B/variant parameters and robot model, with hardware/firmware compatibility records |
| Physical acceptance | Authorized results covering stop, control arbitration, limits and supported operating envelope |
| Public documentation | Accurate install/user/developer instructions, license, release notes, limitations and support procedure |
| Release packaging | Assign formal version after acceptance, update package metadata consistently and publish a new fixed reference |
| Website entry | Remove preview status only when the formal record is complete; preserve earlier previews and provenance |

Open items remain until evidence closes them. Keep the preview name visible while physical or protocol acceptance is pending. This preview assigns no future formal version number.
