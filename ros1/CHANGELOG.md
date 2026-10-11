# Changelog

> **Development preview 1.1.0-preview.1.** Adapted from the publicly distributed Luchs software v1.1.0 on 2026-10-11. Code/package versions remain 1.1.0 for source traceability. Tests supplied with the source are not evidence of successful execution or vehicle validation. Luchs B LCI is a draft interface; B configuration and firmware compatibility still require confirmation. See the [version development manual](../docs/luchs-development-manual.md).

All notable changes to luchs_ros1. Versioning follows luchs_ros2 (same luchs_sdk).

## Repository preview 1.1.0-preview.1 — 2026-10-11

- Preserves upstream package/software version 1.1.0 and implementation.
- Adds repository-aware workspace instructions and explicit draft/validation status.
- B configuration and launch default to simulation; hardware requires an explicit override after confirmation.
- Adds a self-contained ROS 1 driver parameter reference.

## Upstream 1.1.0 — 2026-10-05

First release of the ROS 1 (Noetic) software for the Luchs family.

- `luchs_sdk` 1.1.0, identical to luchs_ros2: Luchs A protocol v1.1, LCI v1.0 draft r3, Luchs B VCU upper interface V2.6; simulators; `luchs_cli`.
- `luchs_driver`: ROS 1 node with the same topics, services, safety behaviour and diagnostics as the ROS 2 driver; parameter files for the four Luchs A builds and Luchs B; rostests for Luchs A and Luchs B dry runs.
- `luchs_msgs`, `luchs_description`, `luchs_bringup` (`luchs_a.launch`, `luchs_b.launch`), `luchs_teleop`, `luchs` metapackage, DBC files and protocol documents.
- Replaces the earlier Luchs A ROS 1 reference driver (`chassis_can`): no compile-time configuration, host time sent, full motor fault words, command and feedback watchdogs, parameters per build.
