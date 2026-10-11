# Get started with the Luchs preview

Clone the frozen `luchs-v1.1.0-preview.1` release snapshot as shown in the [repository README](../README.md). Review the [development manual](luchs-development-manual.md), then use either the [ROS 2 README](../ros2/README.md) or [ROS 1 README](../ros1/README.md) to copy only that software tree into your workspace.

Use the documented Linux and ROS distribution, install dependencies and run the supplied tests. Begin with `dry_run:=true`. Select the A build or the B protocol explicitly; A and B wire protocols are different. B LCI is a development draft whose availability depends on the exact vehicle firmware.

When reporting a result, include the repository commit, ROS distribution, operating system and architecture, selected build/protocol and whether transport was simulated, virtual CAN or physical CAN. Physical tests also need hardware and firmware versions. Publish only sanitized reproduction steps and logs.
