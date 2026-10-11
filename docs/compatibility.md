# Luchs preview compatibility

These are software targets of the public source baseline, not certified vehicle combinations. Actual results are recorded in the [development manual](luchs-development-manual.md).

| Software tree | Intended environment | Vehicle status |
| --- | --- | --- |
| `ros2/` | ROS 2 Humble on Ubuntu 22.04, Linux | Hardware and firmware acceptance pending |
| `ros2/` | ROS 2 Jazzy on Ubuntu 24.04, Linux | Hardware and firmware acceptance pending |
| `ros1/` | ROS 1 Noetic on Ubuntu 20.04, Linux | Legacy development target; hardware acceptance pending |
| `ros*/luchs_sdk/` | C++17, CMake, Linux SocketCAN; simulator without CAN hardware | Codec and simulator checks do not validate vehicle behavior |

Luchs A uses its own protocol and build options. Luchs B provides separate draft LCI and VCU V2.6 paths. A shared package does not make these protocols interchangeable. B example dimensions, limits and model values are provisional until checked against the actual variant.

ROS platform references: [Humble installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html), [Jazzy installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), [Noetic installation](https://wiki.ros.org/noetic/Installation/Ubuntu).
