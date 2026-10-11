# luchs_description

> **Development preview 1.1.0-preview.1.** Adapted from the publicly distributed Luchs software v1.1.0 on 2026-10-11. Code/package versions remain 1.1.0 for source traceability. Tests supplied with the source are not evidence of successful execution or vehicle validation. Luchs B LCI is a draft interface; B configuration and firmware compatibility still require confirmation. See the [version development manual](../../docs/luchs-development-manual.md).

Robot description (URDF/xacro, meshes, RViz launch) for the Luchs family: the **Luchs A** builds and **Luchs B**. Both use the same frame and joint names.
Chassis by RJ Tech Limited, software by TerraAgent Robotics. ROS 2 Humble / Jazzy, `ament_cmake`.

| `config` | Drive | Envelope L x W x H (mm) | Wheelbase / track (mm) | Clearance (mm) | Mass (kg) | Steering joints |
|---|---|---|---|---|---|---|
| `df`  | skid-steer 4WD | 800 x 665 x 292 | 594 / 596 | 77 | 40 | none (fixed) |
| `fa`  | front Ackermann 4WD | 800 x 665 x 292 | 594 / 596 | 77 | 45 | front, +/-0.6 rad |
| `da`  | dual Ackermann 4WD | 800 x 665 x 292 | 594 / 596 | 77 | 50 | front + rear, +/-0.6 rad |
| `4ws` | four-wheel independent steering, swing-arm suspension | 800 x 580 x 372 | 594 / 500 | 220 | 55 | all four, +/-pi/2 (to be confirmed) |

All builds use 206 mm wheels and two 2040 aluminium rails on the deck, 200 mm apart (centre to centre), running fore-aft.
The +/-0.6 rad Ackermann limit is consistent with the published minimum turning radii
(0.594 / tan 0.6 = 0.87 m for `fa`, half of that for `da`).

Only the `4ws` build has CAD. `df`, `fa` and `da` are drawn with boxes and cylinders of the published
envelope until their CAD is available.

## Frames

```
base_footprint                 ground plane, centred between the axles and between the wheels
└── base_link                  z = +0.103 (wheel-axle height), x forward, y left, z up
    ├── front_left_steer_link  vertical steering axis at the wheel centre (fixed joint when not steerable)
    │   └── front_left_wheel_link     continuous joint about y
    ├── front_right_steer_link
    │   └── front_right_wheel_link
    ├── rear_left_steer_link
    │   └── rear_left_wheel_link
    ├── rear_right_steer_link
    │   └── rear_right_wheel_link
    ├── imu_link               PLACEHOLDER at base_link origin
    ├── rail_left_link         top face of the left 2040 rail, mid length (y = +0.100)
    ├── rail_right_link        top face of the right 2040 rail, mid length (y = -0.100)
    └── top_plate_link         top surface of the deck
        └── laser_link         EXAMPLE mount, 0.32 m ahead of top_plate_link (laser_example:=false removes it)
```

Joint names follow the link names: `<corner>_steer_joint`, `<corner>_wheel_joint`, where
`<corner>` is `front_left`, `front_right`, `rear_left` or `rear_right`. Every link and joint
name is prepended with the `prefix` argument.

Key heights above `base_footprint`:

| frame | `4ws` (from CAD) | `df` / `fa` / `da` |
|---|---|---|
| `base_link` | 0.103 | 0.103 |
| `top_plate_link` | 0.354 | 0.272 (to be confirmed) |
| `rail_left_link` / `rail_right_link` | 0.374 | 0.292 (to be confirmed) |

Wheel centres are at x = +/-0.297 m and y = +/-0.250 m (`4ws`) or +/-0.298 m (others), z = 0 in `base_link`,
so the wheel bottoms sit at z = 0 in `base_footprint` in every configuration and at every steering angle.

### How the CAD was oriented

The CAD export uses X = width, Y = up, Z = length. It is mapped to REP-103 as
`x = Z_cad`, `y = X_cad`, `z = Y_cad - Y_axle`, which places the mesh origin at axle height.
The CAD +Z end was taken as the **front**: it carries a separate fascia bracket with a centred,
forward-facing sensor housing, while the CAD -Z end carries a plate hanging under the bumper with
two electrode strips (auto-docking charging contacts), which was treated as the rear. The side
panel connector with the dust cap therefore sits on the left side, rear half. **to be confirmed**
against the real vehicle; if it is the other way round, regenerate the meshes with the front/rear
swapped (rotate by pi about z in `tools/glb_to_meshes.py`).

## View it

```bash
cd ~/luchs_ws
colcon build --packages-select luchs_description
source install/setup.bash
ros2 launch luchs_description display.launch.py                 # config:=4ws, sliders
ros2 launch luchs_description display.launch.py config:=fa use_gui:=false
```

Launch arguments: `config` (`df|fa|da|4ws`), `prefix`, `use_gui` (joint_state_publisher_gui vs
joint_state_publisher), `use_rviz`. The RViz fixed frame is `base_footprint`; change it in RViz if you
use a prefix.

Plain URDF:

```bash
xacro $(ros2 pkg prefix luchs_description)/share/luchs_description/urdf/luchs_a.urdf.xacro config:=da > luchs_a_da.urdf
check_urdf luchs_a_da.urdf
```

## Use it in your own robot and attach sensors

`urdf/luchs_a_macro.xacro` defines the `luchs_a` macro, so the chassis can be embedded in a larger
description. Mount payloads on `rail_left_link`, `rail_right_link` or `top_plate_link`; all three have
z pointing up out of the mounting surface.

```xml
<?xml version="1.0"?>
<robot name="my_luchs" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="$(find luchs_description)/urdf/luchs_a_macro.xacro"/>
  <xacro:luchs_a prefix="" config="4ws" laser_example="false"/>

  <!-- GNSS antenna on the left rail, 0.25 m behind the rail centre, 60 mm bracket -->
  <link name="gnss_link"/>
  <joint name="gnss_joint" type="fixed">
    <parent link="rail_left_link"/>
    <child link="gnss_link"/>
    <origin xyz="-0.25 0 0.06" rpy="0 0 0"/>
  </joint>

  <!-- forward-looking camera on the left rail, near the front end, 5 deg pitched down -->
  <link name="front_camera_link">
    <visual>
      <origin xyz="0 0 0.02" rpy="0 0 0"/>
      <geometry><box size="0.03 0.10 0.03"/></geometry>
    </visual>
  </link>
  <joint name="front_camera_joint" type="fixed">
    <parent link="rail_left_link"/>
    <child link="front_camera_link"/>
    <origin xyz="0.30 0 0" rpy="0 0.0873 0"/>
  </joint>
</robot>
```

## Meshes

| file | content | triangles |
|---|---|---|
| `meshes/luchs_a_4ws_body.stl` | body incl. deck, rails, bumpers, steering-motor housings, charging plate | ~36 k |
| `meshes/luchs_a_wheel.stl` | one wheel (tyre + rim), centred on its axle, axle = mesh y, outer face +y | ~8 k |
| `meshes/luchs_a_4ws_steer_module.stl` | steering module below the housing (swing arm, spring, axle carrier), front-left variant | ~6.5 k |
| `meshes/luchs_a_4ws_steer_module_mirror.stl` | mirrored module, front-right variant | ~6.5 k |

Binary STL, metres, REP-103 axes. The four corner modules are point-symmetric in pairs, so
front-left / rear-right share one mesh (rear rotated by pi about z) and front-right / rear-left share the
mirrored one; no negative mesh scaling is used. Right-hand wheels use the same wheel mesh rotated by pi.
Colours are set by URDF materials (body dark grey, wheels black, steering modules accent red #C00B11).

The meshes were produced from the 4WS IP54 CAD export (GLB) with `tools/glb_to_meshes.py`
(per-part quadric decimation, `trimesh` + `fast-simplification`):

```bash
python3 tools/glb_to_meshes.py Luchs_A-4WS_IP54.glb meshes
```

## to be confirmed

- Front/rear assignment of the CAD (see "How the CAD was oriented").
- `4ws` steering limits +/-pi/2, and all steering/wheel effort and velocity limits (placeholders).
- `imu_link` pose (placeholder at `base_link` origin).
- `laser_link` is an example mount only.
- Masses split: 1.2 kg per wheel and 0.6 kg per steering/hub module are estimates; the body gets the rest of
  the published vehicle mass. Body inertia is a uniform box with the COM at its centre.
- `4ws` ground clearance: the CAD belly (charging plate) sits 0.134 m above ground versus 220 mm in the spec;
  CAD overall height is 0.374 m versus 372 mm in the spec.
- `df` / `fa` / `da`: body box 0.70 x 0.48 m, wheel width 69 mm (665 - 596), and the assumption that the
  292 mm height includes the 20 mm rails (deck at 0.272 m) are all estimates until CAD is available.
- Steering axis of the `4ws` module taken through the wheel centre (CAD offset below 1 mm).

## Luchs B

`urdf/luchs_b.urdf.xacro` (macro `luchs_b` in `luchs_b_macro.xacro`), args `variant` (`standard` black / `cooling` white), `prefix`, `laser_example`.

| | |
|---|---|
| Envelope L × W × H | 1990 × 1150 × 800 mm (data sheet v1.3) |
| Wheelbase / track | 1430 / 970 mm (to be confirmed: 1445 / 990 mm in other sources) |
| Tyres | 22×7-10, radius 0.279 m; in-wheel hub motors |
| Ground clearance | 300 mm |
| Mass | 230 kg (body box and masses are estimates) |
| Steering joints | all four, ±0.5236 rad (30°, to be confirmed): front and rear EPS, single or dual Ackermann |

```
base_footprint
└── base_link                  z = +0.279 (axle height)
    ├── <corner>_steer_link / <corner>_wheel_link     same names as Luchs A
    ├── imu_link               chassis IMU (LCI), PLACEHOLDER at base_link
    └── top_plate_link         deck surface, z = 0.800 above ground
        └── laser_link         EXAMPLE mount, 0.75 m ahead of top_plate_link
```

Drawn with boxes and cylinders of the published envelope until the Luchs B CAD is released. The driver publishes the four steering joints from the measured axle angles (LCI 0x5B6), so the wheels in RViz follow the real ones.

```bash
ros2 launch luchs_description display.launch.py config:=b variant:=cooling
```
