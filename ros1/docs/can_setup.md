# CAN setup

> **Development preview 1.1.0-preview.1.** Adapted from the publicly distributed Luchs software v1.1.0 on 2026-10-11. Code/package versions remain 1.1.0 for source traceability. Tests supplied with the source are not evidence of successful execution or vehicle validation. Luchs B LCI is a draft interface; B configuration and firmware compatibility still require confirmation. See the [version development manual](../../docs/luchs-development-manual.md).

> ROS 1 (`luchs_ros1`): parameter names use `/` where the ROS 2 driver uses `.` (`protocol/type`, `lci/follow_live_limits`); launch files are `luchs_a.launch` / `luchs_b.launch`.

| | Luchs A | Luchs B |
|---|---|---|
| Host CAN | signal expansion port, rear of the deck (DB9) | dedicated customer CAN on the payload connector (LCI); connector and pin-out to be defined with the chassis team |
| Bit rate | 1 Mbit/s | 1 Mbit/s (LCI); VCU interface V2.6: to be confirmed, 500 kbit/s expected |
| Frames from the chassis | 0x5B5 … 0x5BC | LCI: 0x5B0 … 0x5C1 · VCU V2.6: 0x703 |

## Wiring (Luchs A)

The signal expansion port is under the magnetic cover at the rear of the deck (lift it with a flat screwdriver). It carries CAN, RS-485 and RS-232 on one DB9 cable:

| Pin | Wire | Signal |
|---|---|---|
| 1 | black | RS-232 RX |
| 2 | brown | RS-232 TX |
| 3 | red | GND |
| 4 | orange | **CAN-H** |
| 5 | yellow | **CAN-L** |
| 6 | green | RS-485 A |
| 7 | blue | RS-485 B |
| 8 | grey | GND |
| 9 | white | GND |

Connect CAN-H, CAN-L and GND to the USB-CAN adapter. The bus needs 120 Ω termination at both ends; if your adapter has a switchable terminator and the bus is just chassis + adapter, switch it on.

Power for the payload is on the power expansion port next to it: 48 V / 10 A (XT60) and 12 V / 8 A (DC5521).

## Adapters

Any adapter with a Linux SocketCAN driver works: candleLight / CANable / other `gs_usb` devices, PEAK PCAN-USB (`peak_usb`), Kvaser (`kvaser_usb`), on-board CAN of Jetson / industrial PCs. Serial-line adapters (`slcan`) work but are not recommended at 1 Mbit/s.

## Wiring (Luchs B)

The LCI draft calls for a dedicated customer CAN channel isolated from the drive bus; installation and wiring on the actual B unit are unconfirmed. Use a shielded twisted pair, 120 Ω at both ends, and keep the host's ground on the chassis signal ground. Until the payload connector is released, ask the chassis team for the harness of the unit you have.

## Bring the interface up

```bash
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up
candump can0            # sudo apt install can-utils
```

or `sudo luchs_bringup/scripts/setup_can.sh can0` (second argument = bit rate, e.g. `can0 500000`). At boot: `luchs_bringup/systemd/luchs-can.service`. To give the adapter a fixed name independent of plug order: `luchs_bringup/udev/80-luchs-can.rules`.

With the chassis switched on you should see its feedback frames continuously (table above). Nothing on `candump`: check power, wiring (CAN-H/L swapped is the most common fault), termination and bitrate (`ip -details link show can0` → `bitrate 1000000`). `ERROR-PASSIVE` / `BUS-OFF` in `ip -details -statistics link show can0` almost always means wrong bitrate or missing termination.

## Bus load

Luchs A: the driver sends 5 frames per control cycle (default 20 Hz = 100 frames/s, about 1.3 % of a 1 Mbit/s bus). Raising `control/rate` to 50 Hz adds 250 frames/s.

Luchs B, LCI: 3 frames every 20 ms plus lamps (10 Hz) and host time (1 Hz) = 161 frames/s from the host; the chassis sends about 620 frames/s (IMU at 100 Hz included). Together about 10 % of 1 Mbit/s.
