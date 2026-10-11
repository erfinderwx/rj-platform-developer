#!/usr/bin/env python3
# Copyright 2026 TerraAgent Robotics. SPDX-License-Identifier: Apache-2.0
"""Minimal host loop for a Luchs A chassis in Python: drive at a fixed twist, then park.
Shows the frames a host must send every cycle. For real applications use luchs_sdk (C++) or
luchs_driver (ROS 2), which add limits, ramps and watchdogs.

    python3 luchs_drive.py --vx 0.2 --seconds 2 --model 4ws --yes
"""
import argparse
import struct
import time

import can

MODELS = {'diff': 1, 'ackermann': 2, '4ws': 3, 'dual_ackermann': 4}
GEAR_PARK, GEAR_DRIVE = 1, 4


def i16(x):
    return max(-32768, min(32767, round(x * 1000)))


def cycle(bus, vx, vy, wz, model, gear):
    bus.send(can.Message(arbitration_id=0x5A0, is_extended_id=False,
                         data=struct.pack('>hhhBB', i16(vx), i16(vy), i16(wz), model, 0x00)))
    bus.send(can.Message(arbitration_id=0x5A1, is_extended_id=False,
                         data=struct.pack('>BhhBBB', gear, 0, 0, 0, 0, 0)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--interface', default='can0')
    ap.add_argument('--vx', type=float, default=0.0)
    ap.add_argument('--vy', type=float, default=0.0)
    ap.add_argument('--wz', type=float, default=0.0)
    ap.add_argument('--model', choices=MODELS, default='4ws')
    ap.add_argument('--seconds', type=float, default=2.0)
    ap.add_argument('--yes', action='store_true', help='confirm that the area is clear')
    a = ap.parse_args()
    if not a.yes:
        raise SystemExit('This moves the vehicle. Clear the area, keep the e-stop in reach, add --yes.')
    if max(abs(a.vx), abs(a.vy)) > 0.5 or abs(a.wz) > 0.8 or a.seconds > 10:
        raise SystemExit('Example limits: |vx|,|vy| <= 0.5 m/s, |wz| <= 0.8 rad/s, <= 10 s.')
    with can.Bus(interface='socketcan', channel=a.interface, bitrate=1_000_000) as bus:
        end = time.monotonic() + a.seconds
        try:
            while time.monotonic() < end:
                cycle(bus, a.vx, a.vy, a.wz, MODELS[a.model], GEAR_DRIVE)
                time.sleep(0.05)                       # 20 Hz
        finally:
            for _ in range(10):                        # stop and park, also on Ctrl-C
                cycle(bus, 0.0, 0.0, 0.0, MODELS[a.model], GEAR_PARK)
                time.sleep(0.05)
    print('stopped, gear P')


if __name__ == '__main__':
    main()
