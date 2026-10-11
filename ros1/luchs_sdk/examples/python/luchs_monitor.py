#!/usr/bin/env python3
# Copyright 2026 TerraAgent Robotics. SPDX-License-Identifier: Apache-2.0
"""Prints decoded Luchs A chassis feedback - no ROS, no C++ needed.

    pip install python-can cantools
    python3 luchs_monitor.py --interface can0 --dbc ../../../dbc/luchs_a_can_v1.1.dbc
"""
import argparse
import time

import can
import cantools


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--interface', default='can0')
    ap.add_argument('--dbc', required=True)
    ap.add_argument('--period', type=float, default=0.5, help='print period, s')
    a = ap.parse_args()

    db = cantools.database.load_file(a.dbc)
    feedback = {m.frame_id for m in db.messages if 'Chassis' in m.senders}
    latest = {}
    with can.Bus(interface='socketcan', channel=a.interface, bitrate=1_000_000) as bus:
        last = time.monotonic()
        while True:
            msg = bus.recv(timeout=0.05)
            if msg and msg.arbitration_id in feedback:
                m = db.get_message_by_frame_id(msg.arbitration_id)
                latest[m.name] = m.decode(msg.data, decode_choices=True)
            if time.monotonic() - last >= a.period:
                last = time.monotonic()
                print('\033[2J\033[H' + time.strftime('%H:%M:%S'))
                for name in sorted(latest):
                    vals = '  '.join(f'{k}={v:.3f}' if isinstance(v, float) else f'{k}={v}'
                                     for k, v in latest[name].items())
                    print(f'{name:14s} {vals}')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
