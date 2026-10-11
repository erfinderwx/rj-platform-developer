#!/usr/bin/env python3
"""Generates luchs_b_vcu_v2.6.dbc - the Luchs B VCU upper-computer interface V2.6 (0x701 / 0x703),
the interface of Luchs B units shipping before LCI firmware - and checks it with cantools against
the byte vectors of luchs_sdk/test/test_luchs_b.cpp. Byte order: little-endian (Intel).
Items marked TO CONFIRM are open with the chassis team (see docs/protocol_luchs_b.md)."""
import os
import sys

import cantools
from cantools.database import Database, Message, Signal
from cantools.database.conversion import BaseConversion

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'luchs_b_vcu_v2.6.dbc')


def sig(name, start, length, signed=False, factor=1.0, offset=0.0, unit='', comment='', choices=None):
    return Signal(name=name, start=start, length=length, byte_order='little_endian', is_signed=signed,
                  conversion=BaseConversion.factory(scale=factor, offset=offset, choices=choices),
                  unit=unit or None, comment=comment or None, receivers=[])


GEAR = {0: 'N', 1: 'D', 2: 'R'}
msgs = [
    Message(0x701, 'UpperCmd', 8, [
        sig('Throttle', 0, 8, unit='%', comment='Accelerator pedal 0-100'),
        sig('Brake', 8, 8, unit='%', comment='Brake pedal 0-100'),
        sig('SteerAngle', 16, 8, True, unit='deg',
            comment='Road-wheel angle -42..42. Sign TO CONFIRM (EPS sheet: clockwise = positive)'),
        sig('SpeedCap', 24, 8, unit='km/h', comment='Target speed limit 0-30'),
        sig('SteerRate', 32, 10, unit='deg/s', comment='50-520'),
        sig('RemoteEnable', 48, 1, comment='0 handset (RC), 1 upper computer'),
        sig('Headlight', 49, 1),
        sig('EStop', 50, 1),
        sig('Park', 51, 1),
        sig('Gear', 52, 4, choices=GEAR),
        sig('Heartbeat', 56, 8, comment='0-255, increment every frame')],
        senders=['Host'], cycle_time=10,
        comment='Host -> VCU, 10 ms. Identifier written "701" in the sheet; hexadecimal TO CONFIRM'),
    Message(0x703, 'VCU2Upper', 8, [
        sig('SOC', 0, 8, unit='%'),
        sig('BatteryTemp', 8, 8, offset=-40, unit='degC'),
        sig('BatteryVoltage', 16, 8, unit='V'),
        sig('FaultLevel', 24, 2, choices={0: 'none', 1: 'motor_controller', 2: 'BMS', 3: 'several'}),
        sig('Current', 32, 10, unit='A', comment='Range -200..200 A in the sheet; offset TO CONFIRM'),
        sig('Gear', 42, 2, choices=GEAR),
        sig('Mode', 44, 1, choices={0: 'remote_control', 1: 'upper_computer'}),
        sig('Speed', 48, 8, unit='km/h', comment='Unsigned, 1 km/h; direction from Gear'),
        sig('Heartbeat', 56, 8)],
        senders=['VCU'], cycle_time=10, comment='VCU -> host, 10 ms'),
]
db = Database(messages=msgs, version='Luchs B VCU upper interface V2.6')
cantools.database.dump_file(db, OUT)
print('wrote', OUT)

# ---- check against the SDK test vectors ----
db = cantools.database.load_file(OUT)
fail = 0
got = db.get_message_by_name('UpperCmd').encode(dict(
    Throttle=30, Brake=0, SteerAngle=-12, SpeedCap=6, SteerRate=200, RemoteEnable=1, Headlight=1,
    EStop=0, Park=0, Gear=1, Heartbeat=0x42), strict=False).hex().upper()
fail += got != '1E00F406C8001342'
print('encode UpperCmd ', got, 'ok' if got == '1E00F406C8001342' else 'FAIL')
dec = db.get_message_by_name('VCU2Upper').decode(bytes([80, 70, 75, 0, 0, (2 << 2) | 0x10, 7, 9]),
                                                  decode_choices=False)
want = dict(SOC=80, BatteryTemp=30, BatteryVoltage=75, Gear=2, Mode=1, Speed=7, Heartbeat=9)
bad = {k: dec[k] for k, v in want.items() if dec[k] != v}
fail += bool(bad)
print('decode VCU2Upper', 'ok' if not bad else f'FAIL {bad}')
sys.exit(1 if fail else 0)
