#!/usr/bin/env python3
"""Checks luchs_lci_v1.0_draft_r3.dbc against the byte vectors of the C++ codec tests
(luchs_sdk/test/test_lci.cpp, tests marked "DBC vector"), so the DBC given to the chassis team
and the SDK cannot drift apart. Run: python3 dbc/check_lci_dbc.py"""
import os
import sys

import cantools

HERE = os.path.dirname(os.path.abspath(__file__))
db = cantools.database.load_file(os.path.join(HERE, 'luchs_lci_v1.0_draft_r3.dbc'))

# (message, physical values, expected bytes) - identical to test_lci.cpp
ENCODE = [
    ('LCI_MotionCmd', dict(TargetVx=1.0, TargetVy=0.0, TargetWz=0.3, KinematicModel=4, ControlMode=0),
     '03E80000012C0400'),
    ('LCI_SteerCmd', dict(Gear=4, FrontSteerAngle=0.211, RearSteerAngle=-0.211, SteerSource=1,
                          AccelLimit=1.5), '0400D3FF2D0105DC'),
    ('LCI_LampCmd', dict(TurnIndicator=3, ReverseLamp=0, FaultLamp=0, Headlight=2, Horn=1, Beacon=0),
     '0300000201000000'),
    ('LCI_HostHeartbeat', dict(AliveCounter=0x7F, EStopRequest=0, ParkRequest=1, PayloadPower1_On=0,
                               PayloadPower2_On=1, SpeedLimit=2.5, Checksum=0x74), '7F1200FA00000074'),
]
DECODE = [
    ('LCI_LimitsFb', '01F400C8020B1103', dict(MaxVxFwd=5.0, MaxVxRev=2.0, MaxSteerAngle=0.523,
                                               LimitReason=0x11, SpeedClipped=1, SteerClipped=1,
                                               AccelClipped=0)),
    ('LCI_ChassisState', '0280000D01F405', dict(OperatingMode=2, EStopSource=0x80, FaultLevel=0,
                                                RCOnline=1, RCEnabled=0, DriveMode4WD=1,
                                                ParkBrakeEngaged=1, RCLink=0, CmdClipped=0,
                                                ActiveSpeedLimit=5.0, AliveCounter=5)),
    ('LCI_BatteryFb', '222E011D00000300', dict(SOC=87.5, BatteryTemp=28.5, ChargeProgress=0,
                                                BatteryVoltage=76.8)),
    ('LCI_ImuRateFb', '0000000001F40700', dict(RollRate=0, PitchRate=0, YawRate=0.5, ImuCounter=7)),
    ('LCI_AttitudeFb', '03E8FC1800000000', dict(Roll=0.1, Pitch=-0.1)),
    ('LCI_B_MotorTempFb', '288C00FF46474849', dict(MotorTemp_LF=0, MotorTemp_RF=100,
                                                   MotorTemp_LR=-40, CtrlTemp_RR=33)),
]

fail = 0
for name, phys, want in ENCODE:
    got = db.get_message_by_name(name).encode(phys, scaling=True, strict=False).hex().upper()
    ok = got == want
    fail += not ok
    print(f"{'ok  ' if ok else 'FAIL'} encode {name:20s} {got} {'' if ok else '(want ' + want + ')'}")
for name, data, want in DECODE:
    raw = bytes.fromhex(data.ljust(16, '0'))
    if name == 'LCI_ChassisState':   # checksum: low byte of sum(bytes 0-6) XOR 0xFF
        raw = raw[:7] + bytes([(sum(raw[:7]) & 0xFF) ^ 0xFF])
    dec = db.get_message_by_name(name).decode(raw, decode_choices=False)
    bad = {k: (dec[k], v) for k, v in want.items() if abs(float(dec[k]) - v) > 1e-6}
    fail += bool(bad)
    print(f"{'ok  ' if not bad else 'FAIL'} decode {name:20s} {raw.hex().upper()} {bad or ''}")
print('all DBC vectors match the SDK' if not fail else f'{fail} mismatches')
sys.exit(1 if fail else 0)
