#!/usr/bin/env python3
"""Generates luchs_a_can_v1.1.dbc (Luchs A chassis CAN protocol v1.1) and checks it with cantools.

Byte order: big-endian (Motorola). Values: physical = raw * factor.
0x5A2 / 0x5A3: both layouts are defined - WheelSteerCmd_0x5A2 and LampCmd_0x5A3 follow the protocol
document; LampCmd_0x5A2 and WheelSteerCmd_0x5A3 follow the reference driver. Only one pair is valid
for a given firmware; see docs/protocol_luchs_a.md.
"""
import os
import cantools
from cantools.database import Database, Message, Signal
from cantools.database.conversion import BaseConversion

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'luchs_a_can_v1.1.dbc')


def sig(name, byte, length, signed=False, factor=1.0, unit='', comment='', choices=None,
        minimum=None, maximum=None):
    # Motorola: start bit = MSB of the first (most significant) byte
    start = byte * 8 + 7
    return Signal(name=name, start=start, length=length, byte_order='big_endian',
                  is_signed=signed, conversion=BaseConversion.factory(scale=factor, offset=0,
                                                                     choices=choices),
                  unit=unit or None, comment=comment or None, minimum=minimum, maximum=maximum,
                  receivers=[])


def i16(name, byte, unit, comment='', factor=0.001):
    return sig(name, byte, 16, True, factor, unit, comment)


MODEL = {1: 'differential', 2: 'front_ackermann', 3: 'four_wheel_steer', 4: 'dual_ackermann'}
GEAR = {0: 'disabled', 1: 'park', 2: 'reverse', 3: 'neutral', 4: 'drive'}
TURN = {0: 'off', 1: 'left', 2: 'right'}
FAULT = {0: 'no_fault', 1: 'ultrasonic_stop', 2: 'emergency_stop', 3: 'parked', 4: 'charging',
         5: 'fl_drive_motor', 6: 'fr_drive_motor', 7: 'rl_drive_motor', 8: 'rr_drive_motor',
         9: 'fl_steer_motor', 10: 'fr_steer_motor', 11: 'rl_steer_motor', 12: 'rr_steer_motor',
         13: 'rc_link_lost', 14: 'watchdog'}

msgs = [
    Message(0x5A0, 'MotionCmd', 8, [
        i16('Vx', 0, 'm/s'), i16('Vy', 2, 'm/s', '4WS model only'), i16('Wz', 4, 'rad/s'),
        sig('Model', 6, 8, choices=MODEL),
        sig('ControlMode', 7, 8, comment='0x00 = host (autonomous) control')],
        senders=['Host'], comment='Host -> chassis motion command'),
    Message(0x5A1, 'SteerCmd', 8, [
        sig('Gear', 0, 8, choices=GEAR), i16('FrontAngle', 1, 'rad'), i16('RearAngle', 3, 'rad')],
        senders=['Host'], comment='Gear and Ackermann (axle) angles'),
    Message(0x5A2, 'WheelSteerCmd_0x5A2', 8, [
        i16('FL', 0, 'rad'), i16('RL', 2, 'rad'), i16('FR', 4, 'rad'), i16('RR', 6, 'rad')],
        senders=['Host'], comment='Protocol-document layout (4WS). Alternative: LampCmd_0x5A2'),
    Message(0x5A3, 'WheelSteerCmd_0x5A3', 8, [
        i16('FL', 0, 'rad'), i16('FR', 2, 'rad'), i16('RL', 4, 'rad'), i16('RR', 6, 'rad')],
        senders=['Host'], comment='Reference-driver layout (4WS). Alternative: LampCmd_0x5A3'),
    Message(0x5A4, 'HostTime', 8, [sig('UnixTimeMs', 0, 64, unit='ms')], senders=['Host'],
            comment='Host time, 64-bit big-endian; epoch to be confirmed'),
    Message(0x5B5, 'MotionFb', 8, [
        i16('Vx', 0, 'm/s'), i16('Vy', 2, 'm/s'), i16('Wz', 4, 'rad/s'),
        sig('Model', 6, 8, choices=MODEL),
        sig('ControlMode', 7, 8, comment='11 / 0x11 remote control, 22 / 0x22 host')],
        senders=['Chassis']),
    Message(0x5B6, 'SteerFb', 8, [
        sig('Gear', 0, 8, choices=GEAR), i16('FrontAngle', 1, 'rad'), i16('RearAngle', 3, 'rad')],
        senders=['Chassis']),
    Message(0x5B7, 'WheelSteerFb', 8, [
        i16('FL', 0, 'rad'), i16('RL', 2, 'rad'), i16('FR', 4, 'rad'), i16('RR', 6, 'rad')],
        senders=['Chassis']),
    Message(0x5B8, 'ProtectionFb', 8, [
        sig('UltrasonicTrigger', 0, 8, choices={0: 'none', 1: 'left', 2: 'right', 3: 'front', 4: 'rear'}),
        sig('Bumper', 1, 8, choices={0: 'none', 1: 'front', 2: 'rear'}),
        sig('EStop', 2, 8, choices={0: 'none', 1: 'left', 2: 'right'}),
        sig('FaultCode', 3, 8, choices=FAULT),
        sig('DistFront', 4, 8, factor=0.01, unit='m', comment='scaling to be confirmed'),
        sig('DistRear', 5, 8, factor=0.01, unit='m'),
        sig('DistLeft', 6, 8, factor=0.01, unit='m'),
        sig('DistRight', 7, 8, factor=0.01, unit='m')], senders=['Chassis']),
    Message(0x5B9, 'WheelSpeedFb', 8, [
        i16('FL', 0, 'rad/s'), i16('RL', 2, 'rad/s'), i16('FR', 4, 'rad/s'), i16('RR', 6, 'rad/s')],
        senders=['Chassis'], comment='Order and unit to be confirmed (reference-driver order shown)'),
    Message(0x5BA, 'MotorFaultFb', 8, [sig('FaultHigh', 0, 32), sig('FaultLow', 4, 32)],
            senders=['Chassis'], comment='Bit definitions not yet published'),
    Message(0x5BB, 'BatteryFb', 8, [
        sig('SoC', 0, 16, factor=0.1, unit='%', comment='scaling to be confirmed'),
        sig('Temperature', 2, 16, True, factor=0.1, unit='degC', comment='scaling to be confirmed'),
        sig('Charging', 4, 16, unit='%'),
        sig('Voltage', 6, 16, factor=0.1, unit='V')], senders=['Chassis']),
    Message(0x5BC, 'LampFb', 8, [
        sig('TurnSignal', 0, 8, choices=TURN), sig('ReverseLamp', 1, 8), sig('FaultLamp', 2, 8),
        sig('LightRain', 3, 8, comment='0x00 dark/dry 0x01 dark/rain 0x10 light/dry 0x11 light/rain')],
        senders=['Chassis']),
]
# Lamp layouts (alternatives on the same identifiers) go in a second file to keep IDs unique.
lamp = [
    Message(0x5A2, 'LampCmd_0x5A2', 8, [sig('TurnSignal', 0, 8, choices=TURN),
            sig('ReverseLamp', 1, 8), sig('FaultLamp', 2, 8)], senders=['Host'],
            comment='Reference-driver layout'),
    Message(0x5A3, 'LampCmd_0x5A3', 8, [sig('TurnSignal', 0, 8, choices=TURN),
            sig('ReverseLamp', 1, 8), sig('FaultLamp', 2, 8)], senders=['Host'],
            comment='Protocol-document layout'),
]
from cantools.database.can.node import Node
db = Database(messages=msgs, nodes=[Node('Host'), Node('Chassis')], version='1.1')
db.refresh()
cantools.database.dump_file(db, OUT)
db2 = Database(messages=lamp, nodes=[Node('Host')], version='1.1')
db2.refresh()
cantools.database.dump_file(db2, OUT.replace('.dbc', '_lamp_cmd.dbc'))

# self-check against the SDK test vectors
d = cantools.database.load_file(OUT)
assert d.encode_message('MotionCmd', {'Vx': 0.5, 'Vy': -0.25, 'Wz': 1.0, 'Model': 3, 'ControlMode': 0}) \
    == bytes([0x01, 0xF4, 0xFF, 0x06, 0x03, 0xE8, 0x03, 0x00])
assert d.encode_message('SteerCmd', {'Gear': 4, 'FrontAngle': 0.3, 'RearAngle': -0.3}) \
    == bytes([0x04, 0x01, 0x2C, 0xFE, 0xD4, 0, 0, 0])
assert d.encode_message('WheelSteerCmd_0x5A3', {'FL': .1, 'FR': .2, 'RL': .3, 'RR': .4}) \
    == bytes([0x00, 0x64, 0x00, 0xC8, 0x01, 0x2C, 0x01, 0x90])
assert d.encode_message('HostTime', {'UnixTimeMs': 0x0000019A3B2C1D0E}) \
    == bytes([0x00, 0x00, 0x01, 0x9A, 0x3B, 0x2C, 0x1D, 0x0E])
m = d.decode_message(0x5BB, bytes([0x03, 0x6B, 0x00, 0xFD, 0x00, 0x00, 0x02, 0x04]))
assert abs(m['SoC'] - 87.5) < 1e-6 and abs(m['Voltage'] - 51.6) < 1e-6, m
m = d.decode_message(0x5BA, bytes([0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0]))
assert m['FaultHigh'] == 0x12345678
print('wrote', OUT, 'and the lamp-command file; all SDK vectors match')
