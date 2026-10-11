#!/usr/bin/env bash
# Bring up the CAN interface for a Luchs chassis.
#   sudo ./setup_can.sh                    # can0, 1 Mbit/s (Luchs A, Luchs B with LCI)
#   sudo ./setup_can.sh can1
#   sudo ./setup_can.sh can0 500000        # other bit rates, e.g. a Luchs B VCU bus (TO CONFIRM)
# USB-CAN adapters based on gs_usb (candleLight, CANable, PEAK via peak_usb) appear as canX.
set -euo pipefail
IFACE="${1:-can0}"
BITRATE="${2:-1000000}"

if [[ $EUID -ne 0 ]]; then echo "run with sudo" >&2; exit 1; fi
modprobe can_raw 2>/dev/null || true
modprobe gs_usb 2>/dev/null || true
if ! ip link show "$IFACE" >/dev/null 2>&1; then
  echo "interface $IFACE not found. Plug in the USB-CAN adapter, then check: ip -br link" >&2
  exit 1
fi
ip link set "$IFACE" down 2>/dev/null || true
ip link set "$IFACE" type can bitrate "$BITRATE" restart-ms 100
ip link set "$IFACE" txqueuelen 1000
ip link set "$IFACE" up
ip -details -statistics link show "$IFACE" | sed -n '1,6p'
echo "$IFACE up at $BITRATE bit/s. Check traffic with: candump $IFACE   (package can-utils)"
