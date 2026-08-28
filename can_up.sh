#!/usr/bin/env bash
# Bring up the OpenArm CAN-FD interface(s) correctly. Idempotent.
# Always does down->up so it also clears a BUS-OFF.
#
#   sudo ./can_up.sh            # both can0 and can1
#   sudo ./can_up.sh can1       # just one
#
# Settings and why:
#   fd on, dbitrate 5000000 : the left arm's motors reply with BRS CAN-FD
#     frames -- without FD mode those frames never arrive.
#   restart-ms 100 : auto-recover from BUS-OFF instead of wedging until a
#     manual down/up.
#   txqueuelen 1000 : socketcan TX ring; the default 10 overflows with 8
#     motors at 50 Hz ("No buffer space [105]").
set -u
if [ "$#" -gt 0 ]; then IFACES=("$@"); else IFACES=(can0 can1); fi
rc=0
for IF in "${IFACES[@]}"; do
  if [ ! -e "/sys/class/net/$IF" ]; then
    echo "[$IF] no such interface (adapter unplugged?) -- skipping"; rc=1; continue
  fi
  ip link set "$IF" down 2>/dev/null || true
  if ip link set "$IF" up type can bitrate 1000000 dbitrate 5000000 fd on restart-ms 100 \
     && ip link set "$IF" txqueuelen 1000; then
    echo -n "[$IF] "; ip -details link show "$IF" | awk '/state/{print $9, $10, $11}' | head -1
  else
    echo "[$IF] FAILED to bring up"; rc=1
  fi
done
exit $rc
