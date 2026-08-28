#!/usr/bin/env bash
# ONE-TIME setup -- run with sudo, from this directory.
#
#   sudo ./setup_can.sh
#
# After this, on EVERY boot and EVERY CAN-adapter hot-plug, can0/can1 come up
# in CAN-FD mode with txqueuelen 1000 and restart-ms 100 -- for every user, no
# password prompts. Any user can also re-kick it with `sudo openarm-can-up`.
set -euo pipefail
[ "$(id -u)" = 0 ] || { echo "run me with sudo"; exit 1; }
SRC="$(cd "$(dirname "$0")" && pwd)"

install -m 0755 "$SRC/can_up.sh" /usr/local/sbin/openarm-can-up
echo "installed /usr/local/sbin/openarm-can-up"

cat > /etc/systemd/system/openarm-can.service <<'UNIT'
[Unit]
Description=Bring up OpenArm CAN-FD interfaces
After=systemd-udev-settle.service
Wants=systemd-udev-settle.service
[Service]
Type=oneshot
RemainAfterExit=yes
ExecStartPre=/bin/sleep 2
ExecStart=/usr/local/sbin/openarm-can-up
Restart=on-failure
RestartSec=3
[Install]
WantedBy=multi-user.target
UNIT

cat > /etc/systemd/system/openarm-can-hotplug.service <<'UNIT'
[Unit]
Description=Re-bring-up OpenArm CAN after adapter hot-plug
[Service]
Type=oneshot
ExecStartPre=/bin/sleep 1
ExecStart=/usr/local/sbin/openarm-can-up
UNIT

cat > /etc/udev/rules.d/90-openarm-can.rules <<'RULE'
ACTION=="add", SUBSYSTEM=="net", KERNEL=="can*", ENV{ID_VENDOR_ID}=="0c72", TAG+="systemd", ENV{SYSTEMD_WANTS}+="openarm-can-hotplug.service"
RULE

cat > /etc/sudoers.d/openarm-can <<'SUD'
ALL ALL=(root) NOPASSWD: /usr/local/sbin/openarm-can-up
SUD
chmod 0440 /etc/sudoers.d/openarm-can
visudo -cf /etc/sudoers.d/openarm-can >/dev/null && echo "sudoers rule OK"

systemctl daemon-reload
udevadm control --reload-rules
systemctl enable --now openarm-can.service
echo
systemctl --no-pager --full status openarm-can.service | head -12 || true
echo
echo "Done. Both buses should be up now; they will come up automatically from now on."
