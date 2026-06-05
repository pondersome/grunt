# grunt_bringup systemd units

System-level units for the Grunt robot (barney NUC). These are **not** ROS
nodes — they are host systemd services installed outside the ROS workspace.

## grunt-wifi-on.service

Forces the WiFi radio on at every boot.

**Why:** NetworkManager persists the soft radio state in
`/var/lib/NetworkManager/NetworkManager.state` (`WirelessEnabled=false`). A WiFi
toggle-off from the desktop GUI (or `nmcli radio wifi off`) survives reboots and
leaves the headless robot unreachable — recovery otherwise requires opening the
chassis and attaching a monitor + keyboard. This is a soft NM state, not an
rfkill hardware block.

The unit is a oneshot ordered `After=NetworkManager.service`; it waits for NM via
`nm-online` then runs `nmcli radio wifi on` (idempotent).

### Install

    sudo bash install-grunt-wifi-on.sh

Run from your own shell — `sudo` needs a real TTY for the password prompt.

### Trade-off

WiFi can no longer be *persistently* disabled; a reboot always re-enables it. To
genuinely keep it off across a reboot:

    sudo systemctl disable grunt-wifi-on.service
