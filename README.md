# Embedded software to run on ALTAIR V2 Raspberry Pi 4B flight computer.

Peripherals include:
- Holybro Pixhawk 6X mini
- LR-900p radio
- VESC
- Photidiode interface and power distribution HATs


## Auto-start process:

Systemd unit file written in `/etc/systemd/system/flight.service`

Commands:

```
sudo systemctl enable flight    # auto-start on boot
sudo systemctl disable flight   # stop auto-starting
sudo systemctl start flight     # start now
sudo systemctl stop flight      # stop now
sudo systemctl status flight    # check state + recent logs
journalctl -u flight -f         # tail logs
```