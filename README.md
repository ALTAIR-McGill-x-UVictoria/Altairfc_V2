# Embedded software to run on ALTAIR V2 Raspberry Pi 4B flight computer.

Peripherals include:
- Holybro Pixhawk 6X mini
- LR-900p radio
- VESC
- Photidiode interface and power distribution HATs


## Auto-start process:

Copy the unit file from the repo root, then reload systemd:

```
sudo cp flight.service /etc/systemd/system/flight.service
sudo systemctl daemon-reload
```

Systemd unit file written in `/etc/systemd/system/flight.service`

Commands:

```
sudo systemctl enable flight    -- auto-start on boot
sudo systemctl disable flight   -- stop auto-starting
sudo systemctl start flight     -- start now
sudo systemctl stop flight      -- stop now
sudo systemctl status flight    -- check state + recent logs
journalctl -u flight -f         -- tail logs
```

## Optional CPU profiling

For test runs, set `enabled = true` in the `[profiling]` section of
`altairfc/config/settings.toml`. The flight log will periodically report CPU
used by the highest-load AltairFC scheduler tasks, along with execution counts
and average/maximum execution time. CPU used by transport, watchdog, buzzer,
and other helper threads is reported separately. Profiling is disabled by
default and starts no background thread until enabled.
