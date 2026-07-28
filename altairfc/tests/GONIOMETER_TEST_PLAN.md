# Sphere LED stability + goniometric scan — lab test plan

Status: 2026-07-27. Scripts and drivers referenced here live in this
directory (`altairfc/tests/`) and `altairfc/drivers/`. Reduction lives in
the sibling `NRC-calibration/` repo.

## Scope and two hard constraints, up front

The sphere's R/G/B LEDs have **no per-colour control** — they are always
driven together as one combined optical output, with one shared
current-sense resistor and one shared thermistor for the whole board
(`drivers/sphere_led.py`). Consequences:

- Nothing below can isolate a single wavelength. Every "LED on" measurement
  is the combined R+G+B spectrum.
- This scan **cannot** satisfy `ALTAIR-analysis/Experiment_Design/01_source_calibration.md`
  section 2 step 3, which calls for angular response measured per wavelength
  independently. It produces I(θ,φ) of the combined spectrum only.
- Getting a true per-wavelength curve needs a hardware change (independently
  switchable LED drivers) or a spectrally-resolving detector at the
  goniometer stage. Neither exists yet — this is an open decision, not
  something fixed by running the scripts differently.

Treat every curve this campaign produces as the combined-spectrum response
and say so explicitly wherever it's used.

There **are** two independently addressable drive channels for that combined
output — a high-power channel (MCP4728 A) and a low-power channel (MCP4728
B), selected with `--channel {high,low}` on both scripts below (default
`high`). DAC codes on **both** channels are hard-limited to `1400` (of 4095)
inside `drivers/sphere_led.py` itself — enforced unconditionally, regardless
of what `--code` requests, so this is a safety floor you can't accidentally
override from the CLI.

## Prerequisites (once, on the Pi)

```bash
cd altairfc
bash drivers/build_all.sh      # builds MCP4728, ADS1220, ADS124S08, DAC5311, gpio_hold .so
sudo pigpiod
pip install smbus2
```

For reduction (can run off the Pi):

```bash
pip install pandas             # not in the conda base env
```

All commands below write CSVs under `altairfc/lab_data/` (create it once:
`mkdir -p lab_data`). `*.csv` is already covered by `.gitignore`, so nothing
here risks being committed — the point of using a repo-local folder instead
of `/tmp` is just that it survives a reboot and keeps a scan's dark/soak/scan
files grouped together instead of scattered.

Known-fixed issue: `MCP4728Driver.set_codes()` (Fast Write) fails on this
hardware — see memory `mcp4728-fast-write-bug`. `SphereLedSource` already
works around it (uses Multi-Write unconditionally); no action needed unless
you're calling `MCP4728Driver` directly.

## Step 1 — Ambient light check (~2 min, LEDs off)

```bash
python tests/soak_sphere_led.py --code 0 --mode open \
    --duration 120 --interval 1 --dark-interval 0 --settle-s 0 \
    --csv lab_data/ambient.csv
```

Watch the `ext=` column; flip the room lights partway through. Large jump →
baffle the detector or kill the lights for everything below. Flat → proceed.

## Step 2 — Smoke test (~5 min, LEDs on)

Confirms DAC, LDAC, current sense, thermistor, both sphere photodiodes, and
the external ADS1220 all read plausibly in one place.

```bash
python tests/soak_sphere_led.py --code 700 --mode open \
    --duration 300 --csv lab_data/soak_smoke.csv
```

Add `--no-external` if the goniometer detector isn't wired yet — the sphere
photodiodes alone are still useful (they're inside the sphere; ambient can't
reach them). Add `--channel low` to smoke-test the low-power channel instead
of the default high-power one.

## Step 3 — Phase 1: LED stability soak (no jig needed — can run in parallel with jig assembly)

Run this once per channel you intend to use for the scan (Step 3 and Step 7
should use the same `--channel`).

```bash
python tests/soak_sphere_led.py --code 700 --mode open \
    --duration 3600 --csv lab_data/soak_open.csv

# use the settled current printed by the run above as --target-current
python tests/soak_sphere_led.py --code 700 --mode current \
    --target-current <value> --duration 3600 --csv lab_data/soak_pi.csv
```

Let the sphere cool fully between the two runs, or the second starts warm
and the comparison is meaningless. Both take periodic LED-off dark frames by
default (`--dark-interval 300`, override with `--dark-interval 0` to
disable) so ambient/electronic drift isn't mistaken for source drift.

```bash
cd ../../NRC-calibration
python reduce_goniometer.py --input ../Altairfc_V2/altairfc/lab_data/soak_open.csv --soak
python reduce_goniometer.py --input ../Altairfc_V2/altairfc/lab_data/soak_pi.csv --soak
```

**Decision point:** compare `dI/dT` and p-p drift between the two runs. If
open-loop drift over the scan's expected duration is comfortably inside your
uncertainty budget, scan open-loop. Otherwise scan with `--target-current`.

## Step 4 — Jig homing (once, after assembly)

```bash
cd ../Altairfc_V2/altairfc
python tests/scan_goniometer.py --home --slew-rate 10
```

Jog each axis to its physical reference (polar = port normal facing the
detector; azimuth = jig fiducial mark), press `a` to accept. Low slew rate,
watch it — this is the step most likely to foul the hardware if rushed.
Saves to `goniometer_home.json`, persists across sessions.

## Step 5 — Scan dry run (no hardware touched)

```bash
python tests/scan_goniometer.py --dry-run --polar -90:90:5 --azimuth 0:180:45
```

Check the printed row count and duration estimate before committing.
`ads1220_read_single_shot()` is not DRDY-driven — ~75 ms per external-PD
conversion — so this adds up fast at high sample counts.

## Step 6 — Single-point scan (confirm schema)

```bash
python tests/scan_goniometer.py --polar 0 --azimuth 0 --samples 20 --csv lab_data/point.csv
```

## Step 7 — Coarse scan, then fine

```bash
python tests/scan_goniometer.py --polar -90:90:15 --azimuth 0:180:90 --csv lab_data/coarse.csv
# then, once that looks right:
python tests/scan_goniometer.py --polar -90:90:5 --azimuth 0:180:45 --csv lab_data/fine.csv
```

Add `--target-current <value>` on both if Step 3 showed meaningful drift.
At least 2 azimuths is required — it's the symmetry check, not padding.

The scan moves nadir (polar, servo b) as the outer loop and azimuth (servo a)
as the inner loop — for each nadir angle it sweeps every azimuth before
moving nadir again, since nadir is the harder motion on this jig and this
keeps it to one move per `--polar` value instead of one per sample.

## Step 8 — Reduce

```bash
cd ../../NRC-calibration
python reduce_goniometer.py --input ../Altairfc_V2/altairfc/lab_data/coarse.csv
```

Read the output in this order — each gates trust in the next:

1. **Sphere-PD flatness / `repeat_drift_frac`** — did the source stay stable
   during the scan?
2. **Azimuthal symmetry verdict** — is the θ-only collapse justified? If
   `ASYMMETRIC`, the deliverable becomes a 2D map and the flight pipeline
   needs gondola roll at each flash — see the design doc, section 2 step 4.
3. **The curve** (`goniometer_curve.png`) vs. the Lambertian overlay
   (comparison only, never a fallback).

## Bench facts to record on paper (not code)

- Sphere exit-port diameter — needed to justify the far-field approximation
  at ~50 cm (design doc section 2 step 2).
- Detector's calibration certificate reference — the SI-traceability chain
  depends on it and nothing in either repo documents it yet.

## Open items not resolved by this plan

- Absolute SI scale (waiting on `distances.csv` / the reference-source
  transfer stubbed out in `NRC-calibration/calibrate_photodiode.py`).
- Per-wavelength angular response (see "Scope and a hard limitation" above).
- In-flight LED thermal control (see memory `led-flight-thermal-control`).
