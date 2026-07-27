"""Shared instrument bundle for the integrating-sphere lab rig.

Composes the four things both the soak characterization
(tests/soak_sphere_led.py) and the goniometric scan (tests/scan_goniometer.py)
need, and defines the one CSV schema they both write so a single reducer
handles either file:

    LED source          drivers/sphere_led.SphereLedSource   (MCP4728 + ADS1115)
    Sphere photodiodes  drivers/uvic_pdro.UVICPDRO           (Sergeant, Soldier)
    External photodiode drivers/ads1220_driver.Ads1220Driver (goniometer detector)

Not a pytest module despite living in tests/ — it holds no test functions and
is not collected.

SPI bus note, and the one thing most likely to silently corrupt a scan:
spi0 runs with dtoverlay=spi0-0cs, so no chip select is driven by hardware and
every device depends on its CS being actively held high to stay off MISO.
UVICPDRO opens a GpioHold on DEFAULT_BACKUP_CS_OFFSETS = (4, 17) for exactly
that reason.  This rig drives one of those boards as the external detector, so
it must pass backup_cs_offsets excluding that offset — otherwise GpioHold
holds the line high while Ads1220Driver tries to pull it low, and the reads
come back quietly wrong rather than failing.  ``_backup_cs_offsets()`` below
does that subtraction.
"""

from __future__ import annotations

import csv
import logging
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from drivers.sphere_led import Color, SphereLedSource  # noqa: E402

logger = logging.getLogger(__name__)

DEFAULT_EXTERNAL_CS_OFFSET = 17  # ADS1220 backup board used as the goniometer detector
DEFAULT_SPI_DEV = "/dev/spidev0.0"
DEFAULT_GPIOCHIP = "gpiochip0"

CSV_COLUMNS = [
    "timestamp_utc",
    "elapsed_s",
    "mode",
    "led",
    "polar_deg",
    "azimuth_deg",
    "sample_index",
    "dac_code",
    "target_current_a",
    "led_current_a",
    "bridge_temperature_c",
    "sgt_tia_v",
    "sol_tia_v",
    "external_pd_v",
]

# Nominal peak wavelengths, carried into the CSV so the reducer can label
# curves per Experiment_Design/01 without a second lookup table.  The sphere
# runs in NRC-calibration all peak near 453 nm.
LED_WAVELENGTH_NM = {Color.RED: 625.0, Color.GREEN: 525.0, Color.BLUE: 453.0}


@dataclass
class RigReading:
    """One simultaneous sample of every instrument in the rig."""

    led_current_a: float
    bridge_temperature_c: float
    dac_code: int
    sgt_tia_v: float | None = None
    sol_tia_v: float | None = None
    external_pd_v: float | None = None


def _backup_cs_offsets(external_cs_offset: int | None) -> tuple[int, ...]:
    """CS lines UVICPDRO should hold high: the defaults, minus the one we drive."""
    from drivers.uvic_pdro import DEFAULT_BACKUP_CS_OFFSETS

    return tuple(o for o in DEFAULT_BACKUP_CS_OFFSETS if o != external_cs_offset)


class SphereRig:
    """Open, sample, and close every instrument as one unit."""

    def __init__(
        self,
        *,
        bus,
        channel_map: dict[Color, int] | None = None,
        pi=None,
        use_ldac: bool = True,
        i2c_dev: str = "/dev/i2c-1",
        use_pdro: bool = True,
        use_external: bool = True,
        external_cs_offset: int = DEFAULT_EXTERNAL_CS_OFFSET,
        spi_dev: str = DEFAULT_SPI_DEV,
        gpiochip: str = DEFAULT_GPIOCHIP,
        signal_data_rate: str = "SPS_100",
    ) -> None:
        self._use_pdro = use_pdro
        self._use_external = use_external
        self._external_cs_offset = external_cs_offset
        self._spi_dev = spi_dev
        self._gpiochip = gpiochip
        self._signal_data_rate_name = signal_data_rate

        self.led = SphereLedSource(
            bus=bus,
            channel_map=channel_map,
            pi=pi,
            use_ldac=use_ldac,
            i2c_dev=i2c_dev,
        )
        self._pdro = None
        self._external = None
        self._readouts = ()
        self._input = None
        self._data_rate = None

        try:
            self._open_photodiodes()
        except Exception:
            self.led.close()
            raise

    def _open_photodiodes(self) -> None:
        if self._use_external:
            from drivers.ads1220_driver import Ads1220Driver

            self._external = Ads1220Driver(
                self._spi_dev, self._gpiochip, cs_offset=self._external_cs_offset
            )
            logger.info(
                "SphereRig: external photodiode on ADS1220 CS %s:%d",
                self._gpiochip,
                self._external_cs_offset,
            )

        if self._use_pdro:
            from drivers.uvic_pdro import DataRate, Input, Readout, UVICPDRO

            self._readouts = (Readout.SERGEANT, Readout.SOLDIER)
            self._input = Input.TIA_LOW_GAIN
            self._data_rate = DataRate[self._signal_data_rate_name]
            self._pdro = UVICPDRO(
                readouts=self._readouts,
                spi_dev=self._spi_dev,
                gpiochip=self._gpiochip,
                backup_cs_offsets=_backup_cs_offsets(
                    self._external_cs_offset if self._use_external else None
                ),
            )
            logger.info("SphereRig: PDRO open on Sergeant + Soldier")

    def sample(self, color: Color | None) -> RigReading:
        """Take one reading from every instrument.

        ``color`` selects which DAC code is recorded; pass None for a dark
        frame, where no channel is driven.
        """
        state = self.led.update()

        reading = RigReading(
            led_current_a=state.current_a,
            bridge_temperature_c=state.temperature_c,
            dac_code=0 if color is None else state.codes[color],
        )

        if self._external is not None:
            reading.external_pd_v = self._external.read_photodiode()

        if self._pdro is not None:
            sergeant, soldier = self._readouts
            reading.sgt_tia_v = self._pdro.read_voltage(sergeant, self._input, self._data_rate)
            reading.sol_tia_v = self._pdro.read_voltage(soldier, self._input, self._data_rate)

        return reading

    def close(self) -> None:
        """Zero the LEDs and release every instrument, worst case first."""
        try:
            self.led.close()
        except Exception:
            logger.exception("SphereRig: LED source close failed")
        for name, device in (("PDRO", self._pdro), ("external ADS1220", self._external)):
            if device is None:
                continue
            try:
                device.close()
            except Exception:
                logger.exception("SphereRig: %s close failed", name)
        self._pdro = None
        self._external = None

    def __enter__(self) -> SphereRig:
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()


class RigCsvWriter:
    """Write the shared schema, flushing every row.

    An interrupted run keeps everything already measured — the convention
    tests/stream_UVICPDRO_calibration.py established for this rig.
    """

    def __init__(self, path: Path) -> None:
        self._path = Path(path)
        self._fh = self._path.open("w", newline="")
        self._writer = csv.DictWriter(self._fh, fieldnames=CSV_COLUMNS)
        self._writer.writeheader()
        self._fh.flush()
        self.rows = 0

    def write(
        self,
        reading: RigReading,
        *,
        elapsed_s: float,
        mode: str,
        led: str,
        sample_index: int,
        polar_deg: float | None = None,
        azimuth_deg: float | None = None,
        target_current_a: float | None = None,
    ) -> None:
        self._writer.writerow(
            {
                "timestamp_utc": datetime.now(timezone.utc).isoformat(),
                "elapsed_s": f"{elapsed_s:.6f}",
                "mode": mode,
                "led": led,
                "polar_deg": "" if polar_deg is None else f"{polar_deg:.3f}",
                "azimuth_deg": "" if azimuth_deg is None else f"{azimuth_deg:.3f}",
                "sample_index": sample_index,
                "dac_code": reading.dac_code,
                "target_current_a": "" if target_current_a is None else f"{target_current_a:.6f}",
                "led_current_a": f"{reading.led_current_a:.9f}",
                "bridge_temperature_c": f"{reading.bridge_temperature_c:.4f}",
                "sgt_tia_v": "" if reading.sgt_tia_v is None else f"{reading.sgt_tia_v:.9f}",
                "sol_tia_v": "" if reading.sol_tia_v is None else f"{reading.sol_tia_v:.9f}",
                "external_pd_v": ""
                if reading.external_pd_v is None
                else f"{reading.external_pd_v:.9f}",
            }
        )
        self._fh.flush()
        self.rows += 1

    def close(self) -> None:
        if self._fh is not None and not self._fh.closed:
            self._fh.close()

    def __enter__(self) -> RigCsvWriter:
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()
