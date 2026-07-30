from __future__ import annotations

import csv
import logging
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any

from core.datastore import DataStore
from core.photodiode_stream import (
    SERGEANT_VALID,
    SOLDIER_VALID,
    PhotodiodeSample,
    PhotodiodeSampleBuffer,
)
from core.task_base import BaseTask

logger = logging.getLogger(__name__)

_ADC_VREF_V = 2.5
_ADC_FULL_SCALE_CODE = float(1 << 23)
_LOG_FLUSH_INTERVAL_S = 1.0


def _code_to_voltage(code: int) -> float:
    """Match ads124s08_code_to_volts() without discarding the raw ADC code."""

    return (float(code) / _ADC_FULL_SCALE_CODE) * _ADC_VREF_V + _ADC_VREF_V


class PhotodiodeTask(BaseTask):
    """Acquire and locally log time-aligned photodiode samples.

    TIA inputs are configured once and sampled every task period. Temperature
    channels are polled on their own slower schedule, then both ADCs are restored
    to the TIA mux before the next high-rate sample.
    """

    def __init__(
        self,
        name: str,
        period_s: float,
        datastore: DataStore,
        *,
        signal_data_rate: str = "SPS_800",
        temperature_data_rate: str = "SPS_2000",
        temperature_period_s: float = 1.0,
        bias_voltage_v: float = 0.0,
        sample_buffer: PhotodiodeSampleBuffer | None = None,
        log_path: Path | None = None,
        pdro_factory: Callable[..., Any] | None = None,
    ) -> None:
        super().__init__(name=name, period_s=period_s, datastore=datastore)
        if temperature_period_s <= 0:
            raise ValueError("temperature_period_s must be greater than zero")
        self._signal_data_rate_name = signal_data_rate
        self._temperature_data_rate_name = temperature_data_rate
        self._temperature_period_s = temperature_period_s
        self._bias_voltage_v = bias_voltage_v
        self._sample_buffer = sample_buffer
        self._log_path = log_path
        self._pdro_factory = pdro_factory
        self._pdro = None
        self._signal_data_rate = None
        self._temperature_data_rate = None
        self._readouts = ()
        self._log_file = None
        self._log_writer = None

    @staticmethod
    def _parse_data_rate(data_rate, name: str):
        try:
            return data_rate[name.strip().upper()]
        except KeyError as exc:
            choices = ", ".join(data_rate.__members__)
            raise ValueError(
                f"unknown PDRO data rate {name!r}; choose from {choices}"
            ) from exc

    def setup(self) -> None:
        # Keep hardware imports lazy so a disabled task does not load Pi-only
        # dependencies during flight-computer startup.
        from drivers.uvic_pdro import DataRate, Input, Readout, SignalPath, UVICPDRO

        self.datastore.write("system.photodiode_connected", 0)
        signal_rate = self._parse_data_rate(
            DataRate, self._signal_data_rate_name
        )
        temperature_rate = self._parse_data_rate(
            DataRate, self._temperature_data_rate_name
        )
        readouts = (Readout.SERGEANT, Readout.SOLDIER)
        factory = self._pdro_factory or UVICPDRO
        pdro = factory(readouts=readouts)
        try:
            for readout in readouts:
                pdro.set_bias_voltage(readout, self._bias_voltage_v)
                pdro.set_signal_paths(readout, SignalPath.TIA_LOW_GAIN)
                if (
                    pdro.configure_input(
                        readout, Input.TIA_LOW_GAIN, signal_rate
                    )
                    is None
                ):
                    raise OSError(f"failed to configure {readout.value} TIA input")

            if self._log_path is not None:
                self._log_path.parent.mkdir(parents=True, exist_ok=True)
                self._log_file = open(
                    self._log_path,
                    "w",
                    newline="",
                    buffering=64 * 1024,
                )
                self._log_writer = csv.writer(self._log_file)
                self._log_writer.writerow(
                    [
                        "sequence",
                        "time_unix_us",
                        "monotonic_ns",
                        "valid_flags",
                        "sergeant_code",
                        "soldier_code",
                        "sergeant_voltage_v",
                        "soldier_voltage_v",
                    ]
                )
        except Exception:
            try:
                pdro.close()
            except Exception:
                logger.exception("PhotodiodeTask: cleanup after setup failure failed")
            self._close_log()
            raise

        now = time.monotonic()
        self._pdro = pdro
        self._signal_data_rate = signal_rate
        self._temperature_data_rate = temperature_rate
        self._readouts = readouts
        self._next_sequence = 0
        self._next_temperature_t = now + self._temperature_period_s
        self._next_log_flush_t = now + _LOG_FLUSH_INTERVAL_S
        self._last_sample_monotonic_ns: int | None = None
        self._overruns = 0
        self._failed_conversions = 0
        self._samples_acquired = 0
        self._samples_logged = 0
        self._rate_window_started_t = now
        self._rate_window_samples = 0
        self.datastore.write("system.photodiode_connected", 1)
        self.datastore.write("photodiode.samples_acquired", 0)
        self.datastore.write("photodiode.samples_logged", 0)
        self.datastore.write("photodiode.sample_overruns", 0)
        self.datastore.write("photodiode.failed_conversions", 0)
        logger.info(
            "PhotodiodeTask: UVIC PDRO ready (signal=%s at %.1f Hz, "
            "temperature=%s every %.2f s, bias=%.3f V)",
            signal_rate.name,
            1.0 / self.period_s,
            temperature_rate.name,
            self._temperature_period_s,
            self._bias_voltage_v,
        )

    def _restore_signal_inputs(self) -> None:
        from drivers.uvic_pdro import Input

        assert self._pdro is not None
        assert self._signal_data_rate is not None
        for readout in self._readouts:
            if (
                self._pdro.configure_input(
                    readout, Input.TIA_LOW_GAIN, self._signal_data_rate
                )
                is None
            ):
                raise OSError(f"failed to restore {readout.value} TIA input")

    @staticmethod
    def _temperature_value(reading) -> float | None:
        if reading is None or reading.temperature_c is None:
            return None
        return float(reading.temperature_c)

    def _poll_temperatures(self, now: float) -> None:
        assert self._pdro is not None
        assert self._temperature_data_rate is not None
        sergeant, soldier = self._readouts
        measurements = (
            (
                "photodiode.sergeant.board_temperature",
                self._pdro.read_board_thermistor(
                    sergeant, self._temperature_data_rate
                ),
            ),
            (
                "photodiode.soldier.board_temperature",
                self._pdro.read_board_thermistor(
                    soldier, self._temperature_data_rate
                ),
            ),
            (
                "photodiode.sergeant.photodiode_temperature",
                self._pdro.read_photodiode_thermistor(
                    sergeant, self._temperature_data_rate
                ),
            ),
            (
                "photodiode.soldier.photodiode_temperature",
                self._pdro.read_photodiode_thermistor(
                    soldier, self._temperature_data_rate
                ),
            ),
        )
        for key, reading in measurements:
            value = self._temperature_value(reading)
            if value is None:
                self._failed_conversions += 1
            else:
                self.datastore.write(key, value)
        self._restore_signal_inputs()

        self._next_temperature_t += self._temperature_period_s
        if self._next_temperature_t < now:
            self._next_temperature_t = now + self._temperature_period_s

    def _write_sample_log(self, sample: PhotodiodeSample) -> None:
        if self._log_writer is None:
            return
        sergeant_v = (
            _code_to_voltage(sample.sergeant_code)
            if sample.valid_flags & SERGEANT_VALID
            else ""
        )
        soldier_v = (
            _code_to_voltage(sample.soldier_code)
            if sample.valid_flags & SOLDIER_VALID
            else ""
        )
        self._log_writer.writerow(
            [
                sample.sequence,
                sample.time_unix_us,
                sample.monotonic_ns,
                sample.valid_flags,
                sample.sergeant_code,
                sample.soldier_code,
                sergeant_v,
                soldier_v,
            ]
        )
        self._samples_logged += 1

    def _publish_metrics(self, now: float) -> None:
        elapsed = now - self._rate_window_started_t
        if elapsed < 1.0:
            return
        rate_hz = self._rate_window_samples / elapsed
        self.datastore.write("photodiode.sample_rate_hz", rate_hz)
        self.datastore.write(
            "photodiode.samples_acquired", self._samples_acquired
        )
        self.datastore.write("photodiode.samples_logged", self._samples_logged)
        self.datastore.write("photodiode.sample_overruns", self._overruns)
        self.datastore.write(
            "photodiode.failed_conversions", self._failed_conversions
        )
        if self._sample_buffer is not None:
            self.datastore.write(
                "photodiode.tx_queue_depth", len(self._sample_buffer)
            )
        self._rate_window_started_t = now
        self._rate_window_samples = 0

    def execute(self) -> None:
        if self._pdro is None:
            raise RuntimeError("UVIC PDRO is not initialized")

        started_monotonic_ns = time.monotonic_ns()
        started_unix_ns = time.time_ns()
        sergeant, soldier = self._readouts
        sergeant_code = self._pdro.read_raw_code(sergeant)
        soldier_code = self._pdro.read_raw_code(soldier)
        ended_monotonic_ns = time.monotonic_ns()

        valid_flags = 0
        if sergeant_code is not None:
            valid_flags |= SERGEANT_VALID
        else:
            sergeant_code = 0
            self._failed_conversions += 1
        if soldier_code is not None:
            valid_flags |= SOLDIER_VALID
        else:
            soldier_code = 0
            self._failed_conversions += 1

        midpoint_offset_ns = (ended_monotonic_ns - started_monotonic_ns) // 2
        sample = PhotodiodeSample(
            sequence=self._next_sequence,
            time_unix_us=(started_unix_ns + midpoint_offset_ns) // 1_000,
            monotonic_ns=started_monotonic_ns + midpoint_offset_ns,
            sergeant_code=int(sergeant_code),
            soldier_code=int(soldier_code),
            valid_flags=valid_flags,
        )
        self._next_sequence = (self._next_sequence + 1) & 0xFFFFFFFF

        if self._last_sample_monotonic_ns is not None:
            interval_ns = sample.monotonic_ns - self._last_sample_monotonic_ns
            if interval_ns > int(self.period_s * 1_000_000_000 * 1.5):
                self._overruns += 1
        self._last_sample_monotonic_ns = sample.monotonic_ns

        # Local recording always happens before a sample becomes eligible for TX.
        self._write_sample_log(sample)
        if self._sample_buffer is not None:
            self._sample_buffer.append(sample)

        if valid_flags & SERGEANT_VALID:
            self.datastore.write(
                "photodiode.sergeant.tia_low_gain",
                _code_to_voltage(sample.sergeant_code),
                timestamp=sample.monotonic_ns / 1_000_000_000.0,
            )
        if valid_flags & SOLDIER_VALID:
            self.datastore.write(
                "photodiode.soldier.tia_low_gain",
                _code_to_voltage(sample.soldier_code),
                timestamp=sample.monotonic_ns / 1_000_000_000.0,
            )

        self._samples_acquired += 1
        self._rate_window_samples += 1
        now = time.monotonic()
        if now >= self._next_temperature_t:
            self._poll_temperatures(now)
        if self._log_file is not None and now >= self._next_log_flush_t:
            self._log_file.flush()
            self._next_log_flush_t = now + _LOG_FLUSH_INTERVAL_S
        self._publish_metrics(now)

    def _close_log(self) -> None:
        log_file = self._log_file
        self._log_file = None
        self._log_writer = None
        if log_file is not None:
            try:
                log_file.flush()
                log_file.close()
            except Exception:
                logger.exception("PhotodiodeTask: failed to close sample log")

    def teardown(self) -> None:
        pdro = self._pdro
        self._pdro = None
        self._signal_data_rate = None
        self._temperature_data_rate = None
        self._readouts = ()
        try:
            if pdro is not None:
                pdro.close()
        finally:
            self._close_log()
            self.datastore.write("system.photodiode_connected", 0)
        logger.info("PhotodiodeTask: UVIC PDRO closed")
