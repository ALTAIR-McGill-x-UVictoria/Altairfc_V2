from __future__ import annotations

import csv
import dataclasses
import logging
import time
from pathlib import Path

from core.datastore import DataStore
from core.task_base import BaseTask
from telemetry.registry import packet_registry

logger = logging.getLogger(__name__)

_FLUSH_INTERVAL_S = 5.0


class DataLoggerTask(BaseTask):
    """
    Writes one CSV per registered packet type to a timestamped log directory.

    Each CSV has a header row derived from the packet's field names, prefixed with
    time_unix and monotonic columns. Rows are written at each packet's TX_RATE_HZ
    (same rate as the telemetry task). Files are flushed to disk every 5 seconds.

    Log directory: <log_root>/<YYYY-MM-DD_HH-MM-SS>/
    """

    def __init__(
        self,
        name: str,
        period_s: float,
        datastore: DataStore,
        log_root: Path,
    ) -> None:
        super().__init__(name=name, period_s=period_s, datastore=datastore)
        self._log_dir = log_root

    def setup(self) -> None:
        self._log_dir.mkdir(parents=True, exist_ok=True)
        logger.info("DataLoggerTask: logging to %s", self._log_dir)

        # {packet_id: (pkt_class, csv.writer, file, next_write_monotonic)}
        self._schedule: dict[int, tuple[type, csv.writer, object, float]] = {}
        self._files: list[object] = []

        now = time.monotonic()
        eligible = [
            (pid, cls) for pid, cls in packet_registry.all_packets().items()
            if getattr(cls, "DATASTORE_KEYS", {}) and getattr(cls, "TX_RATE_HZ", 0) > 0
        ]
        for i, (pid, cls) in enumerate(eligible):
            fname = self._log_dir / f"{cls.__name__}.csv"
            fh = open(fname, "w", newline="", buffering=1)
            field_names = [f.name for f in dataclasses.fields(cls)]
            writer = csv.writer(fh)
            writer.writerow(["time_unix", "monotonic"] + field_names)
            period = 1.0 / cls.TX_RATE_HZ
            stagger = i * period / max(len(eligible), 1)
            self._schedule[pid] = (cls, writer, fh, now + stagger)
            self._files.append(fh)

        self._next_flush = now + _FLUSH_INTERVAL_S
        self.datastore.write("event.data_logging_active", 1)
        logger.info("DataLoggerTask: opened %d CSV files", len(self._schedule))

    def execute(self) -> None:
        now = time.monotonic()
        time_unix = time.time()

        for pid, (cls, writer, fh, next_write) in list(self._schedule.items()):
            if now < next_write:
                continue

            period = 1.0 / cls.TX_RATE_HZ
            new_next = next_write + period
            if new_next < now:
                new_next = now + period
            self._schedule[pid] = (cls, writer, fh, new_next)

            field_types = {f.name: f.type for f in dataclasses.fields(cls)}
            row = [f"{time_unix:.3f}", f"{now:.3f}"]
            for field_name, ds_key in cls.DATASTORE_KEYS.items():
                raw = self.datastore.read(ds_key, default=0)
                val = int(raw) if field_types.get(field_name) == "int" else float(raw)
                row.append(val)
            writer.writerow(row)

        if now >= self._next_flush:
            for fh in self._files:
                fh.flush()
            self._next_flush = now + _FLUSH_INTERVAL_S

    def teardown(self) -> None:
        self.datastore.write("event.data_logging_active", 0)
        for fh in self._files:
            try:
                fh.flush()
                fh.close()
            except Exception:
                pass
        logger.info("DataLoggerTask: all CSV files closed")
