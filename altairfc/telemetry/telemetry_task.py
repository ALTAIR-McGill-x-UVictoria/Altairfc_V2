from __future__ import annotations

import dataclasses
import logging
import threading
import time

from core.datastore import DataStore
from core.photodiode_stream import PhotodiodeSampleBuffer
from core.task_base import BaseTask
from telemetry.packets.heartbeat import collect_system_stats
from telemetry.packets.photodiode_batch import (
    PHOTODIODE_BATCH_PACKET_ID,
    PHOTODIODE_BATCH_SIZE,
    PHOTODIODE_SAMPLE_PERIOD_US,
    PhotodiodeBatchPacket,
)
from telemetry.registry import packet_registry
from telemetry.serializer import PacketSerializer
from telemetry.transport import SerialTransport

logger = logging.getLogger(__name__)

_STATS_INTERVAL_S = 2.0


def _stats_worker(datastore: DataStore, stop: threading.Event, tasks_fn) -> None:
    while not stop.wait(timeout=_STATS_INTERVAL_S):
        for key, value in collect_system_stats(tasks_running=tasks_fn()).items():
            datastore.write(key, value)


class TelemetryTask(BaseTask):
    """
    Collects data from the DataStore, serializes registered packet types,
    and transmits binary frames over the telemetry radio.

    The task tick rate (period_s in settings.toml) sets how often execute() runs.
    Each packet class controls its own TX rate via TX_RATE_HZ — execute() sends
    whichever packets are due on each tick. Initial deadlines are staggered to
    avoid a burst on startup.

    Adding a new packet type: define it in telemetry/packets/, decorate with
    @packet_registry.register(packet_id=N), set TX_RATE_HZ, and import it before
    startup. No changes needed here.
    """

    def __init__(
        self,
        name: str,
        period_s: float,
        datastore: DataStore,
        transport: SerialTransport,
        photodiode_samples: PhotodiodeSampleBuffer | None = None,
        photodiode_batch_size: int = PHOTODIODE_BATCH_SIZE,
        photodiode_batch_rate_hz: float = 2.0,
    ) -> None:
        super().__init__(name, period_s, datastore)
        if not 1 <= photodiode_batch_size <= 255:
            raise ValueError("photodiode_batch_size must be between 1 and 255")
        if photodiode_batch_rate_hz <= 0:
            raise ValueError("photodiode_batch_rate_hz must be greater than zero")
        self.transport = transport
        self._photodiode_samples = photodiode_samples
        self._photodiode_batch_size = photodiode_batch_size
        self._photodiode_batch_period_s = 1.0 / photodiode_batch_rate_hz
        self._serializer = PacketSerializer()
        self._seq_counters: dict[int, int] = {}

    def setup(self) -> None:
        self.transport.open()
        # {packet_id: (pkt_class, next_send_monotonic)}
        self._packet_schedule: dict[int, tuple[type, float]] = {}
        self._stats_stop = threading.Event()
        self._stats_thread = threading.Thread(
            target=_stats_worker,
            args=(self.datastore, self._stats_stop, lambda: len(self._packet_schedule)),
            name="telemetry-stats",
            daemon=True,
        )
        self._stats_thread.start()
        self._next_photodiode_batch_t = time.monotonic()
        logger.info("TelemetryTask: transport opened")

    def _send_photodiode_batch(self, now: float) -> None:
        if self._photodiode_samples is None:
            return
        if now < self._next_photodiode_batch_t:
            return

        samples = self._photodiode_samples.peek_batch(
            self._photodiode_batch_size
        )
        if not samples:
            self._next_photodiode_batch_t = now + self._photodiode_batch_period_s
            return

        sample_period_us = PHOTODIODE_SAMPLE_PERIOD_US
        if len(samples) > 1:
            measured_period_us = round(
                (samples[-1].time_unix_us - samples[0].time_unix_us)
                / (len(samples) - 1)
            )
            sample_period_us = max(1, min(0xFFFF, measured_period_us))
        packet = PhotodiodeBatchPacket(
            samples=samples,
            sample_period_us=sample_period_us,
        )
        packet_id = PHOTODIODE_BATCH_PACKET_ID
        seq = self._seq_counters.get(packet_id, 0)
        frame = self._serializer.pack(packet, seq=seq)
        self.transport.send(frame)
        self._seq_counters[packet_id] = (seq + 1) & 0xFF
        self._photodiode_samples.discard(len(samples))
        self.datastore.write(
            "photodiode.tx_queue_depth", len(self._photodiode_samples)
        )

        self._next_photodiode_batch_t += self._photodiode_batch_period_s
        if self._next_photodiode_batch_t < now:
            self._next_photodiode_batch_t = now + self._photodiode_batch_period_s

    def execute(self) -> None:
        self.datastore.write("system.time_unix", time.time())
        now = time.monotonic()
        try:
            self._send_photodiode_batch(now)
        except Exception:
            logger.exception("TelemetryTask: error packing/sending photodiode batch")

        # Scale factor from radio.rate_scale, written by RadioConfigTask whenever
        # the modem data_rate changes. Defaults to 1.0 until radio config is known.
        scale = float(self.datastore.read("radio.rate_scale", default=1.0))
        scale = max(scale, 0.01)  # guard against zero/negative

        # Build schedule once on first call — stagger initial deadlines to avoid burst
        if not self._packet_schedule:
            eligible = [
                (pid, cls) for pid, cls in packet_registry.all_packets().items()
                if getattr(cls, "DATASTORE_KEYS", {}) and getattr(cls, "TX_RATE_HZ", 0) > 0
            ]
            for i, (pid, cls) in enumerate(eligible):
                period = 1.0 / cls.TX_RATE_HZ
                stagger = i * period / max(len(eligible), 1)
                self._packet_schedule[pid] = (cls, now + stagger)

        if not self._packet_schedule:
            return

        for packet_id, (pkt_class, next_send) in list(self._packet_schedule.items()):
            if now < next_send:
                continue

            # Effective period stretches when scale < 1.0 (slower data rate).
            period = 1.0 / (pkt_class.TX_RATE_HZ * scale)
            # Advance deadline; skip catch-up burst if fallen behind.
            new_next = next_send + period
            if new_next < now:
                new_next = now + period
            self._packet_schedule[packet_id] = (pkt_class, new_next)

            field_types = {f.name: f.type for f in dataclasses.fields(pkt_class)}
            kwargs: dict[str, object] = {}
            for field_name, ds_key in pkt_class.DATASTORE_KEYS.items():
                raw = self.datastore.read(ds_key, default=0)
                kwargs[field_name] = int(raw) if field_types.get(field_name) == "int" else float(raw)

            try:
                packet = pkt_class(**kwargs)
            except TypeError:
                logger.warning("TelemetryTask: failed to instantiate %s", pkt_class.__name__)
                continue

            seq = self._seq_counters.get(packet_id, 0)
            self._seq_counters[packet_id] = (seq + 1) & 0xFF

            try:
                frame = self._serializer.pack(packet, seq=seq)
                self.transport.send(frame)
            except Exception:
                logger.exception("TelemetryTask: error packing/sending %s", pkt_class.__name__)

    def teardown(self) -> None:
        self._stats_stop.set()
        self.transport.close()
        logger.info("TelemetryTask: transport closed")
