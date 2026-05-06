from __future__ import annotations

import dataclasses
import logging
import threading

from core.datastore import DataStore
from core.task_base import BaseTask
from telemetry.packets.heartbeat import collect_system_stats
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

    Runs at a configurable rate (default 10 Hz). On each execute():
      1. Iterates over all registered packet types.
      2. Reads DataStore keys listed in each packet's DATASTORE_KEYS class var.
      3. Instantiates each packet with the latest values.
      4. Packs and sends the binary frame via SerialTransport.

    Adding a new packet type: define it in telemetry/packets/, decorate with
    @packet_registry.register(packet_id=N), and import it before startup.
    No changes needed here.
    """

    def __init__(
        self,
        name: str,
        period_s: float,
        datastore: DataStore,
        transport: SerialTransport,
    ) -> None:
        super().__init__(name, period_s, datastore)
        self.transport = transport
        self._serializer = PacketSerializer()
        self._seq_counters: dict[int, int] = {}

    def setup(self) -> None:
        self.transport.open()
        self._packet_list: list[tuple[int, type]] = []
        self._packet_index: int = 0
        self._stats_stop = threading.Event()
        self._stats_thread = threading.Thread(
            target=_stats_worker,
            args=(self.datastore, self._stats_stop, lambda: len(self._packet_list)),
            name="telemetry-stats",
            daemon=True,
        )
        self._stats_thread.start()
        logger.info("TelemetryTask: transport opened")

    def execute(self) -> None:
        # Rebuild packet list only when the registry changes (e.g. on first call)
        all_packets = packet_registry.all_packets()
        if len(all_packets) != len(self._packet_list):
            self._packet_list = [
                (pid, cls) for pid, cls in all_packets.items()
                if getattr(cls, "DATASTORE_KEYS", {})
            ]
            self._packet_index = 0

        if not self._packet_list:
            return

        # Send one packet this cycle
        packet_id, pkt_class = self._packet_list[self._packet_index]
        self._packet_index = (self._packet_index + 1) % len(self._packet_list)

        field_types = {f.name: f.type for f in dataclasses.fields(pkt_class)}
        kwargs: dict[str, object] = {}
        for field_name, ds_key in pkt_class.DATASTORE_KEYS.items():
            raw = self.datastore.read(ds_key, default=0)
            kwargs[field_name] = int(raw) if field_types.get(field_name) == "int" else float(raw)

        try:
            packet = pkt_class(**kwargs)
        except TypeError:
            logger.warning("TelemetryTask: failed to instantiate %s", pkt_class.__name__)
            return

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
