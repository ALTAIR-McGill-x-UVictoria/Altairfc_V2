from __future__ import annotations

import dataclasses
import logging

from core.datastore import DataStore
from core.task_base import BaseTask
from telemetry.registry import packet_registry
from telemetry.serializer import PacketSerializer
from telemetry.transport import SerialTransport

logger = logging.getLogger(__name__)


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
        logger.info("TelemetryTask: transport opened")

    def execute(self) -> None:
        for packet_id, pkt_class in packet_registry.all_packets().items():
            keys_map: dict[str, str] = getattr(pkt_class, "DATASTORE_KEYS", {})
            if not keys_map:
                continue

            kwargs: dict[str, object] = {}
            for field_name, ds_key in keys_map.items():
                kwargs[field_name] = self.datastore.read(ds_key, default=0.0)

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
        self.transport.close()
        logger.info("TelemetryTask: transport closed")
