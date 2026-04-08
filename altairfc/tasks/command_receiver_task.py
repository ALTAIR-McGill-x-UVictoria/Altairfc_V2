from __future__ import annotations

import dataclasses
import logging

from core.datastore import DataStore
from core.task_base import BaseTask
from telemetry.command_registry import command_registry
from telemetry.packets.ack import AckPacket
from telemetry.registry import packet_registry
from telemetry.serializer import PacketSerializer, SYNC_BYTE, HEADER_SIZE, CRC_SIZE, _HEADER_STRUCT

logger = logging.getLogger(__name__)

MIN_FRAME_SIZE = HEADER_SIZE + CRC_SIZE

# ACK status codes
ACK_OK       = 0
ACK_REJECTED = 1


class CommandReceiverTask(BaseTask):
    """
    Reads GS→FC command frames from the telemetry serial port and dispatches
    them to the DataStore.

    Shares the SerialTransport instance with TelemetryTask. TelemetryTask.setup()
    opens the port — this task must be registered AFTER TelemetryTask and must
    NOT call transport.open() itself.

    On each execute() call (20 Hz), drains all available bytes from the serial
    port, scans for valid command frames, unpacks them using the command_registry,
    and writes the command's DATASTORE_KEY with the payload value.

    FlightStageTask polls those DataStore keys on its next cycle.
    """

    def __init__(
        self,
        name: str,
        period_s: float,
        datastore: DataStore,
        transport,  # SerialTransport — no type import to avoid circular deps
    ) -> None:
        super().__init__(name, period_s, datastore)
        self._transport = transport
        self._serializer = PacketSerializer()
        self._buf = bytearray()
        # Pre-compiled struct for the ACK packet (looked up once from registry)
        self._ack_seq: int = 0

    def setup(self) -> None:
        # Transport already opened by TelemetryTask — do not call transport.open() here
        logger.info("CommandReceiverTask: ready (sharing transport with TelemetryTask)")

    def execute(self) -> None:
        chunk = self._transport.read_available()
        if chunk:
            self._buf.extend(chunk)
            self._process_buffer()

    def teardown(self) -> None:
        self._buf.clear()

    # ------------------------------------------------------------------
    # Frame parsing (mirrors GS SerialReader._process_buffer)
    # ------------------------------------------------------------------

    def _process_buffer(self) -> None:
        while len(self._buf) >= MIN_FRAME_SIZE:
            # Find sync byte
            sync_pos = self._buf.find(SYNC_BYTE)
            if sync_pos == -1:
                self._buf.clear()
                return
            if sync_pos > 0:
                del self._buf[:sync_pos]

            if len(self._buf) < HEADER_SIZE:
                return  # wait for full header

            _, cmd_id, cmd_seq, _, length = _HEADER_STRUCT.unpack_from(self._buf, 0)
            frame_size = HEADER_SIZE + length + CRC_SIZE

            if len(self._buf) < frame_size:
                return  # wait for full frame

            frame = bytes(self._buf[:frame_size])
            del self._buf[:frame_size]

            result = self._serializer.unpack(frame, registry=command_registry)
            if result is None:
                # Not a valid command frame — could be our own telemetry echo,
                # which is expected on half-duplex radios. Silently ignore.
                continue

            command, _ = result
            self._dispatch(command, cmd_id, cmd_seq)

    def _dispatch(self, command: object, cmd_id: int, cmd_seq: int) -> None:
        ds_key = getattr(type(command), "DATASTORE_KEY", None)
        status = ACK_OK

        if ds_key is not None:
            # Write the command value to the DataStore
            fields = dataclasses.fields(command)
            value = float(getattr(command, fields[0].name, 1)) if fields else 1.0
            self.datastore.write(ds_key, value)
            logger.info(
                "CommandReceiverTask: %s → %s = %s", type(command).__name__, ds_key, value
            )

            # LAUNCH_OK: reject immediately if not in ARMED stage
            if ds_key == "command.launch_ok":
                from tasks.flight_stage_task import STAGE_ARMED  # local import to avoid circular
                stage = int(self.datastore.read("event.flight_stage", default=0))
                if stage != STAGE_ARMED:
                    status = ACK_REJECTED
                    logger.warning(
                        "CommandReceiverTask: LAUNCH_OK rejected — stage is %d, expected %d (ARMED)",
                        stage, STAGE_ARMED,
                    )
        else:
            # No DataStore key — command is acknowledged at the transport layer only (e.g. PING)
            logger.info("CommandReceiverTask: %s received (no DataStore key)", type(command).__name__)

        ack = AckPacket(cmd_id=cmd_id, cmd_seq=cmd_seq, status=status)
        ack_frame = self._serializer.pack(ack, seq=self._ack_seq)
        self._ack_seq = (self._ack_seq + 1) & 0xFF
        self._transport.send(ack_frame)
        logger.info(
            "CommandReceiverTask: ACK sent (cmd_id=0x%02X cmd_seq=%d status=%d)",
            cmd_id, cmd_seq, status,
        )
