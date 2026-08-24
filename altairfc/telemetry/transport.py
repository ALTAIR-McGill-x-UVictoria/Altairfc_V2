from __future__ import annotations

import logging
import queue
import threading
import time

import serial

from drivers.lr900p import (
    FrameAssembler,
    ConfigResponse,
    WriteAckResponse,
    build_heartbeat,
    build_config_read,
    build_config_write,
    parse_heartbeat_response,
    parse_config_response,
    parse_write_ack,
    HEARTBEAT_INTERVAL,
    SUBBLOCK_READ,
    SUBBLOCK_WRITE,
)
from telemetry.tee_server import TeeServer

logger = logging.getLogger(__name__)

_SENTINEL      = object()
_BITS_PER_BYTE = 10  # 1 start + 8 data + 1 stop


class SerialTransport:
    """
    Serial transport for the LR900P telemetry radio.

    Owns the serial port exclusively.  Three daemon threads run while open:

      writer    — single serialized writer; drains _priority_queue first (heartbeats,
                  config frames, ACKs), then _tx_queue (telemetry). All outgoing bytes
                  flow through this one thread — no lock contention, no interleaving.
      heartbeat — enqueues LR900P keepalive frames into _priority_queue every 625 ms.
      reader    — reads all incoming bytes, routes them:
                    • LR900P frames → FrameAssembler → _cfg_queue
                    • all bytes → _cmd_buf for CommandReceiverTask

    Public interface:

      send(frame)              — enqueue a telemetry frame (drops oldest if full)
      send_priority(frame)     — enqueue to priority queue (ACKs, config frames)
      read_available() → bytes — drain _cmd_buf for CommandReceiverTask

      read_config(timeout)     → ConfigResponse | None   (blocking)
      write_config(...)        → WriteAckResponse | None (blocking; modem reboots after)
      is_linked() → bool
      wait_until_open(timeout) → bool
    """

    def __init__(self, port: str, baud: int, write_queue_maxsize: int = 64,
                 tee: TeeServer | None = None) -> None:
        self.port = port
        self.baud = baud
        self._secs_per_byte = _BITS_PER_BYTE / baud
        # Optional mirror of every byte written to the radio, out over TCP —
        # lets a ground station reach us over the network (e.g. ZeroTier)
        # when RF is unavailable but we have internet. See tee_server.py.
        self._tee = tee

        self._serial: serial.Serial | None = None

        # Two outgoing queues — writer drains priority first on every iteration.
        # Priority: heartbeats, ACKs, config frames (small, infrequent, must not be dropped).
        # Normal:   telemetry frames (large, frequent, oldest dropped when full).
        self._priority_queue: queue.Queue[bytes | object] = queue.Queue()
        self._tx_queue:       queue.Queue[bytes | object] = queue.Queue(maxsize=write_queue_maxsize)

        # Incoming bytes for CommandReceiverTask
        self._cmd_buf      = bytearray()
        self._cmd_buf_lock = threading.Lock()

        # LR900P config response queue
        self._cfg_queue: queue.Queue[tuple[int, object]] = queue.Queue()

        # LR900P state
        self._lr_seq        = 0
        self._start_t       = 0.0
        self._linked        = False
        self._config_active = False  # True while read_config/write_config is running
        self._hb_gate       = threading.Event()
        self._hb_gate.set()          # starts unblocked (heartbeats allowed)
        self._hb_response   = threading.Event()  # set each time a heartbeat response arrives

        self._assembler = FrameAssembler(self._on_lr_frame)

        self._writer_thread:     threading.Thread | None = None
        self._reader_thread:     threading.Thread | None = None
        self._heartbeat_thread:  threading.Thread | None = None
        self._port_retry_thread: threading.Thread | None = None
        self._running    = False
        self._open_event = threading.Event()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    @property
    def is_open(self) -> bool:
        return self._open_event.is_set()

    def wait_until_open(self, timeout: float = 10.0) -> bool:
        return self._open_event.wait(timeout=timeout)

    def open(self) -> None:
        """
        Start the transport. Never raises: if the configured serial port
        doesn't exist or can't be opened (no radio physically wired — a
        legitimate deployment now that a TeeServer/tunnel can carry telemetry
        and commands on its own, see tee_server.py), self._serial stays None
        and all three threads still start in "radio absent" mode — the
        writer still drains queues and feeds the tee, it just skips the
        actual serial write; the reader idles instead of polling a
        nonexistent port. _port_retry_loop (started alongside the other
        three) periodically retries opening the real port in the
        background, so plugging the radio in later picks it up without a
        restart. Previously this raised on a missing port, which left
        TelemetryTask stuck retrying setup() forever via BaseTask's restart
        backoff — silently killing telemetry over the tunnel too, since
        execute() (and therefore the tee-broadcast path) never got to run
        without a successful setup().
        """
        self._try_open_serial()
        self._start_t = time.monotonic()
        self._running = True
        self._linked  = False

        self._writer_thread = threading.Thread(
            target=self._writer_loop, name="transport-writer", daemon=True)
        self._reader_thread = threading.Thread(
            target=self._reader_loop, name="transport-reader", daemon=True)
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop, name="transport-heartbeat", daemon=True)
        self._port_retry_thread = threading.Thread(
            target=self._port_retry_loop, name="transport-port-retry", daemon=True)

        self._writer_thread.start()
        self._reader_thread.start()
        self._heartbeat_thread.start()
        self._port_retry_thread.start()
        self._open_event.set()
        if self._serial is not None:
            logger.info("SerialTransport: opened %s @ %d baud", self.port, self.baud)
        else:
            logger.warning(
                "SerialTransport: %s not available — running radio-absent "
                "(tunnel-only if configured); retrying in background",
                self.port,
            )

    def _try_open_serial(self) -> bool:
        """
        Attempt to open self.port. Returns success; never raises. Catches
        Exception broadly (not just serial.SerialException) — same
        precedent as _handle_disconnect's own reconnect loop below — since
        this is startup/background-retry code where any failure to open
        must degrade to "radio absent" rather than propagate and take down
        the calling thread (open() itself, or _port_retry_loop).
        """
        try:
            self._serial = serial.Serial(self.port, self.baud, timeout=0.05)
            return True
        except Exception as e:
            logger.warning("SerialTransport: failed to open %s: %s", self.port, e)
            self._serial = None
            return False

    def _port_retry_loop(self) -> None:
        """
        While self._serial is None (radio never opened, or _handle_disconnect
        gave up — it doesn't; see below), periodically retry opening the
        real port so a radio plugged in after startup is picked up without a
        restart. No-ops once _serial is set. Uses the same backoff shape as
        _handle_disconnect's reconnect loop.
        """
        delay = 1.0
        while self._running:
            if self._serial is None:
                if self._try_open_serial():
                    self._linked = False   # unknown until the next heartbeat response
                    logger.info("SerialTransport: %s became available, opened", self.port)
                else:
                    time.sleep(delay)
                    delay = min(delay * 2, 30.0)
                    continue
            time.sleep(delay)

    def close(self) -> None:
        self._running = False
        # Unblock the writer and let it drain before we close the port.
        self._priority_queue.put(_SENTINEL)
        self._tx_queue.put(_SENTINEL)
        if self._writer_thread:
            self._writer_thread.join(timeout=3.0)
        if self._serial and self._serial.is_open:
            # Return radio to transparent bridge mode before closing.
            try:
                self._serial.write(
                    build_heartbeat(self._next_lr_seq(), self._pc_uptime_ms(), link_flag=0x00))
                time.sleep(0.05)
            except Exception:
                pass
            self._serial.close()
        logger.info("SerialTransport: closed")

    # ------------------------------------------------------------------
    # Telemetry TX
    # ------------------------------------------------------------------

    def send(self, frame: bytes) -> None:
        """Enqueue a telemetry frame; drop oldest if full."""
        while True:
            try:
                self._tx_queue.put_nowait(frame)
                return
            except queue.Full:
                try:
                    dropped = self._tx_queue.get_nowait()
                    if isinstance(dropped, bytes):
                        logger.debug("TX queue full — dropped %d-byte frame", len(dropped))
                except queue.Empty:
                    pass

    def send_priority(self, frame: bytes) -> None:
        """Enqueue to the priority queue (ACK frames, config frames). Never dropped."""
        self._priority_queue.put(frame)

    # ------------------------------------------------------------------
    # Command RX
    # ------------------------------------------------------------------

    def read_available(self) -> bytes:
        with self._cmd_buf_lock:
            data = bytes(self._cmd_buf)
            self._cmd_buf.clear()
        return data

    def attach_tee(self, tee: TeeServer) -> None:
        """
        Attach a TeeServer after construction. Split from __init__'s tee
        parameter because building the tee now requires a callback bound to
        this transport (TeeServer(on_recv=transport.feed_command_bytes)) —
        see altairfc/main.py's startup sequence — so the transport must
        exist first.
        """
        self._tee = tee

    def feed_command_bytes(self, data: bytes) -> None:
        """
        Inject bytes from a non-radio command source (currently: TeeServer's
        recv side, since the radio link is RF-unidirectional and can no
        longer carry GS->FC bytes at all) into the same buffer
        CommandReceiverTask drains via read_available(). Bytes from the
        tunnel and the radio (were it bidirectional) are indistinguishable
        once in _cmd_buf — the frame parser in CommandReceiverTask._process_buffer
        re-syncs on the 0xAA sync byte regardless of source, same as it
        already tolerates telemetry echo interleaved with real commands on a
        half-duplex radio.
        """
        with self._cmd_buf_lock:
            self._cmd_buf.extend(data)

    # ------------------------------------------------------------------
    # LR900P config API
    # ------------------------------------------------------------------

    def is_linked(self) -> bool:
        return self._linked

    def _enter_config_mode(self, timeout: float = 2.0) -> bool:
        """
        Send a 0x1E heartbeat and wait for the modem to acknowledge it.
        Returns True if the modem responded within timeout, False otherwise.
        Must be called with _hb_gate already cleared.
        """
        self._hb_response.clear()
        self._priority_queue.put(
            build_heartbeat(self._next_lr_seq(), self._pc_uptime_ms(), link_flag=0x1E))
        return self._hb_response.wait(timeout=timeout)

    def read_config(self, timeout: float = 3.0) -> ConfigResponse | None:
        self._flush_cfg_queue()
        self._config_active = True
        self._hb_gate.clear()
        try:
            if not self._enter_config_mode(timeout=min(2.0, timeout)):
                logger.warning("SerialTransport: modem did not respond to config-mode heartbeat")
                return None
            self._priority_queue.put(build_config_read(self._next_lr_seq()))
            return self._wait_cfg(SUBBLOCK_READ, timeout)
        finally:
            self._config_active = False
            self._hb_gate.set()

    def write_config(self, data_rate: int, tx_power: int, channel: int,
                     timeout: float = 3.0) -> WriteAckResponse | None:
        if not 0 <= data_rate <= 2:
            raise ValueError("data_rate must be 0-2")
        if not 0 <= tx_power <= 2:
            raise ValueError("tx_power must be 0-2")
        if not 0 <= channel <= 63:
            raise ValueError("channel must be 0-63")
        self._flush_cfg_queue()
        self._config_active = True
        self._hb_gate.clear()
        try:
            if not self._enter_config_mode(timeout=min(2.0, timeout)):
                logger.warning("SerialTransport: modem did not respond to config-mode heartbeat")
                return None
            self._priority_queue.put(
                build_config_write(self._next_lr_seq(), data_rate, tx_power, channel))
            result = self._wait_cfg(SUBBLOCK_WRITE, timeout)
            # Modem reboots after a write — mark link down so callers can detect it.
            self._linked = False
            return result
        finally:
            self._config_active = False
            self._hb_gate.set()

    # ------------------------------------------------------------------
    # Internal — LR900P helpers
    # ------------------------------------------------------------------

    def _next_lr_seq(self) -> int:
        s = self._lr_seq
        self._lr_seq = (self._lr_seq + 1) & 0xFF
        return s

    def _pc_uptime_ms(self) -> int:
        return int((time.monotonic() - self._start_t) * 1000) & 0xFFFF

    def _flush_cfg_queue(self) -> None:
        while not self._cfg_queue.empty():
            try:
                self._cfg_queue.get_nowait()
            except queue.Empty:
                break

    def _wait_cfg(self, subblock: int, timeout: float):
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            try:
                item = self._cfg_queue.get(timeout=remaining)
                if item[0] == subblock:
                    return item[1]
                self._cfg_queue.put(item)
                time.sleep(0.005)
            except queue.Empty:
                return None

    def _on_lr_frame(self, raw: bytes) -> None:
        if parse_heartbeat_response(raw) is not None:
            self._linked = True
            self._hb_response.set()
            return
        cfg = parse_config_response(raw)
        if cfg is not None:
            self._cfg_queue.put((SUBBLOCK_READ, cfg))
            return
        ack = parse_write_ack(raw)
        if ack is not None:
            self._cfg_queue.put((SUBBLOCK_WRITE, ack))

    # ------------------------------------------------------------------
    # Threads
    # ------------------------------------------------------------------

    def _writer_loop(self) -> None:
        next_send = time.monotonic()
        while True:
            # Drain priority queue first (non-blocking).
            try:
                item = self._priority_queue.get_nowait()
            except queue.Empty:
                # Nothing urgent — block on normal telemetry queue.
                try:
                    item = self._tx_queue.get(timeout=0.01)
                except queue.Empty:
                    continue

            if item is _SENTINEL:
                break
            if not isinstance(item, bytes):
                continue

            now  = time.monotonic()
            wait = next_send - now
            if wait > 0:
                time.sleep(wait)

            # No physical radio open (never connected, or a mid-session
            # disconnect _port_retry_loop hasn't recovered from yet) — still
            # feed the tee so tunnel-only operation keeps working, just skip
            # the serial write there's no port to receive it. self._secs_per_byte
            # pacing is radio-specific and meaningless here, so it's skipped
            # too — the tee has no such rate limit to respect.
            if self._serial is None:
                if self._tee is not None:
                    self._tee.broadcast(item)
                continue

            try:
                self._serial.write(item)
                if self._tee is not None:
                    self._tee.broadcast(item)
                next_send = time.monotonic() + len(item) * self._secs_per_byte
            except serial.SerialException as e:
                logger.error("SerialTransport: write error — %s", e)
                self._handle_disconnect()

    def _heartbeat_loop(self) -> None:
        next_tick = time.monotonic()
        while self._running:
            # Block here while a config exchange is in progress — read/write_config
            # enqueues its own 0x1E heartbeat before the command frame and resumes
            # this loop via _hb_gate.set() in its finally block.
            self._hb_gate.wait()
            now = time.monotonic()
            if now >= next_tick:
                # LR900P keepalive — pointless with no physical modem to keep
                # alive, and _writer_loop would otherwise still tee.broadcast()
                # these out over the tunnel for nothing (harmless — GS-side
                # TunnelReader's 0xAA sync scan just skips 0xEF-prefixed
                # bytes as noise — but wasted bandwidth).
                if self._serial is not None:
                    self._priority_queue.put(
                        build_heartbeat(self._next_lr_seq(), self._pc_uptime_ms(), link_flag=0x00))
                next_tick += HEARTBEAT_INTERVAL
            time.sleep(0.01)

    def _handle_disconnect(self) -> None:
        """Called by writer or reader on a hard serial error. Closes the port and
        reconnects with exponential backoff. Blocks until reconnected or stopped."""
        logger.warning("SerialTransport: port disconnected — attempting reconnect")
        try:
            self._serial.close()
        except Exception:
            pass
        self._linked = False
        self._hb_gate.clear()  # pause heartbeats while port is down

        delay = 1.0
        while self._running:
            time.sleep(delay)
            try:
                self._serial = serial.Serial(self.port, self.baud, timeout=0.05)
                logger.info("SerialTransport: reconnected to %s", self.port)
                self._hb_gate.set()
                return
            except Exception as e:
                logger.warning("SerialTransport: reconnect failed (%s) — retrying in %.0fs", e, delay)
                delay = min(delay * 2, 30.0)

    def _reader_loop(self) -> None:
        while self._running:
            if self._serial is None:
                # No physical radio open — nothing to poll. _port_retry_loop
                # owns retrying the actual open(); this just idles rather than
                # spinning on self._serial.read() raising AttributeError.
                # Inbound command bytes still arrive via feed_command_bytes()
                # from the tee's recv side regardless of this loop's state.
                time.sleep(0.5)
                continue
            try:
                data = self._serial.read(256)
            except serial.SerialException as e:
                if self._running:
                    logger.error("SerialTransport: read error — %s", e)
                    self._handle_disconnect()
                continue
            except Exception:
                if self._running:
                    time.sleep(0.1)
                continue
            if not data:
                continue
            self._assembler.feed(data)
            with self._cmd_buf_lock:
                self._cmd_buf.extend(data)
