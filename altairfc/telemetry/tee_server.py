from __future__ import annotations

import logging
import queue
import socket
import threading
from typing import Callable

logger = logging.getLogger(__name__)


class TeeServer:
    """
    Bidirectional TCP tee for the LR900P telemetry link. Lets a ground
    station reach the flight computer over a network path (e.g. a ZeroTier
    tunnel) when RF is unavailable but the Pi has internet.

    Outbound (FC -> GS): best-effort fan-out of the exact bytes
    SerialTransport writes to the radio — see broadcast(). Never allowed to
    block or slow down the radio writer thread: each client gets its own
    bounded queue and its own send thread, so a stalled or slow network
    client only drops its own old bytes, never blocks broadcast().

    Inbound (GS -> FC): now that the radio link itself is unidirectional
    (RF hardware/config only carries FC->GS), this is the only surviving
    path for GS->FC command frames. Each client also gets a recv thread that
    forwards whatever bytes it reads to on_recv, unchanged and unparsed —
    the same command-frame parser that already reads serial bytes
    (CommandReceiverTask, via SerialTransport._cmd_buf) is what interprets
    them; this class only moves bytes.
    """

    def __init__(self, host: str = "0.0.0.0", port: int = 5760, queue_maxsize: int = 256,
                 on_recv: Callable[[bytes], None] | None = None) -> None:
        self.host = host
        self.port = port
        self._queue_maxsize = queue_maxsize
        self._on_recv = on_recv
        self._clients: dict[socket.socket, "queue.Queue[bytes]"] = {}
        self._lock = threading.Lock()
        self._running = False
        self._server_sock: socket.socket | None = None

    def start(self) -> None:
        self._running = True
        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_sock.bind((self.host, self.port))
        self._server_sock.listen(4)
        threading.Thread(target=self._accept_loop, name="tee-accept", daemon=True).start()
        logger.info("TeeServer: listening on %s:%d", self.host, self.port)

    def stop(self) -> None:
        self._running = False
        if self._server_sock is not None:
            try:
                self._server_sock.close()
            except OSError:
                pass
        with self._lock:
            clients = list(self._clients)
            self._clients.clear()
        for sock in clients:
            try:
                sock.close()
            except OSError:
                pass
        logger.info("TeeServer: stopped")

    def broadcast(self, data: bytes) -> None:
        """Enqueue data for every connected client. Never blocks."""
        with self._lock:
            queues = list(self._clients.values())
        for q in queues:
            try:
                q.put_nowait(data)
            except queue.Full:
                try:
                    q.get_nowait()
                    q.put_nowait(data)
                except queue.Empty:
                    pass

    def _accept_loop(self) -> None:
        while self._running:
            try:
                sock, addr = self._server_sock.accept()
            except OSError:
                break
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            # Detect a GS peer that vanished without a FIN/RST in a few seconds
            # instead of relying on OS defaults (~2h), so a stale client is
            # cleaned up promptly rather than piling up dead send threads.
            for opt, val in (
                (getattr(socket, "TCP_KEEPIDLE", None), 3),
                (getattr(socket, "TCP_KEEPINTVL", None), 2),
                (getattr(socket, "TCP_KEEPCNT", None), 3),
            ):
                if opt is not None:
                    sock.setsockopt(socket.IPPROTO_TCP, opt, val)
            client_queue: "queue.Queue[bytes]" = queue.Queue(maxsize=self._queue_maxsize)
            with self._lock:
                self._clients[sock] = client_queue
            logger.info("TeeServer: client connected from %s", addr)
            threading.Thread(
                target=self._client_send_loop, args=(sock, client_queue, addr),
                name=f"tee-client-send-{addr[0]}:{addr[1]}", daemon=True,
            ).start()
            threading.Thread(
                target=self._client_recv_loop, args=(sock, addr),
                name=f"tee-client-recv-{addr[0]}:{addr[1]}", daemon=True,
            ).start()

    def _client_send_loop(self, sock: socket.socket, client_queue: "queue.Queue[bytes]", addr) -> None:
        try:
            while self._running:
                data = client_queue.get()
                sock.sendall(data)
        except OSError as e:
            logger.info("TeeServer: client %s send side closed (%s)", addr, e)
        finally:
            with self._lock:
                self._clients.pop(sock, None)
            try:
                sock.shutdown(socket.SHUT_WR)
            except OSError:
                pass

    def _client_recv_loop(self, sock: socket.socket, addr) -> None:
        """
        Forward inbound command bytes to on_recv, unparsed. Runs independently
        of the send loop/queue above (a single socket used for both
        directions) so a GS command can arrive at any time regardless of
        outbound telemetry traffic. Exits once the peer closes its write side
        or the connection errors, shutting down only this socket's read
        half — the send loop owns the actual close()/self._clients removal,
        since a half-closed read side shouldn't stop outbound telemetry to a
        client that's still receiving.
        """
        try:
            while self._running:
                data = sock.recv(4096)
                if not data:
                    return   # peer closed its write side
                if self._on_recv is not None:
                    try:
                        self._on_recv(data)
                    except Exception:
                        logger.exception("TeeServer: on_recv callback failed for %s", addr)
        except OSError as e:
            logger.info("TeeServer: client %s recv side closed (%s)", addr, e)
        finally:
            try:
                sock.shutdown(socket.SHUT_RD)
            except OSError:
                pass
