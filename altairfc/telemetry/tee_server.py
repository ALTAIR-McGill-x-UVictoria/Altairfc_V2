from __future__ import annotations

import logging
import queue
import socket
import threading

logger = logging.getLogger(__name__)


class TeeServer:
    """
    Best-effort TCP fan-out of the exact bytes SerialTransport writes to the
    LR900P radio. Lets a ground station reach the flight computer over a
    network path (e.g. a ZeroTier tunnel) and see the same byte stream the
    radio carries, for use when RF is unavailable but the Pi has internet.

    Never allowed to block or slow down the radio writer thread: each client
    gets its own bounded queue and its own send thread, so a stalled or slow
    network client only drops its own old bytes, never blocks broadcast().
    """

    def __init__(self, host: str = "0.0.0.0", port: int = 5760, queue_maxsize: int = 256) -> None:
        self.host = host
        self.port = port
        self._queue_maxsize = queue_maxsize
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
                target=self._client_loop, args=(sock, client_queue, addr),
                name=f"tee-client-{addr[0]}:{addr[1]}", daemon=True,
            ).start()

    def _client_loop(self, sock: socket.socket, client_queue: "queue.Queue[bytes]", addr) -> None:
        try:
            while self._running:
                data = client_queue.get()
                sock.sendall(data)
        except OSError as e:
            logger.info("TeeServer: client %s disconnected (%s)", addr, e)
        finally:
            with self._lock:
                self._clients.pop(sock, None)
            try:
                sock.close()
            except OSError:
                pass
