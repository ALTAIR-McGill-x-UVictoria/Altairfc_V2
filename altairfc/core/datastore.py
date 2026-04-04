from __future__ import annotations

import threading
import time
from typing import Any, Callable


class DataStore:
    """
    Thread-safe blackboard for inter-task data sharing.

    Keys are namespaced strings, e.g. "mavlink.attitude.roll".
    Each entry stores (value, timestamp) where timestamp is time.monotonic().
    All reads and writes are protected by a single RLock.
    """

    def __init__(self) -> None:
        self._store: dict[str, tuple[Any, float]] = {}
        self._lock = threading.RLock()
        self._subscribers: dict[str, list[Callable[[str, Any], None]]] = {}

    def write(self, key: str, value: Any, timestamp: float | None = None) -> None:
        ts = timestamp if timestamp is not None else time.monotonic()
        with self._lock:
            self._store[key] = (value, ts)
            callbacks = self._subscribers.get(key, [])
        for cb in callbacks:
            try:
                cb(key, value)
            except Exception:
                pass

    def read(self, key: str, default: Any = None) -> Any:
        with self._lock:
            entry = self._store.get(key)
        return entry[0] if entry is not None else default

    def read_with_timestamp(self, key: str) -> tuple[Any, float] | None:
        with self._lock:
            return self._store.get(key)

    def read_namespace(self, prefix: str) -> dict[str, Any]:
        """Return a snapshot of all keys that start with the given prefix."""
        with self._lock:
            return {k: v for k, (v, _) in self._store.items() if k.startswith(prefix)}

    def subscribe(self, key: str, callback: Callable[[str, Any], None]) -> None:
        """Register a callback invoked after each write to key. For alerting, not data flow."""
        with self._lock:
            self._subscribers.setdefault(key, []).append(callback)
