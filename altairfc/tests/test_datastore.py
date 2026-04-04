from __future__ import annotations

import threading
import time

import pytest

from core.datastore import DataStore


def test_write_and_read():
    ds = DataStore()
    ds.write("a.b.c", 42)
    assert ds.read("a.b.c") == 42


def test_read_default():
    ds = DataStore()
    assert ds.read("missing.key") is None
    assert ds.read("missing.key", default=99) == 99


def test_read_with_timestamp():
    ds = DataStore()
    before = time.monotonic()
    ds.write("x", 1.0)
    after = time.monotonic()
    result = ds.read_with_timestamp("x")
    assert result is not None
    value, ts = result
    assert value == 1.0
    assert before <= ts <= after


def test_read_with_timestamp_missing():
    ds = DataStore()
    assert ds.read_with_timestamp("nope") is None


def test_read_namespace():
    ds = DataStore()
    ds.write("mavlink.attitude.roll", 0.1)
    ds.write("mavlink.attitude.pitch", 0.2)
    ds.write("mavlink.gps.lat", 48.0)
    ns = ds.read_namespace("mavlink.attitude")
    assert set(ns.keys()) == {"mavlink.attitude.roll", "mavlink.attitude.pitch"}
    assert ns["mavlink.attitude.roll"] == 0.1


def test_read_namespace_empty():
    ds = DataStore()
    ds.write("other.key", 1)
    assert ds.read_namespace("mavlink") == {}


def test_overwrite():
    ds = DataStore()
    ds.write("k", 1)
    ds.write("k", 2)
    assert ds.read("k") == 2


def test_subscribe_callback():
    ds = DataStore()
    received = []
    ds.subscribe("sensor.value", lambda k, v: received.append((k, v)))
    ds.write("sensor.value", 3.14)
    assert received == [("sensor.value", 3.14)]


def test_subscribe_callback_not_called_for_other_keys():
    ds = DataStore()
    called = []
    ds.subscribe("a", lambda k, v: called.append(v))
    ds.write("b", 99)
    assert called == []


def test_thread_safety():
    """Concurrent writes from multiple threads must not corrupt the store."""
    ds = DataStore()
    errors = []

    def writer(thread_id: int) -> None:
        for i in range(100):
            try:
                ds.write(f"thread.{thread_id}.value", i)
            except Exception as e:
                errors.append(e)

    threads = [threading.Thread(target=writer, args=(t,)) for t in range(10)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    assert errors == [], f"Thread-safety violations: {errors}"
    for tid in range(10):
        val = ds.read(f"thread.{tid}.value")
        assert val is not None
