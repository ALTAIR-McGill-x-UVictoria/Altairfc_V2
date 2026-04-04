from __future__ import annotations

import threading
import time

import pytest

from config.settings import SystemConfig, SerialPortConfig, TaskConfig
from core.datastore import DataStore
from core.scheduler import TaskScheduler
from core.task_base import BaseTask, TaskState


def _make_config(tasks: dict | None = None) -> SystemConfig:
    default_tasks = {
        "mock": TaskConfig(name="mock", enabled=True, period_s=0.01),
    }
    return SystemConfig(
        mavlink=SerialPortConfig(port="/dev/null", baud=115200),
        telemetry=SerialPortConfig(port="/dev/null", baud=57600),
        vesc=SerialPortConfig(port="/dev/null", baud=115200),
        tasks=tasks or default_tasks,
        monitor_interval_s=0.1,
    )


class MockTask(BaseTask):
    """A simple task that counts execute() calls."""

    def __init__(self, name: str, datastore: DataStore, fail_on_execute: bool = False) -> None:
        super().__init__(name, period_s=0.01, datastore=datastore)
        self.setup_called = False
        self.teardown_called = False
        self.execute_count = 0
        self.fail_on_execute = fail_on_execute

    def setup(self) -> None:
        self.setup_called = True

    def execute(self) -> None:
        self.execute_count += 1
        if self.fail_on_execute and self.execute_count >= 2:
            raise RuntimeError("Intentional failure")

    def teardown(self) -> None:
        self.teardown_called = True


def test_task_start_stop():
    ds = DataStore()
    config = _make_config()
    scheduler = TaskScheduler(ds, config)
    task = MockTask("mock", ds)
    scheduler.register(task)
    scheduler.start_all()

    time.sleep(0.05)
    assert task.is_alive
    assert task.state == TaskState.RUNNING
    assert task.execute_count > 0

    scheduler.stop_all()
    assert not task.is_alive
    assert task.setup_called
    assert task.teardown_called


def test_disabled_task_not_registered():
    ds = DataStore()
    config = _make_config(
        tasks={"mock": TaskConfig(name="mock", enabled=False, period_s=0.01)}
    )
    scheduler = TaskScheduler(ds, config)
    task = MockTask("mock", ds)
    scheduler.register(task)  # should be skipped silently
    assert scheduler.get_task("mock") is None


def test_duplicate_registration_raises():
    ds = DataStore()
    config = _make_config()
    scheduler = TaskScheduler(ds, config)
    scheduler.register(MockTask("mock", ds))
    with pytest.raises(ValueError, match="already registered"):
        scheduler.register(MockTask("mock", ds))


def test_failed_task_triggers_shutdown():
    ds = DataStore()
    config = _make_config(
        tasks={"mock": TaskConfig(name="mock", enabled=True, period_s=0.01)}
    )
    scheduler = TaskScheduler(ds, config)
    task = MockTask("mock", ds, fail_on_execute=True)
    scheduler.register(task)
    scheduler.start_all()

    # The monitor runs every 0.1s; give it time to detect the failure
    triggered = scheduler.shutdown_event.wait(timeout=2.0)
    assert triggered, "Shutdown event was not set after task failure"
    scheduler.stop_all()


def test_get_task():
    ds = DataStore()
    config = _make_config()
    scheduler = TaskScheduler(ds, config)
    task = MockTask("mock", ds)
    scheduler.register(task)
    assert scheduler.get_task("mock") is task
    assert scheduler.get_task("nonexistent") is None
