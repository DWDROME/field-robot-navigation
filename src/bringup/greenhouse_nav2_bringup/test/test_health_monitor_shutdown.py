import pytest
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions

from greenhouse_nav2_bringup import health_monitor


class FakeNode:
    def __init__(self):
        self.destroyed = False

    def destroy_node(self):
        self.destroyed = True


def _install_fakes(monkeypatch, spin_exception):
    node = FakeNode()
    init_calls = []
    signal_calls = []
    shutdown_calls = []
    monkeypatch.setattr(
        health_monitor.rclpy,
        "init",
        lambda **kwargs: init_calls.append(kwargs),
    )
    monkeypatch.setattr(health_monitor, "BringupHealthMonitor", lambda: node)

    def raise_from_spin(_node, *, timeout_sec):
        assert timeout_sec == 0.1
        raise spin_exception

    monkeypatch.setattr(health_monitor.rclpy, "spin_once", raise_from_spin)
    monkeypatch.setattr(
        health_monitor.signal,
        "signal",
        lambda signum, handler: signal_calls.append((signum, handler)),
    )
    monkeypatch.setattr(
        health_monitor,
        "try_shutdown",
        lambda: shutdown_calls.append(True),
    )
    return node, init_calls, signal_calls, shutdown_calls


def test_external_context_shutdown_is_a_clean_exit(monkeypatch):
    node, init_calls, signal_calls, shutdown_calls = _install_fakes(
        monkeypatch, ExternalShutdownException()
    )

    assert health_monitor.main([]) == 0
    assert node.destroyed
    assert init_calls == [
        {"args": [], "signal_handler_options": SignalHandlerOptions.NO}
    ]
    assert [signum for signum, _handler in signal_calls] == [
        health_monitor.signal.SIGINT,
        health_monitor.signal.SIGTERM,
    ]
    assert shutdown_calls == [True]


def test_unexpected_executor_error_propagates_after_cleanup(monkeypatch):
    node, init_calls, signal_calls, shutdown_calls = _install_fakes(
        monkeypatch, RuntimeError("unexpected executor failure")
    )

    with pytest.raises(RuntimeError, match="unexpected executor failure"):
        health_monitor.main([])

    assert node.destroyed
    assert init_calls == [
        {"args": [], "signal_handler_options": SignalHandlerOptions.NO}
    ]
    assert [signum for signum, _handler in signal_calls] == [
        health_monitor.signal.SIGINT,
        health_monitor.signal.SIGTERM,
    ]
    assert shutdown_calls == [True]
