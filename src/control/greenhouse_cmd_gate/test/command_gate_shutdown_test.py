#!/usr/bin/env python3

import os
from pathlib import Path
import selectors
import signal
import subprocess
import sys
import time


def main() -> int:
    if len(sys.argv) != 2:
        raise SystemExit("usage: command_gate_shutdown_test.py <executable>")

    executable = Path(sys.argv[1])
    command = [
        str(executable),
        "--ros-args",
        "-p", "source_topic:=/cmd_vel/nav",
        "-p", "output_topic:=/cmd_vel",
        "-p", "output_frame:=base_link",
        "-p", "qualification_scope:=controlled_shutdown_test",
        "-p", "publish_rate_hz:=50.0",
        "-p", "max_linear_velocity:=1.0",
        "-p", "max_angular_velocity:=1.0",
        "-p", "max_linear_acceleration:=2.0",
        "-p", "max_angular_acceleration:=2.0",
        "-p", "watchdog_timeout_sec:=0.5",
        "-p", "max_command_age_sec:=0.2",
        "-p", "future_tolerance_sec:=0.05",
        "-p", "auto_recover_after_watchdog:=true",
    ]
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    output = ""
    selector = selectors.DefaultSelector()
    assert process.stdout is not None
    selector.register(process.stdout, selectors.EVENT_READ)
    try:
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if process.poll() is not None:
                break
            for key, _ in selector.select(timeout=0.1):
                line = key.fileobj.readline()
                output += line
                if "gate source=" in line:
                    break
            if "gate source=" in output:
                break
        else:
            raise AssertionError("command gate did not report readiness")

        if process.poll() is not None:
            raise AssertionError(
                f"command gate exited before SIGINT: {process.returncode}\n{output}"
            )

        os.killpg(process.pid, signal.SIGINT)
        remaining, _ = process.communicate(timeout=10.0)
        output += remaining
    except BaseException:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            remaining, _ = process.communicate(timeout=5.0)
            output += remaining
        raise
    finally:
        selector.close()

    if process.returncode != 0:
        raise AssertionError(
            f"command gate SIGINT exit was {process.returncode}\n{output}"
        )
    forbidden = ("[FATAL]", "context is invalid", "process has died")
    for marker in forbidden:
        if marker in output:
            raise AssertionError(f"shutdown output contains {marker!r}\n{output}")
    print("command gate controlled SIGINT shutdown passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
