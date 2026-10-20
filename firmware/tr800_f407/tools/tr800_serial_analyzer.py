#!/usr/bin/env python3
"""Analyze TR800 UART4 telemetry frames.

Current firmware configuration is:
  - PROTOCOL_25 = 0
  - TR800_MOTOR_TELEMETRY_DEBUG = 1

So UART4 currently sends a 24-byte frame at 40 Hz:
  0: 0x7B
  1: Flag_Stop
  2..7: int16 X/Y/Z speed, scaled by 1000 in firmware
  8..19: motor diagnostics, not normal accel/gyro fields
  20..21: uint16 battery voltage in mV
  22: XOR checksum over bytes 0..21
  23: 0x7D
"""

from __future__ import annotations

import argparse
import csv
import statistics
import sys
import time
from dataclasses import dataclass
from typing import BinaryIO, Iterable

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover - depends on host setup
    raise SystemExit(
        "pyserial is required. Install it with: python -m pip install pyserial"
    ) from exc


FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D
FRAME_SIZE = 24
CONTROL_FRAME_SIZE = 11


@dataclass
class Telemetry:
    timestamp: float
    flag_stop: int
    x_mm_s: int
    y_mm_s: int
    z_mrad_s: int
    diag_0: int
    diag_1: int
    diag_2: int
    diag_3: int
    diag_4: int
    diag_5: int
    voltage_v: float
    checksum_ok: bool


def int16_be(frame: bytes, offset: int) -> int:
    return int.from_bytes(frame[offset : offset + 2], "big", signed=True)


def uint16_be(frame: bytes, offset: int) -> int:
    return int.from_bytes(frame[offset : offset + 2], "big", signed=False)


def checksum(frame: bytes) -> int:
    value = 0
    for byte in frame[:22]:
        value ^= byte
    return value & 0xFF


def checksum_prefix(data: bytes | bytearray, count: int) -> int:
    value = 0
    for byte in data[:count]:
        value ^= byte
    return value & 0xFF


def encode_i16_milli(value: float) -> bytes:
    raw = int(round(value * 1000.0))
    raw = max(-32768, min(32767, raw))
    return raw.to_bytes(2, "big", signed=True)


def build_control_frame(x_m_s: float, y_m_s: float, z_rad_s: float) -> bytes:
    """Build the 11-byte UART4 command accepted by UART4_IRQHandler."""
    frame = bytearray(CONTROL_FRAME_SIZE)
    frame[0] = FRAME_HEADER
    frame[1] = 0
    frame[2] = 0
    frame[3:5] = encode_i16_milli(x_m_s)
    frame[5:7] = encode_i16_milli(y_m_s)
    frame[7:9] = encode_i16_milli(z_rad_s)
    frame[9] = checksum_prefix(frame, 9)
    frame[10] = FRAME_TAIL
    return bytes(frame)


def pop_telemetry_frames(buffer: bytearray) -> Iterable[bytes]:
    while True:
        try:
            start = buffer.index(FRAME_HEADER)
        except ValueError:
            buffer.clear()
            break

        if start:
            del buffer[:start]

        if len(buffer) < FRAME_SIZE:
            break

        frame = bytes(buffer[:FRAME_SIZE])
        if frame[-1] == FRAME_TAIL:
            del buffer[:FRAME_SIZE]
            yield frame
        else:
            del buffer[0]


def decode_frame(frame: bytes, timestamp: float) -> Telemetry:
    return Telemetry(
        timestamp=timestamp,
        flag_stop=frame[1],
        x_mm_s=int16_be(frame, 2),
        y_mm_s=int16_be(frame, 4),
        z_mrad_s=int16_be(frame, 6),
        diag_0=int16_be(frame, 8),
        diag_1=int16_be(frame, 10),
        diag_2=int16_be(frame, 12),
        diag_3=int16_be(frame, 14),
        diag_4=int16_be(frame, 16),
        diag_5=int16_be(frame, 18),
        voltage_v=uint16_be(frame, 20) / 1000.0,
        checksum_ok=checksum(frame) == frame[22],
    )


def decode_mode_code(mode_code: int) -> tuple[int, int, int]:
    system_state = mode_code // 100
    requested_mode = (mode_code // 10) % 10
    active_mode = mode_code % 10
    return system_state, requested_mode, active_mode


def format_control_diag(pages: dict[int, Telemetry]) -> str:
    page1 = pages.get(1)
    page2 = pages.get(2)
    page3 = pages.get(3)

    parts: list[str] = []
    if page1:
        parts.append(
            "rc "
            f"LX={page1.diag_1:4d} LY={page1.diag_2:4d} "
            f"RX={page1.diag_3:4d} RY={page1.diag_4:4d} "
            f"Speed={page1.diag_5:4d}"
        )
    else:
        parts.append("rc missing")

    if page2:
        system_state, requested_mode, active_mode = decode_mode_code(page2.diag_5)
        parts.append(
            "cmd "
            f"moveX={page2.diag_1:5d} moveZ={page2.diag_2:5d} "
            f"tarA={page2.diag_3:5d} tarB={page2.diag_4:5d} "
            f"sys={system_state} req={requested_mode} act={active_mode}"
        )
    else:
        parts.append("cmd missing")

    if page3:
        parts.append(
            "loop "
            f"encA={page3.diag_1:5d} encB={page3.diag_2:5d} "
            f"pwmA={page3.diag_3:6d} pwmB={page3.diag_4:6d} "
            f"fault={page3.diag_5:3d}"
        )
    else:
        parts.append("loop missing")

    return " | ".join(parts)


def read_frames(stream: BinaryIO) -> Iterable[bytes]:
    buffer = bytearray()
    while True:
        chunk = stream.read(128)
        if not chunk:
            continue
        buffer.extend(chunk)

        while True:
            try:
                start = buffer.index(FRAME_HEADER)
            except ValueError:
                buffer.clear()
                break

            if start:
                del buffer[:start]

            if len(buffer) < FRAME_SIZE:
                break

            frame = bytes(buffer[:FRAME_SIZE])
            if frame[-1] == FRAME_TAIL:
                del buffer[:FRAME_SIZE]
                yield frame
            else:
                del buffer[0]


def diagnosis(samples: list[Telemetry]) -> str:
    good = [sample for sample in samples if sample.checksum_ok]
    if len(good) < 5:
        return "valid_samples_low"

    xs = [sample.x_mm_s for sample in good]
    zs = [sample.z_mrad_s for sample in good]
    avg_x = statistics.fmean(xs)
    avg_abs_x = statistics.fmean(abs(value) for value in xs)
    avg_z = statistics.fmean(zs)
    avg_abs_z = statistics.fmean(abs(value) for value in zs)

    if avg_abs_x < 20 and avg_abs_z < 20:
        return "nearly_stationary"
    if avg_abs_x < 50 and avg_abs_z >= 80:
        return "encoder_speeds_cancel_or_vehicle_spinning"
    if avg_abs_x >= 80 and avg_abs_z >= avg_abs_x * 0.8:
        return "strong_unwanted_yaw_or_left_right_speed_mismatch"
    if avg_abs_x >= 80 and avg_abs_z < avg_abs_x * 0.35:
        return "mostly_forward_or_backward"
    if avg_z:
        return "mixed_motion"
    return "unknown"


def inferred_track_speeds(sample: Telemetry, wheel_spacing_m: float) -> tuple[float, float]:
    """Infer A(left)/B(right) encoder speed in mm/s from reported X/Z speed."""
    half_delta = sample.z_mrad_s * wheel_spacing_m / 2.0
    left_a = sample.x_mm_s - half_delta
    right_b = sample.x_mm_s + half_delta
    return left_a, right_b


def print_ports() -> None:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found by pyserial.")
        return
    for port in ports:
        print(f"{port.device}\t{port.description}")


def run(args: argparse.Namespace) -> int:
    if args.list_ports:
        print_ports()
        return 0

    if not args.port:
        print("Missing --port. Use --list-ports first.", file=sys.stderr)
        return 2

    csv_file = None
    writer = None
    if args.csv:
        csv_file = open(args.csv, "w", newline="", encoding="utf-8")
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "time_s",
                "flag_stop",
                "x_mm_s",
                "y_mm_s",
                "z_mrad_s",
                "diag_0",
                "diag_1",
                "diag_2",
                "diag_3",
                "diag_4",
                "diag_5",
                "voltage_v",
                "checksum_ok",
                "inferred_enc_a_mm_s",
                "inferred_enc_b_mm_s",
            ]
        )

    sending = any(
        value is not None for value in (args.send_x, args.send_y, args.send_z)
    )
    if sending and args.send_duration <= 0:
        print("When sending commands, --send-duration must be > 0.", file=sys.stderr)
        return 2

    send_x = args.send_x if args.send_x is not None else 0.0
    send_y = args.send_y if args.send_y is not None else 0.0
    send_z = args.send_z if args.send_z is not None else 0.0
    control_frame = build_control_frame(send_x, send_y, send_z)
    zero_frame = build_control_frame(0.0, 0.0, 0.0)

    duration = args.duration
    if sending and duration <= 0:
        duration = args.send_start_delay + args.send_duration + 1.0

    start_time = time.monotonic()
    last_print = start_time
    next_send = start_time + args.send_start_delay
    zero_sent = False
    total = 0
    valid = 0
    window: list[Telemetry] = []
    control_pages: dict[int, Telemetry] = {}
    rx_buffer = bytearray()

    try:
        with serial.Serial(args.port, args.baud, timeout=0.02) as stream:
            print(f"Listening on {args.port} @ {args.baud}. Press Ctrl+C to stop.")
            if sending:
                print(
                    "Sending UART4 control: "
                    f"x={send_x:.3f}m/s y={send_y:.3f}m/s z={send_z:.3f}rad/s "
                    f"start_delay={args.send_start_delay:.2f}s "
                    f"duration={args.send_duration:.2f}s rate={args.send_rate:.1f}Hz "
                    f"frame={control_frame.hex(' ')}"
                )

            while True:
                now = time.monotonic()
                elapsed = now - start_time

                if sending:
                    send_begin = args.send_start_delay
                    send_end = args.send_start_delay + args.send_duration
                    if elapsed < send_begin:
                        pass
                    elif elapsed < send_end:
                        if now >= next_send:
                            stream.write(control_frame)
                            next_send = now + 1.0 / args.send_rate
                    elif args.zero_after:
                        if now >= next_send:
                            stream.write(zero_frame)
                            next_send = now + 1.0 / args.send_rate
                            if not zero_sent:
                                zero_sent = True
                                print(f"Sending zero command frame={zero_frame.hex(' ')}")

                chunk = stream.read(256)
                if chunk:
                    rx_buffer.extend(chunk)

                for frame in pop_telemetry_frames(rx_buffer):
                    sample = decode_frame(frame, time.monotonic() - start_time)
                    total += 1
                    if sample.checksum_ok:
                        valid += 1
                        if args.diag_control and sample.diag_0 in (1, 2, 3):
                            control_pages[sample.diag_0] = sample
                    window.append(sample)
                    if len(window) > args.window:
                        del window[0 : len(window) - args.window]

                    if writer:
                        enc_a_mm_s, enc_b_mm_s = inferred_track_speeds(
                            sample, args.wheel_spacing
                        )
                        writer.writerow(
                            [
                                f"{sample.timestamp:.3f}",
                                sample.flag_stop,
                                sample.x_mm_s,
                                sample.y_mm_s,
                                sample.z_mrad_s,
                                sample.diag_0,
                                sample.diag_1,
                                sample.diag_2,
                                sample.diag_3,
                                sample.diag_4,
                                sample.diag_5,
                                f"{sample.voltage_v:.3f}",
                                int(sample.checksum_ok),
                                f"{enc_a_mm_s:.1f}",
                                f"{enc_b_mm_s:.1f}",
                            ]
                        )

                    now = time.monotonic()
                    if now - last_print >= args.interval:
                        enc_a_mm_s, enc_b_mm_s = inferred_track_speeds(
                            sample, args.wheel_spacing
                        )
                        elapsed = max(now - start_time, 0.001)
                        rate = total / elapsed
                        valid_rate = 100.0 * valid / max(total, 1)
                        state = diagnosis(window)
                        line = (
                            f"t={sample.timestamp:7.2f}s "
                            f"fps={rate:5.1f} valid={valid_rate:5.1f}% "
                            f"stop={sample.flag_stop} "
                            f"x={sample.x_mm_s:6d}mm/s "
                            f"z={sample.z_mrad_s:6d}mrad/s "
                            f"encA={enc_a_mm_s:7.1f} "
                            f"encB={enc_b_mm_s:7.1f} "
                        )
                        if args.diag_control:
                            line += format_control_diag(control_pages) + " "
                        elif args.diag_rc:
                            mode_code = sample.diag_5
                            system_state, requested_mode, active_mode = decode_mode_code(
                                mode_code
                            )
                            line += (
                                f"ch1={sample.diag_0:4d} "
                                f"ch2={sample.diag_1:4d} "
                                f"ch3={sample.diag_2:4d} "
                                f"ch4={sample.diag_3:4d} "
                                f"ch5={sample.diag_4:4d} "
                                f"sys={system_state} req={requested_mode} act={active_mode} "
                            )
                        else:
                            line += (
                                f"ch2={sample.diag_0:4d} "
                                f"ccrA={sample.diag_1:6d} "
                                f"ccrB={sample.diag_2:6d} "
                                f"PA2PA3={sample.diag_3:2d} "
                                f"PA5PA4={sample.diag_4:2d} "
                                f"fault={sample.diag_5:3d} "
                            )
                        line += f"vbat={sample.voltage_v:5.2f}V diag={state}"
                        print(line)
                        last_print = now

                if duration and elapsed >= duration:
                    break
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        if csv_file:
            csv_file.close()

    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list-ports", action="store_true", help="list serial ports")
    parser.add_argument("--port", help="serial port, for example COM4 or /dev/ttyS4")
    parser.add_argument("--baud", type=int, default=115200, help="baud rate")
    parser.add_argument("--duration", type=float, default=0.0, help="seconds to run")
    parser.add_argument("--interval", type=float, default=0.5, help="print interval")
    parser.add_argument("--window", type=int, default=40, help="diagnosis sample window")
    parser.add_argument(
        "--diag-rc",
        action="store_true",
        help="interpret diag_0..diag_5 fields as RC channel/mode diagnostics",
    )
    parser.add_argument(
        "--diag-control",
        action="store_true",
        help="interpret paged diag fields as control-chain diagnostics",
    )
    parser.add_argument(
        "--diag-raw",
        action="store_true",
        help="deprecated alias; current firmware already defaults to motor diagnostics",
    )
    parser.add_argument(
        "--wheel-spacing",
        type=float,
        default=0.564,
        help="track spacing in meters, default matches current TS50 config",
    )
    parser.add_argument("--send-x", type=float, help="send target X speed in m/s")
    parser.add_argument("--send-y", type=float, help="send target Y speed in m/s")
    parser.add_argument("--send-z", type=float, help="send target Z speed in rad/s")
    parser.add_argument(
        "--send-duration",
        type=float,
        default=0.0,
        help="seconds to keep sending the non-zero command",
    )
    parser.add_argument(
        "--send-rate",
        type=float,
        default=20.0,
        help="UART4 command send rate in Hz",
    )
    parser.add_argument(
        "--send-start-delay",
        type=float,
        default=0.0,
        help="seconds to wait after opening the port before sending commands",
    )
    parser.add_argument(
        "--no-zero-after",
        dest="zero_after",
        action="store_false",
        help="do not send a final zero-speed command after --send-duration",
    )
    parser.set_defaults(zero_after=True)
    parser.add_argument("--csv", help="optional CSV output path")
    return run(parser.parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
