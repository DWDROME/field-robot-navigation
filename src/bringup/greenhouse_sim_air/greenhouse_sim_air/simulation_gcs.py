"""Maintain the automated SITL ground station and log MAVLink health messages.

PX4 keeps its normal arming and datalink checks. This local endpoint supplies
the GCS connection for unattended simulation; flight commands still use DDS.
It is not a hardware ground station or a replacement for a human pilot.
"""
import time
from pymavlink import mavutil


def main():
    connection = mavutil.mavlink_connection(
        'udpout:127.0.0.1:18570', source_system=255, source_component=190)
    next_heartbeat = 0.0
    try:
        while True:
            now = time.monotonic()
            if now >= next_heartbeat:
                connection.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
                next_heartbeat = now + 1.0
            message = connection.recv_match(blocking=True, timeout=0.1)
            if message and message.get_type() == 'STATUSTEXT':
                print(f'PX4 severity={message.severity}: {message.text}', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        connection.close()


if __name__ == '__main__':
    main()
