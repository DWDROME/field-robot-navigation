"""Launch the Micro-XRCE-DDS agent that bridges PX4 uORB to ROS 2.

PX4 v1.17 requires agent v2.4.3 (v3.x is documented as incompatible). The
binary is built into the simulation image at /opt/xrce-agent.
"""
import shutil
import subprocess
import sys


def main() -> int:
    agent = shutil.which('MicroXRCEAgent') or '/opt/xrce-agent/bin/MicroXRCEAgent'
    try:
        return subprocess.call([agent, 'udp4', '-p', '8888'])
    except OSError as exc:
        print(f'failed to start MicroXRCEAgent at {agent}: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
