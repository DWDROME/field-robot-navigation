#!/usr/bin/env python3
"""Capture simulation screenshots from the Gazebo scene camera.

The simulation_camera model is spawned into the running world; this node
subscribes to its image topic through the ros_gz bridge, waits for a
non-degenerate frame (rejects black/blank images), and writes a PNG with a
JSON metadata sidecar recording the run id, world/robot, camera pose and
simulation time.

Usage:
  python3 capture_simulation_image.py --run-id maize_field_husky_a200_r1 \
      --world maize_field --robot husky_a200 --output docs/media/simulation
"""
import argparse
import json
import time
from pathlib import Path
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


class Capture(Node):
    def __init__(self, topic: str):
        super().__init__('simulation_capture')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.frame = None
        self.create_subscription(Image, topic, self.on_image, qos)

    def on_image(self, msg: Image) -> None:
        self.frame = msg

    def wait_frame(self, timeout: float):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=1.0)
            if self.frame is not None:
                frame, self.frame = self.frame, None
                return frame
        return None


def frame_metrics(msg: Image):
    image = CvBridge().imgmsg_to_cv2(msg, desired_encoding='bgr8')
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return float(gray.mean()), float(gray.var())


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--world', required=True)
    parser.add_argument('--robot', required=True)
    parser.add_argument('--topic', default='/sim_camera/image')
    parser.add_argument('--output', default='docs/media/simulation')
    parser.add_argument('--timeout', type=float, default=60.0)
    camera = parser.add_mutually_exclusive_group(required=True)
    camera.add_argument('--camera-pose',
                        help='Actual launched x,y,z,roll,pitch,yaw camera pose')
    camera.add_argument('--camera-config', type=Path,
                        help='camera.json emitted by the running simulation entry')
    parser.add_argument('--status', default='captured')
    parser.add_argument('--kind', choices=['task','panorama'], default='task')
    parser.add_argument('--wait-marker', type=Path,
                        help='Wait for the actual mission start/takeoff marker before subscribing')
    args = parser.parse_args()
    configuration = None
    if args.camera_config:
        deadline=time.monotonic()+args.timeout
        while not args.camera_config.exists() and time.monotonic()<deadline:
            time.sleep(.1)
        if not args.camera_config.exists():
            parser.error('Running entry did not emit camera.json')
        configuration = json.loads(args.camera_config.read_text())
        if configuration['world'] != args.world or configuration['robot'] != args.robot:
            parser.error('Camera configuration world/robot mismatch')
        if configuration['topic'] != args.topic:
            parser.error('Camera configuration topic mismatch')
        args.camera_pose = configuration['pose']

    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    mission_state = None
    if args.wait_marker:
        deadline = time.monotonic()+args.timeout
        while not args.wait_marker.exists() and time.monotonic()<deadline:
            time.sleep(.1)
        if not args.wait_marker.exists():
            print(json.dumps({'run_id':args.run_id,'result':'mission-marker-timeout'}))
            return 1
        mission_state = json.loads(args.wait_marker.read_text())

    rclpy.init()
    node = Capture(args.topic)
    try:
        deadline = time.monotonic() + args.timeout
        frame = None
        while time.monotonic() < deadline:
            candidate = node.wait_frame(5.0)
            if candidate is None:
                continue
            mean, variance = frame_metrics(candidate)
            if candidate.width >= 1920 and candidate.height >= 1080 and mean > 3.0 and variance > 4.0:
                frame = (candidate, mean, variance)
                break
        if frame is None:
            print(json.dumps({'run_id': args.run_id, 'result': 'no-valid-frame'}))
            return 1

        msg, mean, variance = frame
        basename = f'{args.run_id}_{args.kind}'
        png = output / f'{basename}.png'
        image = CvBridge().imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if not cv2.imwrite(str(png), image):
            raise OSError(f'Failed to save {png}')
        compressed = output / f'{basename}.jpg'
        if not cv2.imwrite(str(compressed), image, [cv2.IMWRITE_JPEG_QUALITY, 88]):
            raise OSError(f'Failed to save {compressed}')

        metadata = {
            'run_id': args.run_id,
            'kind': args.kind,
            'world': args.world,
            'robot': args.robot,
            'camera_pose': args.camera_pose,
            'view': configuration['view'] if configuration else 'manual',
            'capture_source': 'Gazebo OGRE2/EGL scene sensor',
            'sim_time': {'sec': msg.header.stamp.sec, 'nanosec': msg.header.stamp.nanosec},
            'resolution': f'{msg.width}x{msg.height}',
            'mean_brightness': round(mean, 2),
            'variance': round(variance, 2),
            'status': args.status,
            'mission_state_at_capture_start': mission_state,
            'image': png.name,
            'compressed_image': compressed.name,
            'visual_review': 'pending; brightness checks do not prove model visibility or framing',
        }
        (output / f'{basename}.json').write_text(json.dumps(metadata, indent=2))
        print(json.dumps({'run_id': args.run_id, 'result': 'saved', 'image': str(png)}))
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
