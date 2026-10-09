"""Fly a PX4 SITL x500 through an offboard waypoint inspection mission.

Mission sequence: wait for the vehicle to be ready (local position valid,
GPS/EKF healthy) -> arm -> switch to offboard -> take off -> hold -> visit
each waypoint -> return -> land -> disarm. Every step is executed through
real PX4 uORB topics over uXRCE-DDS; no entity pose is edited and no
animation substitutes for motion.

Coordinates: waypoints are configured in the site's local NED frame
(x north, y east, z down, origin at the spawn point) and converted to the
PX4 NED setpoints. Results (waypoint reach time, position error, altitude
error, failures) are written as JSON when the mission ends.

Usage:
  ros2 run greenhouse_sim_air air_mission --ros-args \
      -p site:=orchard -p output:=/tmp/simulation/air_orchard.json
"""
import json
import math
import sys
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)

from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition,
                          VehicleStatus, VehicleLandDetected, VehicleAttitude)
from rosgraph_msgs.msg import Clock

# PX4 command ids
VEHICLE_CMD_DO_SET_MODE = 176
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
PX4_CUSTOM_MAIN_MODE_AUTO = 4
PX4_CUSTOM_SUB_MODE_AUTO_LAND = 6

ARMING_STATE_ARMED = 2
NAVIGATION_STATE_OFFBOARD = 14

WAYPOINT_RADIUS = 1.5       # m, horizontal acceptance radius
ALTITUDE_TOLERANCE = 0.6    # m
WAYPOINT_TIMEOUT = 60.0     # s per waypoint
TAKEOFF_TIMEOUT = 45.0      # s
LAND_TIMEOUT = 60.0         # s
RETURN_RADIUS = 0.3        # m, center before AUTO_LAND
RETURN_SPEED = 0.3         # m/s
RETURN_SETTLE_SIM = 2.0    # simulated seconds


def px4_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def output_topic(name, message_type):
    version = message_type.MESSAGE_VERSION
    return f'/fmu/out/{name}' + (f'_v{version}' if version else '')


class AirMission(Node):
    def __init__(self) -> None:
        super().__init__('air_mission')
        self.declare_parameter('site', 'orchard')
        self.declare_parameter('output', '')
        self.declare_parameter('capture_marker', '')

        site = self.get_parameter('site').value
        config_path = Path(__file__).resolve().parents[1] / 'config' / 'x500_sites.yaml'
        if not config_path.is_file():
            from ament_index_python.packages import get_package_share_directory
            config_path = (Path(get_package_share_directory('greenhouse_sim_air'))
                           / 'config' / 'x500_sites.yaml')
        with config_path.open() as handle:
            sites = yaml.safe_load(handle)
        if site not in sites:
            raise ValueError(f'site {site!r} not in {sorted(sites)}')
        self.site = sites[site]
        self.waypoints = self.site['waypoints']

        qos = px4_qos()
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.create_subscription(
            VehicleLocalPosition, output_topic('vehicle_local_position', VehicleLocalPosition),
            self.on_local_position, qos)
        self.create_subscription(
            VehicleStatus, output_topic('vehicle_status', VehicleStatus),
            self.on_vehicle_status, qos)
        self.create_subscription(VehicleLandDetected, output_topic('vehicle_land_detected', VehicleLandDetected),
                                 self.on_land_detected, qos)
        self.create_subscription(VehicleAttitude, output_topic('vehicle_attitude', VehicleAttitude),
                                 self.on_attitude, qos)
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)

        self.local_position = None
        self.vehicle_status = None
        self.land_detected = None
        self.last_position_receipt = -math.inf
        self.last_status_receipt = -math.inf
        self.last_land_receipt = -math.inf
        self.last_attitude_receipt = -math.inf
        self.last_clock_receipt = -math.inf
        self.sim_time = self.first_sim_time = None
        self.started = time.monotonic()
        self.hover_stable_since = None
        self.return_stable_since = None
        self.cruise_target = self.cruise_peak_altitude = None
        self.results = {'site': site, 'waypoints': [], 'steps': [],
                        'result': 'incomplete', 'reason': None,
                        'thresholds': {'waypoint_radius_m': WAYPOINT_RADIUS,
                                       'altitude_tolerance_m': ALTITUDE_TOLERANCE,
                                       'hover_speed_mps': .5, 'hover_stable_sim_s': 3.0,
                                       'return_radius_m': RETURN_RADIUS, 'return_speed_mps': RETURN_SPEED,
                                       'return_stable_sim_s': RETURN_SETTLE_SIM,
                                       'roll_pitch_limit_rad': math.pi/4,
                                       'telemetry_timeout_wall_s': 2.0},
                        'max_abs_roll_rad': 0.0, 'max_abs_pitch_rad': 0.0,
                        'position_samples': 0, 'minimum_cruise_altitude_m': None,
                        'hover_samples': 0, 'hover_max_xy_error_m': 0.0,
                        'hover_max_altitude_error_m': 0.0,
                        'max_cruise_height_loss_m': 0.0, 'path_length_m': 0.0,
                        'landed': False}
        self.output = self.get_parameter('output').value
        self.trace = None
        if self.output:
            trace_path = Path(self.output).with_suffix('.jsonl')
            trace_path.parent.mkdir(parents=True, exist_ok=True)
            self.trace = trace_path.open('w')
            self.results['trace'] = str(trace_path)
        self.timer = self.create_timer(0.1, self.tick)
        self.phase = 'waiting-ready'
        self.phase_started = time.monotonic()
        self.waypoint_index = 0
        # Climb vertically to the checked cruise height before horizontal
        # travel. Return at the same height before descending over home.
        self.hold_altitude = float(self.site['cruise_altitude'])
        self.home = None

    # -- callbacks ---------------------------------------------------------
    def on_local_position(self, msg: VehicleLocalPosition) -> None:
        if not all(math.isfinite(value) for value in (msg.x, msg.y, msg.z)):
            return
        if self.local_position is not None and self.home is not None and msg.xy_valid and msg.z_valid:
            previous = self.local_position
            self.results['path_length_m'] += math.dist(
                (msg.x, msg.y, msg.z), (previous.x, previous.y, previous.z))
        self.local_position = msg
        self.last_position_receipt = time.monotonic()
        self.results['position_samples'] += 1
        if self.home is not None and self.phase in {'hold', 'waypoint', 'return'} and msg.z_valid:
            altitude = self.home[2]-msg.z
            previous = self.results['minimum_cruise_altitude_m']
            self.results['minimum_cruise_altitude_m'] = altitude if previous is None else min(previous, altitude)
            if self.cruise_target is not None:
                self.cruise_peak_altitude = max(self.cruise_peak_altitude, altitude)
                loss = min(self.cruise_peak_altitude, self.cruise_target)-altitude
                self.results['max_cruise_height_loss_m'] = max(
                    self.results['max_cruise_height_loss_m'], loss)
        if self.trace:
            self.trace.write(json.dumps({'sim_time_s': self.sim_time,
                'wall_elapsed_s': time.monotonic()-self.started, 'px4_timestamp_us': msg.timestamp,
                'phase': self.phase, 'ned_position': [msg.x,msg.y,msg.z],
                'ned_velocity': [value if math.isfinite(value) else None
                                 for value in (msg.vx,msg.vy,msg.vz)], 'xy_valid':msg.xy_valid,
                'z_valid':msg.z_valid})+'\n')

    def on_clock(self, msg: Clock) -> None:
        self.sim_time = msg.clock.sec+msg.clock.nanosec*1e-9
        self.last_clock_receipt = time.monotonic()
        if self.first_sim_time is None:
            self.first_sim_time = self.sim_time

    def on_vehicle_status(self, msg: VehicleStatus) -> None:
        self.vehicle_status = msg
        self.last_status_receipt = time.monotonic()

    def on_land_detected(self, msg: VehicleLandDetected) -> None:
        self.land_detected = msg
        self.last_land_receipt = time.monotonic()

    def on_attitude(self, msg: VehicleAttitude) -> None:
        if not all(math.isfinite(value) for value in msg.q):
            return
        self.last_attitude_receipt = time.monotonic()
        w, x, y, z = msg.q
        roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        pitch = math.asin(max(-1., min(1., 2*(w*y-z*x))))
        self.results['max_abs_roll_rad'] = max(self.results['max_abs_roll_rad'], abs(roll))
        self.results['max_abs_pitch_rad'] = max(self.results['max_abs_pitch_rad'], abs(pitch))

    # -- helpers -----------------------------------------------------------
    def now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def publish_offboard_heartbeat(self) -> None:
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.now_us()
        self.offboard_pub.publish(msg)

    def publish_setpoint(self, x: float, y: float, z: float, yaw: float = 0.0) -> None:
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = self.now_us()
        self.setpoint_pub.publish(msg)
        if self.home is not None and self.phase in {'hold', 'waypoint', 'return'}:
            altitude = self.home[2]-z
            if altitude != self.cruise_target:
                self.cruise_target = altitude
                self.cruise_peak_altitude = self.home[2]-self.local_position.z

    def send_command(self, command: int, param1: float = 0.0, param2: float = 0.0,
                     param3: float = 0.0) -> None:
        msg = VehicleCommand()
        msg.command = command
        # Generated ROS float fields must receive Python floats, including
        # integer-valued PX4 mode constants. Integer storage is not preserved
        # by the generated C conversion when field assertions are disabled.
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.now_us()
        self.command_pub.publish(msg)

    def step(self, name: str, note: str = '') -> None:
        self.get_logger().info(f'{name}: {note}')
        self.results['steps'].append({
            'step': name, 'note': note,
            'wall_elapsed_s': round(time.monotonic() - self.phase_started, 2)})
        marker = self.get_parameter('capture_marker').value
        if name == 'takeoff' and marker:
            Path(marker).write_text(json.dumps({'phase':'takeoff-reached',
                'sim_time_s':self.sim_time,'ned_position':[
                    self.local_position.x,self.local_position.y,self.local_position.z]}))

    def finish(self, result: str, reason: str | None = None) -> None:
        wall = time.monotonic()
        self.results['telemetry_age_wall_s'] = {
            name: wall-receipt if math.isfinite(receipt) else None
            for name, receipt in [('position', self.last_position_receipt),
                ('status', self.last_status_receipt), ('attitude', self.last_attitude_receipt),
                ('clock', self.last_clock_receipt), ('land', self.last_land_receipt)]}
        self.results['clock_publishers'] = self.count_publishers('/clock')
        self.results['mission_elapsed_s'] = time.monotonic()-self.started
        if self.sim_time is not None and self.first_sim_time is not None:
            self.results['sim_elapsed_s'] = self.sim_time-self.first_sim_time
            self.results['real_time_factor'] = self.results['sim_elapsed_s']/self.results['mission_elapsed_s']
        self.results['unstable'] = max(self.results['max_abs_roll_rad'], self.results['max_abs_pitch_rad']) > math.pi/4
        if result == 'pass' and self.results['unstable']:
            result,reason = 'fail','attitude exceeded 45 degrees during mission'
        if self.trace:
            self.trace.close()
            self.trace = None
        self.results['result'] = result
        self.results['reason'] = reason
        if self.output:
            Path(self.output).parent.mkdir(parents=True, exist_ok=True)
            Path(self.output).write_text(json.dumps(self.results, indent=2))
        self.get_logger().info(f'mission {result}: {reason or "ok"}')
        raise SystemExit(0 if result == 'pass' else 1)

    # -- mission state machine --------------------------------------------
    def tick(self) -> None:
        if self.phase not in {'land', 'disarm'}:
            self.publish_offboard_heartbeat()
        elapsed = time.monotonic() - self.phase_started
        pos = self.local_position
        status = self.vehicle_status

        if self.phase == 'waiting-ready':
            if self.count_publishers('/clock')>1:
                self.finish('blocked', 'more than one ROS clock publisher')
            if (pos is not None and pos.xy_valid and pos.z_valid and status is not None
                    and status.pre_flight_checks_pass and elapsed >= 2.0
                    and self.sim_time is not None
                    and self.count_publishers('/clock')==1
                    and time.monotonic()-self.last_attitude_receipt < 2.0):
                self.home = (pos.x, pos.y, pos.z)
                self.step('ready', f'nav_state={status.nav_state}')
                self.phase = 'arming'
                self.phase_started = time.monotonic()
                return
            if elapsed > 30:
                self.finish('blocked', 'vehicle not ready within 30 s')
            return

        stale = [name for name, receipt in [('position', self.last_position_receipt),
            ('status', self.last_status_receipt), ('attitude', self.last_attitude_receipt),
            ('clock', self.last_clock_receipt)] if time.monotonic()-receipt > 2.0]
        if stale:
            self.finish('fail', 'PX4 telemetry expired: '+', '.join(stale))
        if status.failure_detector_status:
            self.finish('fail', f'PX4 failure_detector_status={status.failure_detector_status}')
        if self.phase in {'takeoff', 'hold', 'waypoint', 'return'}:
            if not pos.xy_valid or not pos.z_valid:
                self.finish('fail', 'PX4 local position invalid during flight')
            if status.arming_state != ARMING_STATE_ARMED:
                self.finish('fail', 'PX4 disarmed before landing')
            if status.nav_state != NAVIGATION_STATE_OFFBOARD:
                self.finish('fail', 'PX4 left Offboard during flight')
            if max(self.results['max_abs_roll_rad'], self.results['max_abs_pitch_rad']) > math.pi/4:
                self.finish('fail', 'flight attitude exceeds 45 degrees')
        # The static clearance proof budgets these same tracking errors.
        # Enforce them continuously, including between accepted waypoints.
        if self.phase in {'hold', 'waypoint', 'return'}:
            if abs(self.home[2]-pos.z-self.hold_altitude)>ALTITUDE_TOLERANCE:
                self.finish('fail', 'cruise altitude leaves the checked clearance envelope')
        if self.phase in {'takeoff', 'hold', 'land'}:
            if math.hypot(pos.x-self.home[0],pos.y-self.home[1])>WAYPOINT_RADIUS:
                self.finish('fail', 'takeoff or landing leaves the checked home column')

        if self.phase == 'arming':
            self.publish_setpoint(*self.home)
            if status and status.arming_state == ARMING_STATE_ARMED:
                self.step('armed')
                self.phase = 'offboard'
                self.phase_started = time.monotonic()
                return
            self.send_command(VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            if elapsed > 15:
                self.finish('blocked', 'arm rejected within 15 s')
            return

        if self.phase == 'offboard':
            if status and status.nav_state == NAVIGATION_STATE_OFFBOARD:
                self.step('offboard')
                self.phase = 'takeoff'
                self.phase_started = time.monotonic()
                return
            self.publish_setpoint(pos.x, pos.y, pos.z if pos.z_valid else 0.0)
            self.send_command(VEHICLE_CMD_DO_SET_MODE, 1.0,
                              PX4_CUSTOM_MAIN_MODE_OFFBOARD)
            if elapsed > 15:
                self.finish('blocked', 'offboard mode not entered within 15 s')
            return

        if self.phase == 'takeoff':
            self.publish_setpoint(self.home[0], self.home[1], self.home[2] - self.hold_altitude)
            altitude = self.home[2] - pos.z if pos.z_valid else math.nan
            if pos.z_valid and abs(altitude - self.hold_altitude) < ALTITUDE_TOLERANCE:
                self.step('takeoff', f'altitude={altitude:.2f}')
                self.phase = 'hold'
                self.phase_started = time.monotonic()
                return
            if elapsed > TAKEOFF_TIMEOUT:
                self.finish('fail', 'takeoff altitude not reached within timeout')
            return

        if self.phase == 'hold':
            self.publish_setpoint(self.home[0], self.home[1], self.home[2] - self.hold_altitude)
            horizontal = math.hypot(pos.x-self.home[0], pos.y-self.home[1])
            altitude_error = abs(self.home[2]-pos.z-self.hold_altitude)
            stable = (horizontal < WAYPOINT_RADIUS and altitude_error < ALTITUDE_TOLERANCE
                      and math.sqrt(pos.vx**2+pos.vy**2+pos.vz**2) < .5)
            self.results['hover_samples'] += 1
            self.results['hover_max_xy_error_m'] = max(self.results['hover_max_xy_error_m'], horizontal)
            self.results['hover_max_altitude_error_m'] = max(self.results['hover_max_altitude_error_m'], altitude_error)
            if not stable:
                self.hover_stable_since = None
            elif self.hover_stable_since is None:
                self.hover_stable_since = self.sim_time
            if self.hover_stable_since is not None and self.sim_time-self.hover_stable_since >= 3.0:
                self.step('hover', 'position and velocity stable for 3 simulation seconds')
                self.phase = 'waypoint'
                self.phase_started = time.monotonic()
            if elapsed > TAKEOFF_TIMEOUT:
                self.finish('fail', 'hover stability timeout')
            return

        if self.phase == 'waypoint':
            target = self.waypoints[self.waypoint_index]
            x, y, z, yaw = target['x'], target['y'], target['z'], target.get('yaw', 0.0)
            x, y = x + self.home[0], y + self.home[1]
            self.publish_setpoint(x, y, self.home[2] - z, yaw)
            dx, dy = pos.x - x, pos.y - y
            horizontal = math.hypot(dx, dy)
            altitude_error = abs((self.home[2] - pos.z) - z)
            if horizontal < WAYPOINT_RADIUS and altitude_error < ALTITUDE_TOLERANCE:
                self.results['waypoints'].append({
                    'index': self.waypoint_index, 'x': x, 'y': y, 'z': z,
                    'horizontal_error': horizontal,
                    'altitude_error': altitude_error,
                    'elapsed_s': round(elapsed, 2)})
                self.step('waypoint', f'{self.waypoint_index} reached')
                self.waypoint_index += 1
                self.phase_started = time.monotonic()
                if self.waypoint_index >= len(self.waypoints):
                    self.phase = 'return'
                return
            if elapsed > WAYPOINT_TIMEOUT:
                self.finish('fail',
                            f'waypoint {self.waypoint_index} timeout '
                            f'(horizontal={horizontal:.2f} altitude_err={altitude_error:.2f})')
            return

        if self.phase == 'return':
            self.publish_setpoint(self.home[0], self.home[1], self.home[2] - self.hold_altitude)
            horizontal = math.hypot(pos.x-self.home[0], pos.y-self.home[1])
            speed = math.sqrt(pos.vx**2+pos.vy**2+pos.vz**2)
            # Handing a fast inbound vehicle to AUTO_LAND at the outer
            # waypoint radius can carry it outside the checked home column.
            # Keep commanding home until it is centered and stationary.
            stable = (horizontal < RETURN_RADIUS and speed < RETURN_SPEED and
                      abs(pos.z-(self.home[2]-self.hold_altitude)) < ALTITUDE_TOLERANCE)
            if not stable:
                self.return_stable_since = None
            elif self.return_stable_since is None:
                self.return_stable_since = self.sim_time
            if self.return_stable_since is not None and self.sim_time-self.return_stable_since >= RETURN_SETTLE_SIM:
                self.results.update(return_horizontal_error_m=horizontal, return_speed_mps=speed,
                                    return_stable_sim_s=self.sim_time-self.return_stable_since)
                self.step('return', 'centered and stationary for 2 simulation seconds')
                self.phase = 'land'
                self.phase_started = time.monotonic()
            if elapsed > WAYPOINT_TIMEOUT:
                self.finish('fail', 'return timeout')
            return

        if self.phase == 'land':
            self.send_command(VEHICLE_CMD_DO_SET_MODE, 1.0,
                              PX4_CUSTOM_MAIN_MODE_AUTO,
                              PX4_CUSTOM_SUB_MODE_AUTO_LAND)
            if (self.land_detected is not None and self.land_detected.landed
                    and time.monotonic()-self.last_land_receipt < 2.0
                    and status.arming_state != ARMING_STATE_ARMED):
                self.step('land', 'landing detector confirms touchdown and disarm')
                self.results['landed'] = True
                self.phase = 'disarm'
                return
            if elapsed > LAND_TIMEOUT:
                self.finish('fail', 'landing timeout')
            return

        if self.phase == 'disarm':
            self.send_command(VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
            self.step('disarm')
            self.finish('pass')


def main() -> None:
    rclpy.init()
    node = AirMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.finish('blocked', 'mission interrupted')
    finally:
        if node.trace:
            node.trace.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
