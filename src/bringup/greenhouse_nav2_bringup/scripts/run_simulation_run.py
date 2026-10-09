#!/usr/bin/env python3
"""Run one matrix combination and record its result.

Drives the simulation_matrix launch through a fixed task, waits for the
mission to finish (or the run timeout to expire), collects observations from
the simulation topics, and writes a JSON result record plus the captured log
under the output root. The caller (run_matrix.py) iterates the nine
combinations and aggregates the records.

Ground runs: wait for fresh navigation readiness, submit the configured
normal/challenge/danger cases through /navigation/follow_waypoints, and
record odometry, physical contact sensors, mission status and timing.
Air runs: start the offboard mission node and record its JSON result.
"""
import argparse
from collections import deque
import json
import hashlib
import math
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

WORLDS = ('maize_field', 'orchard', 'pipeline')


def runtime_resources():
    return {name: path.read_text().strip() if path.is_file() else 'unreported'
            for name, path in [('cpu_max', Path('/sys/fs/cgroup/cpu.max')),
                               ('memory_max', Path('/sys/fs/cgroup/memory.max'))]} | {
        'gpu_adapter': os.environ.get('MESA_D3D12_DEFAULT_ADAPTER_NAME', 'unreported'),
        'gallium_driver': os.environ.get('GALLIUM_DRIVER', 'unreported')}


def implementation_digest():
    """Refuse resume after a relevant installed implementation/config change."""
    from ament_index_python.packages import get_package_share_directory
    digest = hashlib.sha256()
    for package in ['greenhouse_nav2_bringup','greenhouse_sim_air','greenhouse_mppi_navigation']:
        share = Path(get_package_share_directory(package))
        for path in sorted(share.rglob('*')):
            if path.is_file() and path.suffix in {'.yaml','.xml','.sdf','.py','.rviz'}:
                digest.update(f'{package}/{path.relative_to(share)}'.encode())
                digest.update(path.read_bytes())
    for path in sorted(Path(__file__).parent.glob('*.py')):
        digest.update(path.name.encode()); digest.update(path.read_bytes())
    from greenhouse_sim_air import air_mission
    digest.update(Path(air_mission.__file__).read_bytes())
    for path in [Path('/opt/greenhouse_sim/geometry.json'),
                 Path('/opt/greenhouse_far/lib/far_planner/far_planner'),
                 Path('/opt/px4/build/px4_sitl_default/bin/px4')]:
        digest.update(str(path).encode()); digest.update(path.read_bytes())
    digest.update(os.environ.get('SIM_IMAGE_ID','unreported').encode())
    digest.update(json.dumps(runtime_resources(), sort_keys=True).encode())
    return digest.hexdigest()


def run_command(args, timeout=None):
    return subprocess.run(args, capture_output=True, text=True, timeout=timeout)


def simulation_processes(root_pid, world_path):
    """Track descendants even when Gazebo creates a different process group."""
    processes = {}
    # Gazebo can replace argv with one process-title string containing all
    # flags. Require a complete path token, including in that representation.
    world_token = re.compile(rb'(?<!\S)' + re.escape(str(world_path).encode()) + rb'(?!\S)')
    for directory in Path('/proc').iterdir():
        if not directory.name.isdigit():
            continue
        try:
            fields=(directory/'stat').read_text().rsplit(')',1)[1].split()
            argv=(directory/'cmdline').read_bytes().split(b'\0')
            processes[int(directory.name)]={'parent':int(fields[1]), 'start_ticks':fields[19],
                'state':fields[0], 'world_argument':any(world_token.search(arg) for arg in argv)}
        except (OSError, ValueError, IndexError):
            continue
    owned={root_pid} | {pid for pid, value in processes.items() if value['world_argument']}
    while True:
        children={pid for pid, value in processes.items() if value['parent'] in owned}
        if children <= owned:
            break
        owned |= children
    return {pid:processes[pid] for pid in owned if pid in processes}


def process_alive(pid, expected):
    try:
        fields=Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()
        return fields[19]==expected['start_ticks'] and fields[0]!='Z'
    except (OSError, IndexError):
        return False


def stop_simulation(sim, world_path):
    owned=simulation_processes(sim.pid,world_path)
    try:
        os.killpg(sim.pid,signal.SIGINT)
    except ProcessLookupError:
        pass
    try:
        sim.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(sim.pid,signal.SIGKILL)
        sim.wait(timeout=10)
    # The ruby gz wrapper may exit before its server, which then belongs to
    # PID 1. Match the captured start time before signalling each owned PID.
    forced=[]
    for sig, timeout in [(signal.SIGTERM,3.),(signal.SIGKILL,2.)]:
        for pid, expected in owned.items():
            if pid!=os.getpid() and process_alive(pid,expected):
                try:
                    os.kill(pid,sig)
                    if sig==signal.SIGKILL:
                        forced.append(pid)
                except ProcessLookupError:
                    pass
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline and any(process_alive(pid,value) for pid,value in owned.items()):
            time.sleep(.05)
    remaining=[pid for pid,value in owned.items() if process_alive(pid,value)]
    return {'tracked_pids':sorted(owned),'sigkill_pids':forced,'remaining_pids':remaining}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('world', choices=WORLDS)
    parser.add_argument('robot', choices=['jackal_j100', 'husky_a200', 'x500'])
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--output', default='/tmp/simulation')
    parser.add_argument('--timeout', type=float, default=600.0)
    parser.add_argument('--seed', default='20261006')
    parser.add_argument('--capture', action='store_true')
    parser.add_argument('--capture-rviz', action='store_true')
    parser.add_argument('--capture-panorama', action='store_true')
    parser.add_argument('--media-output', type=Path, default=Path('/media'))
    args = parser.parse_args()

    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    if (output / f'{args.run_id}.json').exists():
        parser.error('Run record already exists; choose a new run id or output directory')
    runtime = output / args.run_id
    attempt = 1
    while runtime.exists():
        attempt += 1
        runtime = output / f'{args.run_id}_attempt{attempt}'
    runtime.mkdir(parents=True, exist_ok=False)
    args.capture_marker = str(runtime/'capture-ready.json') if args.capture else ''
    record = {
        'run_id': args.run_id,
        'world': args.world,
        'robot': args.robot,
        'seed': args.seed,
        'localization': 'simulation_sensors',
        'started_at': time.strftime('%Y-%m-%dT%H:%M:%S'),
        'result': 'not-run',
        'reason': None,
        'observations': {},
        'thresholds': {'goal_xy_m': 0.3, 'roll_pitch_limit_rad': 0.7853981633974483,
                       'stall_window_wall_s': 10.0, 'stall_distance_m': 0.05,
                       'stall_rotation_rad': 0.1},
        'task': 'site_ground_cases' if args.robot != 'x500' else 'site_offboard_inspection',
        'implementation_sha256': implementation_digest(),
        'image_id': os.environ.get('SIM_IMAGE_ID','unreported'),
        'ros_domain_id': os.environ.get('ROS_DOMAIN_ID','0'),
        'gz_partition': f'matrix-{args.run_id}',
        'camera_enabled': args.capture,
        'runtime_resources': runtime_resources(),
    }
    if args.robot == 'x500':
        record['localization'] = 'px4_simulated_sensors'
        from greenhouse_sim_air.air_mission import (WAYPOINT_RADIUS, ALTITUDE_TOLERANCE,
            RETURN_RADIUS, RETURN_SPEED, RETURN_SETTLE_SIM)
        record['thresholds']={'waypoint_radius_m':WAYPOINT_RADIUS,
            'altitude_tolerance_m':ALTITUDE_TOLERANCE,'hover_speed_mps':.5,
            'return_radius_m':RETURN_RADIUS,'return_speed_mps':RETURN_SPEED,
            'return_stable_sim_s':RETURN_SETTLE_SIM,
            'hover_stable_sim_s':3.0,'roll_pitch_limit_rad':math.pi/4,
            'telemetry_timeout_wall_s':2.0}

    log_path = runtime / 'simulation.log'
    record['simulation_log'] = str(log_path)
    sim_log = log_path.open('w')
    environment = dict(os.environ, GZ_IP='127.0.0.1', GZ_PARTITION=f'matrix-{args.run_id}')
    os.environ['GZ_PARTITION'] = environment['GZ_PARTITION']
    os.environ['GZ_IP'] = environment['GZ_IP']
    sim = subprocess.Popen(
        ['ros2', 'launch', 'greenhouse_nav2_bringup', 'simulation_matrix.launch.py',
         f'world:={args.world}', f'robot:={args.robot}', 'headless:=true',
         f'seed:={args.seed}', f'output:={runtime}', f'camera:={str(args.capture).lower()}'],
        stdout=sim_log, stderr=subprocess.STDOUT, env=environment, start_new_session=True)

    capture = None
    rviz_capture = None
    rviz_log = None
    capture_log = None
    if args.capture:
        args.media_output.mkdir(parents=True,exist_ok=True)
        capture_log=(runtime/'capture.log').open('w')
        capture=subprocess.Popen([sys.executable,str(Path(__file__).with_name('capture_simulation_image.py')),
            '--run-id',args.run_id,'--world',args.world,'--robot',args.robot,
            '--camera-config',str(runtime/'camera.json'),'--wait-marker',args.capture_marker,
            '--timeout',str(args.timeout),'--output',str(args.media_output)],
            stdout=capture_log,stderr=subprocess.STDOUT,env=environment)
    if args.capture_rviz:
        rviz_log=(runtime/'rviz-capture.log').open('w')
        rviz_capture=subprocess.Popen([sys.executable,str(Path(__file__).with_name('capture_simulation_rviz.py')),
            '--run-id',args.run_id,'--world',args.world,'--robot',args.robot,
            '--timeout',str(args.timeout),'--output',str(args.media_output)],
            stdout=rviz_log,stderr=subprocess.STDOUT,env=environment)
    try:
        if args.robot == 'x500':
            record.update(run_air(args, runtime))
        else:
            record.update(run_ground(args, runtime))
    except subprocess.TimeoutExpired:
        record['result'] = 'blocked'
        record['reason'] = f'timeout after {args.timeout}s'
    except Exception as exc:
        record['result'] = 'blocked'
        record['reason'] = f'{type(exc).__name__}: {exc}'
    finally:
        if capture is not None:
            try:
                capture.wait(timeout=30)
            except subprocess.TimeoutExpired:
                capture.terminate(); capture.wait(timeout=10)
            capture_log.close()
            metadata=args.media_output/f'{args.run_id}_task.json'
            record['capture_result']='saved' if capture.returncode==0 and metadata.is_file() else 'fail'
            if metadata.is_file():
                value=json.loads(metadata.read_text()); value['status']=record['result']
                value['result_record']=f'{args.run_id}.json'
                metadata.write_text(json.dumps(value,indent=2))
                record['image_metadata']=str(metadata)
        if rviz_capture is not None:
            try: rviz_capture.wait(timeout=20)
            except subprocess.TimeoutExpired:
                rviz_capture.terminate(); rviz_capture.wait(timeout=30)
            rviz_log.close()
            record['rviz_capture_result']='saved' if rviz_capture.returncode==0 else 'fail'
            metadata=args.media_output/f'{args.run_id}_rviz.json'
            if metadata.is_file():
                value=json.loads(metadata.read_text())
                value['status_at_capture']=value['status']
                value['status']=record['result']; value['result_record']=f'{args.run_id}.json'
                metadata.write_text(json.dumps(value,indent=2))
        if args.capture_panorama:
            try: record['panorama_capture_result']=capture_panorama(args,runtime,record['result'])
            except Exception as exc: record['panorama_capture_result']=f'fail: {type(exc).__name__}: {exc}'
        record['process_cleanup']=stop_simulation(sim,runtime/f'{args.world}-runtime.sdf')
        if record['process_cleanup']['remaining_pids']:
            record['result']='blocked'
            record['reason']='Simulation descendant cleanup incomplete; independent reset unavailable'
        sim_log.close()
    record['runtime_directory'] = str(runtime)
    record['ulog_files'] = [str(path) for path in runtime.rglob('*.ulg')]

    record['finished_at'] = time.strftime('%Y-%m-%dT%H:%M:%S')
    result_path = output / f'{args.run_id}.json'
    temporary=runtime/'result.json.tmp'
    temporary.write_text(json.dumps(record, indent=2))
    temporary.replace(result_path)
    print(json.dumps({'run_id': args.run_id, 'result': record['result'],
                      'reason': record['reason'], 'record': str(result_path)}))
    return 0 if record['result'] == 'pass' else 1


def capture_panorama(args,runtime,status):
    import runpy
    from ament_index_python.packages import get_package_share_directory
    share=Path(get_package_share_directory('greenhouse_nav2_bringup'))
    entry=runpy.run_path(str(share/'launch/simulation_matrix.launch.py'))
    x,y,z,roll,pitch,yaw=map(float,entry['CAMERA_POSES'][args.world])
    cr,sr,cp,sp,cy,sy=math.cos(roll/2),math.sin(roll/2),math.cos(pitch/2),math.sin(pitch/2),math.cos(yaw/2),math.sin(yaw/2)
    qx,qy,qz,qw=sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy
    request=(f'name: "sim_camera", position: {{x: {x}, y: {y}, z: {z}}}, '
             f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}')
    result=run_command(['gz','service','-s',f'/world/{entry["WORLDS"][args.world]["name"]}/set_pose',
        '--reqtype','gz.msgs.Pose','--reptype','gz.msgs.Boolean','--timeout','10000','--req',request],timeout=20)
    if result.returncode or 'data: true' not in result.stdout: raise RuntimeError(f'Camera move failed: {result.stdout}{result.stderr}')
    # Only the static camera pose changes. Robot and scene poses are untouched.
    configuration=runtime/'camera-panorama.json'
    configuration.write_text(json.dumps({'world':args.world,'robot':args.robot,'view':'panorama',
        'pose':[x,y,z,roll,pitch,yaw],'resolution':'1920x1080','topic':'/sim_camera/image'}))
    time.sleep(1)
    result=run_command([sys.executable,str(Path(__file__).with_name('capture_simulation_image.py')),
        '--run-id',args.run_id,'--world',args.world,'--robot',args.robot,'--kind','panorama',
        '--camera-config',str(configuration),'--status',status,'--output',str(args.media_output)],timeout=80)
    (runtime/'panorama-capture.log').write_text(result.stdout+result.stderr)
    return 'saved' if result.returncode==0 else 'fail'


def wait_for_px4_position(timeout: float) -> bool:
    """Wait for live telemetry; a discovered topic can outlive the old world."""
    import rclpy
    from px4_msgs.msg import VehicleLocalPosition
    from greenhouse_sim_air.air_mission import output_topic, px4_qos
    received = False
    def position(msg):
        nonlocal received
        received = msg.timestamp > 0 and all(math.isfinite(v) for v in (msg.x, msg.y, msg.z))
    rclpy.init()
    node = rclpy.create_node('simulation_px4_startup')
    node.create_subscription(VehicleLocalPosition,
        output_topic('vehicle_local_position', VehicleLocalPosition), position, px4_qos())
    try:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if received:
                return True
        return False
    finally:
        node.destroy_node()
        rclpy.shutdown()


def run_ground(args, output: Path):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from nav2_msgs.action import FollowWaypoints
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import Bool, String
    from ros_gz_interfaces.msg import Contacts
    from ament_index_python.packages import get_package_share_directory
    import yaml

    configuration = yaml.safe_load((Path(get_package_share_directory('greenhouse_nav2_bringup')) /
        'config/simulation/tasks/ground_sites.yaml').read_text())[args.world]

    rclpy.init()
    node = rclpy.create_node('simulation_ground_run')
    trace = (output / f'{args.run_id}_trace.jsonl').open('w')
    observations = {'path_length_m': 0.0, 'max_abs_roll_rad': 0.0,
                    'max_abs_pitch_rad': 0.0, 'odometry_samples': 0,
                    'unexpected_collisions': None,
                    'collision_measurement': 'contact-sensors', 'contact_messages': 0,
                    'normal_wheel_contacts': 0, 'collision_pairs': [],
                    'stall_events': 0, 'stalled': False, 'trace': trace.name}
    state = {'ready': False, 'ready_receipt': -math.inf, 'odom_receipt': -math.inf,
             'pose': None, 'clock': None, 'mission_status': None,
             'command_started': None, 'command_receipt': -math.inf,
             'contact_receipt': -math.inf, 'case': None}
    state.update(first_clock=None,first_clock_wall=None,clock_receipt=None,normal_case_passed=False)
    motion_window, collision_receipts = deque(), {}
    ground_prefix = {'maize_field': 'heightmap::', 'orchard': 'orchard::',
                     'pipeline': 'pipeline::'}[args.world]
    def contacts(msg):
        observations['contact_messages'] += 1
        state['contact_receipt'] = time.monotonic()
        if observations['unexpected_collisions'] is None:
            observations['unexpected_collisions'] = 0
        for contact in msg.contacts:
            names = (contact.collision1.name, contact.collision2.name)
            wheel_ground = any(
                names[index].startswith('robot::') and 'wheel_link_collision' in names[index]
                and names[1-index].startswith(ground_prefix)
                and any(abs(normal.z) >= math.sqrt(.5) for normal in contact.normals)
                for index in (0, 1))
            if wheel_ground:
                observations['normal_wheel_contacts'] += 1
                continue
            # Internal robot contacts are not collisions with the environment.
            if all(name.startswith('robot::') for name in names):
                continue
            pair = tuple(sorted(names))
            now = time.monotonic()
            if now-collision_receipts.get(pair, -math.inf) > .5:
                observations['unexpected_collisions'] += 1
                observations['collision_pairs'].append(list(pair))
                trace.write(json.dumps({'event': 'unexpected-contact', 'pair': pair,
                                        'sim_time_s': state['clock']})+'\n')
            collision_receipts[pair] = now
    def command(msg):
        now = time.monotonic()
        state['command_receipt'] = now
        active = abs(msg.twist.linear.x) > .05 or abs(msg.twist.angular.z) > .1
        if active and state['command_started'] is None:
            state['command_started'] = now
        elif not active:
            state['command_started'] = None
    def health(msg):
        state.update(ready=msg.data, ready_receipt=time.monotonic())
    def clock(msg):
        state['clock'] = msg.clock.sec + msg.clock.nanosec * 1e-9
        state['clock_receipt']=time.monotonic()
        observations['clock_publishers']=node.count_publishers('/clock')
        if state['first_clock'] is None:
            state['first_clock'],state['first_clock_wall']=state['clock'],state['clock_receipt']
    def mission_status(msg):
        state['mission_status'] = msg.data
    def odometry(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        xyz = (p.x, p.y, p.z)
        if not all(math.isfinite(value) for value in (*xyz, q.x, q.y, q.z, q.w)):
            return
        if state['pose'] is not None:
            observations['path_length_m'] += math.dist(xyz, state['pose'])
        state['pose'] = xyz
        state['odom_receipt'] = time.monotonic()
        roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
        pitch = math.asin(max(-1., min(1., 2*(q.w*q.y-q.z*q.x))))
        observations['max_abs_roll_rad'] = max(observations['max_abs_roll_rad'], abs(roll))
        observations['max_abs_pitch_rad'] = max(observations['max_abs_pitch_rad'], abs(pitch))
        observations['odometry_samples'] += 1
        now = time.monotonic()
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        motion_window.append((now, xyz, yaw))
        while len(motion_window) > 1 and now-motion_window[1][0] >= 10.:
            motion_window.popleft()
        if now-state['command_receipt'] > .5:
            state['command_started'] = None
        stalled = (state['command_started'] is not None
                   and now-state['command_started'] >= 10.
                   and now-motion_window[0][0] >= 10.
                   and max(math.dist(item[1], motion_window[0][1]) for item in motion_window) < .05
                   and max(abs(math.remainder(item[2]-motion_window[0][2],2*math.pi)) for item in motion_window)<.1)
        if stalled and not observations['stalled']:
            observations['stall_events'] += 1
        observations['stalled'] = observations['stalled'] or stalled
        trace.write(json.dumps({'sim_time_s': msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                                'case': state['case'],
                                'position': xyz, 'quaternion': [q.x,q.y,q.z,q.w],
                                'roll': roll, 'pitch': pitch})+'\n')
    node.create_subscription(Bool, '/navigation/data_ready', health, 10)
    node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data)
    node.create_subscription(String, '/navigation/mission_status', mission_status, 10)
    node.create_subscription(Odometry, '/localization/global_odometry', odometry, qos_profile_sensor_data)
    node.create_subscription(Contacts, '/simulation/contacts', contacts, 10)
    node.create_subscription(TwistStamped, '/cmd_vel', command, 10)
    client = ActionClient(node, FollowWaypoints, '/navigation/follow_waypoints')
    def ready():
        now = time.monotonic()
        return (state['ready'] and now-state['ready_receipt'] < .3
                and now-state['odom_receipt'] < .5 and client.server_is_ready()
                and now-state['contact_receipt'] < .7
                and node.count_publishers('/clock')==1)
    def wait(predicate, timeout):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if predicate():
                return True
        return False
    def execute_case(case, remaining):
        state['case'] = case['name']
        started,sim_started=time.monotonic(),state['clock']
        path_started=observations['path_length_m']
        contacts_started=observations['unexpected_collisions']
        def metrics():
            elapsed=time.monotonic()-started
            result={'mission_elapsed_s':elapsed,'final_position':state['pose'],
                'mission_status':state['mission_status'],
                'path_length_m':observations['path_length_m']-path_started,
                'unexpected_collisions':None if contacts_started is None or observations['unexpected_collisions'] is None
                    else observations['unexpected_collisions']-contacts_started,
                'max_abs_roll_rad':observations['max_abs_roll_rad'],
                'max_abs_pitch_rad':observations['max_abs_pitch_rad'],
                'stalled':observations['stalled'],
                'tipped':max(observations['max_abs_roll_rad'],observations['max_abs_pitch_rad'])>math.pi/4}
            if state['clock'] is not None and sim_started is not None:
                result['sim_elapsed_s']=state['clock']-sim_started
                result['real_time_factor']=result['sim_elapsed_s']/elapsed
            if state['pose'] is not None:
                result['goal_error_m']=math.hypot(state['pose'][0]-case['waypoints'][-1][0],
                                                 state['pose'][1]-case['waypoints'][-1][1])
            return result
        def failed(result,reason):
            return {'result':result,'reason':reason,'observations':metrics()}
        if not wait(ready, min(30., remaining)):
            return failed('blocked','fresh navigation, odometry and contacts unavailable')
        goal = FollowWaypoints.Goal()
        for x, y in case['waypoints']:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.w = 1.0
            goal.poses.append(pose)
        sent = client.send_goal_async(goal)
        if not wait(sent.done, 30.):
            return failed('blocked','action acceptance timeout')
        handle = sent.result()
        if not handle.accepted:
            return failed('fail','navigation action rejected')
        marker=getattr(args,'capture_marker','')
        if marker and not Path(marker).exists():
            Path(marker).write_text(json.dumps({'phase':'ground-mission-active',
                'case':case['name'],'sim_time_s':state['clock'],'position':state['pose']}))
        result = handle.get_result_async()
        if not wait(result.done, remaining):
            cancelled = handle.cancel_goal_async()
            wait(cancelled.done, 10.)
            return failed('fail','navigation mission timeout')
        value = result.result()
        measured = dict(metrics(), action_status=value.status,
                            mission_error_code=value.result.error_code,
                            mission_error_msg=value.result.error_msg,
                            mission_status=state['mission_status'])
        reached = (value.status == GoalStatus.STATUS_SUCCEEDED and value.result.error_code == 0
                   and measured.get('goal_error_m', math.inf) <= .3)
        planner_refused = case['expectation'] == 'reach_or_refuse' and value.result.error_msg == 'FAR unreachable'
        refused = planner_refused and state['normal_case_passed']
        passed = ((reached or refused) and not measured['tipped'] and not measured['stalled']
                  and measured['unexpected_collisions'] == 0 and ready())
        measured['outcome'] = 'reached' if reached else 'refused' if refused else 'planner-refused-unqualified' if planner_refused else 'failed'
        return {'result': 'pass' if passed else 'fail',
                'reason': None if passed else value.result.error_msg or 'navigation mission failed',
                'observations': measured}
    try:
        started = time.monotonic()
        if not wait(ready, min(180., args.timeout)):
            observations['cases']=[{**case,'result':'not-run','reason':'navigation readiness unavailable'}
                for case in configuration['cases']]
            return {'result': 'blocked', 'reason': 'fresh navigation readiness and global odometry unavailable',
                    'observations': observations, 'task_configuration': configuration}
        deadline = time.monotonic()+args.timeout
        cases = []
        for case in configuration['cases']:
            remaining = deadline-time.monotonic()
            if observations['stalled'] or max(observations['max_abs_roll_rad'],observations['max_abs_pitch_rad'])>math.pi/4 or (observations['unexpected_collisions'] or 0)>0:
                outcome={'result':'not-run','reason':'run stopped after collision, stall or tip'}
            elif remaining <= 0:
                outcome = {'result': 'not-run', 'reason': 'run deadline exhausted'}
            else:
                outcome = execute_case(case, remaining)
            cases.append({**case, **outcome})
            if len(cases)==1: state['normal_case_passed']=outcome['result']=='pass'
        observations['cases'] = cases
        observations['mission_elapsed_s'] = time.monotonic()-started
        if state['first_clock'] is not None:
            observations['sim_elapsed_s']=state['clock']-state['first_clock']
            clock_wall=state['clock_receipt']-state['first_clock_wall']
            observations['real_time_factor']=observations['sim_elapsed_s']/clock_wall if clock_wall>0 else None
        observations['tipped'] = max(observations['max_abs_roll_rad'], observations['max_abs_pitch_rad']) > math.pi/4
        passed = all(case['result'] == 'pass' for case in cases)
        return {'result': 'pass' if passed else 'fail',
                'reason': None if passed else '; '.join(f"{case['name']}: {case['reason']}"
                    for case in cases if case['result'] != 'pass'),
                'observations': observations, 'task_configuration': configuration}
    finally:
        trace.close()
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()


def run_air(args, output: Path):
    observations = {}
    if not wait_for_px4_position(180):
        return {'result': 'blocked', 'reason': 'PX4 position telemetry never arrived within 180 s',
                'observations': observations}
    result_path = output / f'{args.run_id}_air.json'
    mission_log=output/f'{args.run_id}_air.log'
    command=['ros2', 'run', 'greenhouse_sim_air', 'air_mission', '--ros-args',
             '-p', f'site:={args.world}', '-p', f'output:={result_path}']
    marker=getattr(args,'capture_marker','')
    if marker:
        command.extend(['-p', f'capture_marker:={marker}'])
    with mission_log.open('w') as handle:
        mission=subprocess.Popen(
        command,
        stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            mission.wait(timeout=args.timeout)
        except subprocess.TimeoutExpired:
            os.killpg(mission.pid,signal.SIGINT)
            try: mission.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(mission.pid,signal.SIGKILL); mission.wait(timeout=10)
    observations['mission_log']=str(mission_log)
    observations['mission_output_tail'] = mission_log.read_text()[-2000:]
    air = {}
    if result_path.is_file():
        air = json.loads(result_path.read_text())
        observations['air_mission'] = air
    passed = mission.returncode == 0 and air.get('result') == 'pass'
    return {'result': 'pass' if passed else 'blocked' if air.get('result') == 'blocked' else 'fail',
            'reason': None if passed else air.get('reason', 'air mission failed'),
            'observations': observations}


if __name__ == '__main__':
    raise SystemExit(main())
