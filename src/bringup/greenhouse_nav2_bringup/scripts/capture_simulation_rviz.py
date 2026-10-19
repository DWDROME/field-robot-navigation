#!/usr/bin/env python3
"""Capture an actual RViz window on an independently verified Xvfb display."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import cv2
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Path as RosPath
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id',required=True)
    parser.add_argument('--world',required=True)
    parser.add_argument('--robot',required=True)
    parser.add_argument('--output',type=Path,default=Path('/media'))
    parser.add_argument('--timeout',type=float,default=240)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    config=Path(get_package_share_directory('greenhouse_nav2_bringup'))/'config/simulation/navigation.rviz'
    number=100+os.getpid()%1000
    while Path(f'/tmp/.X11-unix/X{number}').exists(): number+=1
    display=f':{number}'
    environment=dict(os.environ,DISPLAY=display,LIBGL_ALWAYS_SOFTWARE='1',GALLIUM_DRIVER='llvmpipe',QT_X11_NO_MITSHM='1')
    log=(args.output/f'{args.run_id}_rviz.log').open('w')
    server=rviz=node=None
    try:
        server=subprocess.Popen(['Xvfb',display,'-screen','0','1920x1080x24','-nolisten','tcp'],
                                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+10
        while not Path(f'/tmp/.X11-unix/X{number}').exists() and time.monotonic()<deadline:
            if server.poll() is not None: raise RuntimeError('Xvfb exited')
            time.sleep(.1)
        rviz=subprocess.Popen(['rviz2','-d',str(config),'--ros-args','-p','use_sim_time:=true'],
            env=environment,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        rclpy.init(); node=rclpy.create_node('simulation_rviz_capture')
        def terminate(signum,frame): raise KeyboardInterrupt()
        signal.signal(signal.SIGTERM,terminate)
        state={'cloud_points':0,'terrain_points':0,'path_poses':0,'sim_time_s':None,'mission_status':None}
        # Preserve one actual graph path for the display only. Control still
        # consumes /planning/global_path and obeys its cancellation/expiry.
        snapshot=node.create_publisher(RosPath,'/simulation/rviz_path_snapshot',
            QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        def observe_path(message):
            if len(message.poses)<2 or state['path_poses']:
                return
            snapshot.publish(message)
            state.update(path_poses=len(message.poses),
                path_snapshot_stamp_s=message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
                path_snapshot_frame=message.header.frame_id,
                path_snapshot_xyz=[[pose.pose.position.x,pose.pose.position.y,pose.pose.position.z]
                                   for pose in message.poses])
        node.create_subscription(PointCloud2,'/localization/registered_cloud',
            lambda m:state.update(cloud_points=m.width*m.height),qos_profile_sensor_data)
        node.create_subscription(PointCloud2,'/perception/terrain_map',
            lambda m:state.update(terrain_points=m.width*m.height),qos_profile_sensor_data)
        node.create_subscription(RosPath,'/planning/global_path',observe_path,10)
        node.create_subscription(Clock,'/clock',lambda m:state.update(sim_time_s=m.clock.sec+m.clock.nanosec*1e-9),qos_profile_sensor_data)
        node.create_subscription(String,'/navigation/mission_status',lambda m:state.update(mission_status=m.data),10)
        started=time.monotonic(); deadline=started+args.timeout
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if rviz.poll() is not None: raise RuntimeError('RViz exited; inspect capture log')
            if (time.monotonic()-started>=12 and state['cloud_points']>0
                    and state['terrain_points']>0 and state['path_poses']>=2 and state['sim_time_s'] is not None): break
        else: raise RuntimeError('Actual registered cloud, terrain and FAR path were not observed')
        # RViz callbacks and its next render need time after our subscriber
        # receives the path; merely receiving a ROS message does not draw it.
        settled=time.monotonic()+2
        while time.monotonic()<settled:
            rclpy.spin_once(node,timeout_sec=.1)
        png=args.output/f'{args.run_id}_rviz.png'
        subprocess.run(['import','-display',display,'-window','root',str(png)],env=environment,check=True,timeout=20)
        image=cv2.imread(str(png))
        if image is None or image.shape[:2]!=(1080,1920) or float(image.var())<4:
            raise RuntimeError('RViz screenshot is empty or has the wrong resolution')
        jpeg=png.with_suffix('.jpg')
        if not cv2.imwrite(str(jpeg),image,[cv2.IMWRITE_JPEG_QUALITY,88]): raise OSError(jpeg)
        metadata={'run_id':args.run_id,'kind':'rviz','world':args.world,'robot':args.robot,
            'capture_source':'Actual RViz2 window on Xvfb, software GLX',
            'path_display':'First actual /planning/global_path, retained unchanged on a display-only snapshot topic; may be cancelled at capture time',
            'display':display,'resolution':'1920x1080','view_configuration':str(config),
            'camera_pose':{'type':'Orbit','target_frame':'base_link','distance':13,'pitch':.85,'yaw':.85,
                           'focal_point':[2,0,0]},'observed_topics':state,'sim_time_s':state['sim_time_s'],
            'status':state['mission_status'],'image':png.name,'compressed_image':jpeg.name,
            'visual_review':'pending; topic availability alone does not prove rendered model/path visibility'}
        png.with_suffix('.json').write_text(json.dumps(metadata,indent=2))
        print(json.dumps({'result':'saved','image':str(png)}))
        return 0
    finally:
        if node is not None: node.destroy_node(); rclpy.try_shutdown()
        for process in [rviz,server]:
            if process is not None and process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try: process.wait(timeout=10)
                except subprocess.TimeoutExpired: os.killpg(process.pid,signal.SIGKILL); process.wait(timeout=10)
        log.close()


if __name__=='__main__': raise SystemExit(main())
