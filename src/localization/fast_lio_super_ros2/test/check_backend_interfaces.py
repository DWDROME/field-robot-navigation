#!/usr/bin/env python3
"""Actual ROS 2 synchronization, iSAM2 correction and output ownership check."""
import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path as Trajectory
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Header
from diagnostic_msgs.msg import DiagnosticArray
from tf2_msgs.msg import TFMessage


class Fixture(Node):
    def __init__(self):
        super().__init__('slam_backend_interface_check')
        self.odom=self.create_publisher(Odometry,'/check/odometry',qos_profile_sensor_data)
        self.cloud=self.create_publisher(PointCloud2,'/check/cloud',qos_profile_sensor_data)
        self.state={}; self.path=None; self.map=None; self.global_odom=None; self.transforms=[]
        self.create_subscription(DiagnosticArray,'/check/diagnostics',self.diagnostic,10)
        self.create_subscription(Trajectory,'/check/path',lambda m:setattr(self,'path',m),10)
        self.create_subscription(PointCloud2,'/check/map',lambda m:setattr(self,'map',m),10)
        self.create_subscription(Odometry,'/check/global',lambda m:setattr(self,'global_odom',m),10)
        self.create_subscription(TFMessage,'/tf',lambda m:self.transforms.extend(m.transforms),10)
        self.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1)
                     for i,n in enumerate(('x','y','z','intensity'))]
        self.points=[]
        for i in range(900):
            a=i*2.399963229728653; radius=2+(i%41)*.19
            self.points.append((radius*math.cos(a),radius*math.sin(a),math.sin(i*.73)*1.3+(i%17)*.12,float(i%255)))

    def diagnostic(self,m):
        for status in m.status:
            self.state={v.key:int(v.value) for v in status.values}; self.state['status']=status.message

    def wait(self,predicate,timeout=20,pump=None):
        end=time.monotonic()+timeout
        next_publish=0.
        while time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=.02)
            if predicate(): return
            if pump is not None and time.monotonic()>=next_publish:
                pump(); next_publish=time.monotonic()+.1
        raise AssertionError(str(self.state))

    def send(self,x):
        header=Header(); header.frame_id='check_odom'; header.stamp=self.get_clock().now().to_msg()
        odom=Odometry(); odom.header=header; odom.child_frame_id='check_base'
        odom.pose.pose.position.x=x; odom.pose.pose.orientation.w=1.
        for i in range(6): odom.pose.covariance[i*7]=.01
        cloud=create_cloud(header,self.fields,[(px+x,py,pz,intensity) for px,py,pz,intensity in self.points])
        self.pending=(odom,cloud)
        self.publish_pending()
        return odom

    def publish_pending(self):
        # The backend uses best-effort sensor QoS. Send a bounded 10 Hz fixture
        # stream until its diagnostic acknowledges this exact synchronized frame.
        # Retain the stamp so duplicate packets cannot create extra keyframes.
        self.odom.publish(self.pending[0]); self.cloud.publish(self.pending[1])

    def run(self):
        self.wait(lambda:self.odom.get_subscription_count()==1 and self.cloud.get_subscription_count()==1)
        self.wait(lambda:bool(self.state))
        for i,x in enumerate((0.,1.1,.1)):
            raw=self.send(x)
            self.wait(lambda:self.state.get('keyframes')==i+1,pump=self.publish_pending)
            assert raw.pose.pose.position.x==x
            if i<2:
                until=time.monotonic()+.25
                while time.monotonic()<until: rclpy.spin_once(self,timeout_sec=.02)
        self.wait(lambda:self.state.get('accepted_loops')==1 and self.path is not None and len(self.path.poses)==3)
        self.wait(lambda:self.map is not None)
        assert self.path.header.frame_id==self.map.header.frame_id=='check_map'
        assert self.map.width*self.map.height<=6000
        expected=self.path.poses[-1].pose.position.x
        assert abs(expected)<.02,expected
        raw=self.send(.1)
        self.wait(lambda:self.global_odom is not None and self.global_odom.header.stamp==raw.header.stamp,
                  pump=self.publish_pending)
        assert self.global_odom.header.frame_id=='check_map' and self.global_odom.child_frame_id=='check_base'
        assert abs(self.global_odom.pose.pose.position.x-expected)<1e-5
        self.wait(lambda:any(t.header.stamp==raw.header.stamp for t in self.transforms))
        latest=next(t for t in reversed(self.transforms) if t.header.stamp==raw.header.stamp)
        assert latest.header.frame_id=='check_map' and latest.child_frame_id=='check_odom'
        assert abs(latest.transform.translation.x-(expected-.1))<1e-5
        print('PASS exact synchronized body keyframes, real loop/iSAM2, corrected path/map/current pose and sole map->odom TF',flush=True)


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--log-dir',required=True); args=parser.parse_args()
    logs=Path(args.log_dir); logs.mkdir(parents=True,exist_ok=True)
    cmd=['ros2','run','fast_lio_super_ros2','fast_lio_super_backend','--ros-args']
    params={'map_frame':'check_map','odom_frame':'check_odom','body_frame':'check_base','voxel':.05,
        'min_loop_keyframes':2,'min_loop_seconds':.2,'candidate_period':.1,'submap_frames':0,
        'max_keyframes':8,'max_frame_points':2000,'max_cloud_points':16000,'output_period':.1,
        'odom_translation_sigma':.1,'loop_translation_sigma':.01}
    for name,value in params.items(): cmd+=['-p',f'{name}:={value}']
    for source,target in [('odometry','/check/odometry'),('registered_cloud','/check/cloud'),
        ('global_odometry','/check/global'),('corrected_path','/check/path'),('corrected_map','/check/map'),('diagnostics','/check/diagnostics')]:
        cmd+=['-r',f'{source}:={target}']
    with (logs/'backend.log').open('w') as stream:
        child=subprocess.Popen(cmd,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            rclpy.init(); node=Fixture()
            try: node.run()
            finally: node.destroy_node(); rclpy.shutdown()
        finally:
            if child.poll() is None: os.killpg(child.pid,signal.SIGINT)
            try: child.wait(timeout=3)
            except subprocess.TimeoutExpired: os.killpg(child.pid,signal.SIGTERM); child.wait(timeout=3)
        assert child.returncode==0,child.returncode


if __name__=='__main__': main()
