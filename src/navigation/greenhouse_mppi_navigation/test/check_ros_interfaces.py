#!/usr/bin/env python3
"""Targeted, hardware-free checks with real Jazzy controllers, terrain and mission nodes.

The route publisher is an explicit input fixture, not a replacement FAR planner.
FAR's graph-path producer is checked separately at its pinned source/build boundary.
Run in a private ROS_DOMAIN_ID after sourcing the built packages. No sensor replay
or performance result is produced. Only processes started here are stopped.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped, TwistStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path as Route
from nav2_msgs.action import FollowWaypoints
from lifecycle_msgs.srv import GetState
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Bool, String
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory


class Check(Node):
    def __init__(self,controller='mppi'):
        super().__init__('closed_loop_interface_check')
        self.position=(0.,0.); self.obstacle=True; self.cloud_on=True; self.tf_on=True
        self.route_mode='ready'; self.target=None; self.goals=[]; self.status={}
        self.controller=controller; self.local_waypoint=None
        self.waypoint_mode='fresh'; self.last_waypoint=None; self.ready_since=None
        self.ready=False; self.grid=None; self.costmap=None; self.commands=[]; self.nav_commands=[]; self.feedback=[]
        self.odom=self.create_publisher(Odometry,'/localization/odometry',10)
        self.global_odom=self.create_publisher(Odometry,'/localization/global_odometry',10)
        self.cloud=self.create_publisher(PointCloud2,'/perception/terrain_map',10)
        self.route=self.create_publisher(Route,'/planning/global_path',10)
        self.waypoint=self.create_publisher(PointStamped,'/planning/way_point',10)
        self.route_status=self.create_publisher(String,'/planning/route_status',10)
        self.tf=TransformBroadcaster(self)
        self.create_subscription(PointStamped,'/planning/global_goal',self.goal,10)
        self.create_subscription(PointStamped,'/planning/cmu_waypoint',lambda m:setattr(self,'local_waypoint',m),10)
        self.create_subscription(String,'/navigation/mission_status',lambda m:setattr(self,'status',json.loads(m.data)),10)
        self.create_subscription(Bool,'/navigation/data_ready',self.health,10)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid,'/perception/terrain_grid',lambda m:setattr(self,'grid',m),qos)
        self.create_subscription(OccupancyGrid,'/local_costmap/costmap',lambda m:setattr(self,'costmap',m),qos)
        self.create_subscription(TwistStamped,'/cmd_vel',self.command,10)
        self.create_subscription(TwistStamped,'/cmd_vel/nav',lambda m:self.nav_commands.append(
            (time.monotonic(),(m.twist.linear.x,m.twist.angular.z))),10)
        self.client=ActionClient(self,FollowWaypoints,'/navigation/follow_waypoints')
        self.lifecycle=self.create_client(GetState,'/controller_server/get_state')
        self.last_cloud=0.; self.last_route=0.
        self.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1)
                     for i,n in enumerate(('x','y','z','intensity'))]
        # map_from_odom is a 90-degree rotation with translation (10,5,0).
        self.ground=[(10-y,5+x,0.,0.) for x in [i*.2+.1 for i in range(-32,32)]
                     for y in [i*.2+.1 for i in range(-32,32)]]
        self.create_timer(.05,self.publish_inputs)

    @staticmethod
    def ns(stamp): return stamp.sec*1000000000+stamp.nanosec

    def health(self,msg):
        self.ready=msg.data
        self.ready_since=(self.ready_since or time.monotonic()) if msg.data else None

    def command(self,m):
        assert m.header.frame_id=='base_link' and self.ns(m.header.stamp)>0
        values=(m.twist.linear.x,m.twist.angular.z)
        assert all(math.isfinite(v) for v in values)
        self.commands.append((time.monotonic(),values)); self.commands=self.commands[-1000:]

    def goal(self,m):
        if not self.target or self.ns(m.header.stamp)!=self.ns(self.target.header.stamp):
            self.target=m; self.goals.append(m)

    def publish_inputs(self):
        stamp=self.get_clock().now().to_msg(); wall=time.monotonic(); x,y=self.position
        o=Odometry(); o.header.stamp=stamp; o.header.frame_id='odom'; o.child_frame_id='base_link'
        o.pose.pose.position.x=x; o.pose.pose.position.y=y; o.pose.pose.orientation.w=1.
        self.odom.publish(o)
        global_o=Odometry(); global_o.header.stamp=stamp; global_o.header.frame_id='map'; global_o.child_frame_id='base_link'
        global_o.pose.pose.position.x=10-y; global_o.pose.pose.position.y=5+x
        global_o.pose.pose.orientation.z=math.sqrt(.5); global_o.pose.pose.orientation.w=math.sqrt(.5)
        self.global_odom.publish(global_o)
        if self.tf_on:
            a=TransformStamped(); a.header.stamp=stamp; a.header.frame_id='map'; a.child_frame_id='odom'
            a.transform.translation.x=10.; a.transform.translation.y=5.
            a.transform.rotation.z=math.sqrt(.5); a.transform.rotation.w=math.sqrt(.5)
            b=TransformStamped(); b.header.stamp=stamp; b.header.frame_id='odom'; b.child_frame_id='base_link'
            b.transform.translation.x=x; b.transform.translation.y=y; b.transform.rotation.w=1.
            self.tf.sendTransform([a,b])
        if self.cloud_on and wall-self.last_cloud>.15:
            header=Header(); header.stamp=stamp; header.frame_id='map'
            points=self.ground+([(7.9,8.1,0.,.5)] if self.obstacle else [])
            self.cloud.publish(create_cloud(header,self.fields,points)); self.last_cloud=wall
        if self.target and wall-self.last_route>.15 and self.route_mode!='mute':
            path=Route(); path.header.stamp=stamp; path.header.frame_id='map'
            if self.route_mode=='ready':
                sx,sy=10-y,5+x
                for i in range(11):
                    p=PoseStamped(); p.header=path.header
                    p.pose.position.x=sx+(self.target.point.x-sx)*i/10
                    p.pose.position.y=sy+(self.target.point.y-sy)*i/10
                    yaw=math.atan2(self.target.point.y-sy,self.target.point.x-sx)
                    p.pose.orientation.z=math.sin(yaw/2); p.pose.orientation.w=math.cos(yaw/2)
                    path.poses.append(p)
            status=String(data=json.dumps({'goal_stamp_ns':self.ns(self.target.header.stamp),
                'plan_stamp_ns':self.ns(stamp),'status':self.route_mode}))
            # Exercise both cross-topic arrival orders.
            self.route_status.publish(status); self.route.publish(path); self.last_route=wall
            if self.route_mode=='ready':
                waypoint=PointStamped(); waypoint.header=path.header; waypoint.point=self.target.point
                if self.waypoint_mode=='fresh':
                    self.last_waypoint=waypoint; self.waypoint.publish(waypoint)
                elif self.waypoint_mode=='stale' and self.last_waypoint is not None:
                    self.waypoint.publish(self.last_waypoint)

    def wait(self,predicate,timeout=8,label='condition'):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=.03)
            if predicate(): return
        raise AssertionError('timeout: '+label+'; mission='+str(self.status))

    def future(self,future,timeout=8):
        self.wait(future.done,timeout,'future'); return future.result()

    def send(self,points):
        self.wait(lambda:self.client.server_is_ready(),label='FollowWaypoints server')
        request=FollowWaypoints.Goal()
        for x,y in points:
            p=PoseStamped(); p.header.frame_id='map'; p.pose.position.x=x; p.pose.position.y=y; p.pose.orientation.w=1.
            request.poses.append(p)
        handle=self.future(self.client.send_goal_async(request,feedback_callback=lambda f:self.feedback.append(f.feedback.current_waypoint)))
        assert handle.accepted
        return handle

    def stopped(self):
        end=time.monotonic()+.3
        while time.monotonic()<end: rclpy.spin_once(self,timeout_sec=.03)
        requested=[v for t,v in self.nav_commands if t>time.monotonic()-.15]
        assert requested and all(abs(v)+abs(w)<1e-8 for v,w in requested),requested
        # The existing gate ramps valid zero commands under its acceleration limits.
        # Fixture bound: max(0.6/0.5, 1.0/1.0) seconds plus scheduling margin.
        end=time.monotonic()+1.6
        while time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=.03)
            recent=[v for t,v in self.commands if t>time.monotonic()-.15]
            if len(recent)>=2 and all(abs(v)+abs(w)<1e-8 for v,w in recent): break
        recent=[v for t,v in self.commands if t>time.monotonic()-.15]
        assert recent and all(abs(v)+abs(w)<1e-8 for v,w in recent),recent

    @staticmethod
    def cell(grid,x,y):
        ix=int(math.floor((x-grid.info.origin.position.x)/grid.info.resolution))
        iy=int(math.floor((y-grid.info.origin.position.y)/grid.info.resolution))
        assert 0<=ix<grid.info.width and 0<=iy<grid.info.height
        return grid.data[iy*grid.info.width+ix]

    def run(self):
        self.wait(lambda:self.ready and self.grid is not None and (self.controller=='cmu' or self.costmap is not None),30,'controller/terrain/TF ready')
        if self.controller=='mppi':
            state=self.future(self.lifecycle.call_async(GetState.Request()))
            assert state.current_state.label=='active'
        assert self.cell(self.grid,3.1,2.1)==100 and self.cell(self.grid,-2.1,1.1)==0
        if self.controller=='mppi':
            self.wait(lambda:self.cell(self.costmap,3.1,2.1)>=99,label='terrain obstacle in Nav2 layer')
        self.obstacle=False
        self.wait(lambda:self.cell(self.grid,3.1,2.1)==0 and (self.controller=='cmu' or self.cell(self.costmap,3.1,2.1)==0),label='obstacle clearing')
        self.wait(lambda:self.ready_since is not None and time.monotonic()-self.ready_since>=.5,label='stable input readiness')
        print(f'PASS {self.controller} inputs, non-identity TF, terrain obstacle and clearing',flush=True)
        first=len(self.goals); start=time.monotonic()
        handle=self.send([(10.,6.),(9.,6.)]); result=handle.get_result_async()
        self.wait(lambda:any(t>start and abs(v)+abs(w)>.01 for t,(v,w) in self.commands),label='finite gated controller command')
        if self.controller=='cmu':
            assert self.local_waypoint.header.frame_id=='odom'
            assert abs(self.local_waypoint.point.x-1.)<1e-5 and abs(self.local_waypoint.point.y)<1e-5
        assert len(self.goals)==first+1 and self.feedback and self.feedback[-1]==0
        self.position=(1.,0.)
        self.wait(lambda:len(self.goals)==first+2,label='second waypoint after arrival')
        self.position=(1.,1.)
        assert self.future(result).status==GoalStatus.STATUS_SUCCEEDED
        self.wait(lambda:self.status.get('state')=='completed'); self.stopped()
        print(f'PASS actual {self.controller} control, ordered two-waypoint completion and terminal stop',flush=True)
        handle=self.send([(10.,7.)]); result=handle.get_result_async()
        self.wait(lambda:self.status.get('state')=='controlling')
        assert self.future(handle.cancel_goal_async()).goals_canceling
        assert self.future(result).status==GoalStatus.STATUS_CANCELED
        self.wait(lambda:self.status.get('state')=='cancelled'); self.stopped()
        print('PASS action cancellation and zero gate output despite continuing route inputs',flush=True)
        if self.controller=='cmu':
            self.waypoint_mode='stale'
            handle=self.send([(10.,7.)]); result=handle.get_result_async()
            self.wait(lambda:self.status.get('state')=='controlling')
            self.stopped()
            assert self.future(handle.cancel_goal_async()).goals_canceling
            assert self.future(result).status==GoalStatus.STATUS_CANCELED
            self.waypoint_mode='fresh'; start=time.monotonic()
            handle=self.send([(10.,7.)]); result=handle.get_result_async()
            self.wait(lambda:any(t>start and abs(v)+abs(w)>.01 for t,(v,w) in self.commands),label='fresh CMU command after stale waypoint rejection')
            self.waypoint_mode='mute'
            assert self.future(result).status==GoalStatus.STATUS_ABORTED
            self.wait(lambda:self.status.get('reason')=='CMU waypoint expired'); self.stopped()
            self.waypoint_mode='fresh'
            print('PASS old CMU waypoint cannot enable a new task; waypoint expiry stops control',flush=True)
        self.route_mode='unreachable'
        handle=self.send([(10.,7.)])
        assert self.future(handle.get_result_async()).status==GoalStatus.STATUS_ABORTED
        self.wait(lambda:self.status.get('state')=='failed'); self.stopped()
        self.route_mode='mute'
        handle=self.send([(10.,7.)])
        self.stopped()
        assert self.future(handle.get_result_async(),8).status==GoalStatus.STATUS_ABORTED
        self.wait(lambda:self.status.get('state')=='timeout'); self.stopped()
        print('PASS unreachable and planning timeout fail closed',flush=True)
        self.route_mode='ready'; handle=self.send([(10.,7.)]); result=handle.get_result_async()
        self.wait(lambda:self.status.get('state')=='controlling')
        self.cloud_on=False
        assert self.future(result).status==GoalStatus.STATUS_ABORTED
        self.wait(lambda:not self.ready); self.stopped()
        self.cloud_on=True; self.wait(lambda:self.ready)
        handle=self.send([(10.,7.)]); result=handle.get_result_async()
        self.wait(lambda:self.status.get('state')=='controlling')
        self.tf_on=False
        assert self.future(result).status==GoalStatus.STATUS_ABORTED
        self.wait(lambda:not self.ready); self.stopped()
        print('PASS terrain expiry and dynamic TF expiry stop navigation',flush=True)
        self.tf_on=True; self.wait(lambda:self.ready)
        handle=self.send([(10.,7.)]); result=handle.get_result_async()
        self.wait(lambda:self.status.get('state')=='controlling')
        other='mppi' if self.controller=='cmu' else 'cmu'
        duplicate=self.create_publisher(TwistStamped,f'/navigation/{other}_cmd',1)
        try:
            assert self.future(result).status==GoalStatus.STATUS_ABORTED
            self.wait(lambda:not self.ready); self.stopped()
        finally: self.destroy_publisher(duplicate)
        print('PASS simultaneous CMU/MPPI command sources disable navigation',flush=True)


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--log-dir',required=True)
    parser.add_argument('--controller',choices=('cmu','mppi'),default='mppi')
    args=parser.parse_args(); logs=Path(args.log_dir); logs.mkdir(parents=True,exist_ok=True)
    processes=[]; files=[]
    def start(name,command):
        stream=(logs/(name+'.log')).open('w'); files.append(stream)
        processes.append(subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True))
    share=Path(get_package_share_directory('greenhouse_mppi_navigation'))
    try:
        if args.controller=='mppi':
            start('controller',['ros2','run','nav2_controller','controller_server','--ros-args','--params-file',str(share/'config/mppi.yaml'),'-r','cmd_vel:=/navigation/mppi_cmd'])
            start('lifecycle',['ros2','run','nav2_lifecycle_manager','lifecycle_manager','--ros-args','-r','__node:=mppi_lifecycle_manager','-p','autostart:=true','-p',"node_names:=['controller_server']"])
        else:
            paths=Path(get_package_share_directory('cmu_local_planner'))/'paths'
            start('normalizer',['ros2','run','greenhouse_cloud_normalizer','greenhouse_cloud_normalizer','--ros-args',
                '-p','input_topic:=/perception/terrain_map','-p','output_topic:=/perception/terrain_map_odom','-p','target_frame:=odom'])
            start('planner',['ros2','run','cmu_local_planner','local_planner_node','--ros-args',
                '-p',f'path_folder:={paths}','-p','autonomy_mode:=true','-p','use_terrain_analysis:=true',
                '-p','terrain_map_topic:=/perception/terrain_map_odom','-p','goal_topic:=/planning/cmu_waypoint'])
            start('follower',['ros2','run','cmu_path_follower_generic','path_follower_generic','--ros-args',
                '-p','autonomy_mode:=true','-p','cmd_vel_topic:=/navigation/cmu_cmd'])
        start('terrain',['ros2','run','greenhouse_mppi_navigation','terrain_grid'])
        start('adapter',['ros2','run','greenhouse_mppi_navigation','command_adapter','--ros-args','-p',f'command_topic:=/navigation/{args.controller}_cmd'])
        start('mission',['ros2','run','greenhouse_mppi_navigation','mission','--ros-args','-p',f'controller_mode:={args.controller}','-p','planning_timeout:=2.0','-p','waypoint_timeout:=8.0'])
        gate={'source_topic':'/cmd_vel/nav','output_topic':'/cmd_vel','output_frame':'base_link',
              'qualification_scope':'software_interface_fixture','publish_rate_hz':20.0,
              'max_linear_velocity':.6,'max_angular_velocity':1.0,'max_linear_acceleration':.5,
              'max_angular_acceleration':1.0,'watchdog_timeout_sec':.3,'max_command_age_sec':.2,
              'future_tolerance_sec':.05,'auto_recover_after_watchdog':True}
        command=['ros2','run','greenhouse_cmd_gate','greenhouse_cmd_gate','--ros-args']
        for key,value in gate.items(): command+=['-p',f'{key}:={str(value).lower() if isinstance(value,bool) else value}']
        start('gate',command)
        rclpy.init(); node=Check(args.controller)
        try: node.run()
        finally: node.destroy_node(); rclpy.shutdown()
    finally:
        for process in processes:
            if process.poll() is None: os.killpg(process.pid,signal.SIGINT)
        for process in processes:
            try: process.wait(timeout=3)
            except subprocess.TimeoutExpired: os.killpg(process.pid,signal.SIGTERM); process.wait(timeout=3)
        for stream in files: stream.close()


if __name__=='__main__': main()
