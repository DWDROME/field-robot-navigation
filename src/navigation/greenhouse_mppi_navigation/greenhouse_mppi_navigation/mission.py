"""FollowWaypoints missions using FAR routes with CMU or Nav2 FollowPath control."""
import copy
import json
import math
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from nav2_msgs.action import FollowPath, FollowWaypoints
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, Int64, String
from action_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point
from .core import Mission, Freshness, finite_pose


def stamp_ns(stamp):
    return stamp.sec*1000000000+stamp.nanosec


def pose_values(msg):
    p,q=msg.position,msg.orientation
    return (p.x,p.y,p.z,q.x,q.y,q.z,q.w)


class MissionNode(Node):
    def __init__(self):
        super().__init__('tracked_mission')
        self.group=ReentrantCallbackGroup(); self.lock=threading.RLock()
        self.mission=Mission(); self.busy=False; self.path_handle=None
        self.path_generation=0; self.control_feedback=False; self.control_failure=''
        self.route=None; self.route_time=-math.inf; self.routes={}; self.route_status=''; self.pending_status={}
        self.last_geometry=None; self.pose=None; self.pose_fresh=Freshness(.5)
        self.data_ready=False; self.data_time=-math.inf; self.token_counter=0
        self.controller_mode=self.declare_parameter('controller_mode','mppi').value
        if self.controller_mode not in {'cmu','mppi'}:
            raise ValueError('controller_mode must be cmu or mppi')
        self.waypoint=None; self.waypoint_fresh=Freshness(.5)
        self.local_path_fresh=Freshness(.5); self.cmu_goal_fence=math.inf
        self.cmu_route_fence=math.inf; self.cmu_started=False
        self.timeout=float(self.declare_parameter('waypoint_timeout',120.).value)
        self.path_timeout=float(self.declare_parameter('path_timeout',1.).value)
        self.planning_timeout=float(self.declare_parameter('planning_timeout',15.).value)
        self.xy=float(self.declare_parameter('xy_tolerance',.3).value)
        self.yaw=float(self.declare_parameter('yaw_tolerance',math.pi).value)
        if not all(math.isfinite(v) and v>0 for v in (self.timeout,self.path_timeout,self.planning_timeout,self.xy,self.yaw)):
            raise ValueError('mission timeouts and tolerances must be finite and positive')
        self.goal_pub=self.create_publisher(PointStamped,'/planning/global_goal',10)
        self.cancel_pub=self.create_publisher(Int64,'/planning/cancel_goal',10)
        self.enabled=self.create_publisher(Bool,'/navigation/control_enabled',10)
        self.status=self.create_publisher(String,'/navigation/mission_status',10)
        self.controller=None
        if self.controller_mode=='mppi':
            self.controller=ActionClient(self,FollowPath,'/follow_path',callback_group=self.group)
        else:
            if self.yaw<math.pi:
                raise ValueError('CMU supports position-only goals; use mppi for terminal yaw control')
            self.tf_buffer=Buffer(node=self); self.tf_listener=TransformListener(self.tf_buffer,self)
            self.local_goal_pub=self.create_publisher(PointStamped,'/planning/cmu_waypoint',10)
            self.create_subscription(PointStamped,'/planning/way_point',self.cmu_waypoint,10,callback_group=self.group)
            self.create_subscription(Path,'/planning/local_path',self.cmu_path,10,callback_group=self.group)
        self.create_subscription(Path,'/planning/global_path',self.path,10,callback_group=self.group)
        self.create_subscription(String,'/planning/route_status',self.route_result,10,callback_group=self.group)
        self.create_subscription(Odometry,'/localization/global_odometry',self.position,qos_profile_sensor_data,callback_group=self.group)
        self.create_subscription(Bool,'/navigation/data_ready',self.health,10,callback_group=self.group)
        self.server=ActionServer(self,FollowWaypoints,'/navigation/follow_waypoints',execute_callback=self.execute,
            goal_callback=self.accept,cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=self.group)
        self.create_timer(.1,self.heartbeat,callback_group=self.group,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def ros_now(self): return self.get_clock().now().nanoseconds*1e-9

    def health(self,msg):
        with self.lock: self.data_ready,self.data_time=msg.data,time.monotonic()

    def position(self,msg):
        with self.lock:
            if msg.header.frame_id!='map' or msg.child_frame_id!='base_link' or not finite_pose(pose_values(msg.pose.pose)):
                self.pose_fresh.receipt=-math.inf; return
            if self.pose_fresh.update(time.monotonic(),stamp_ns(msg.header.stamp)*1e-9,self.ros_now()):
                self.pose=pose_values(msg.pose.pose)

    def accept(self,request):
        with self.lock:
            valid=not self.busy and request.number_of_loops==0 and request.goal_index==0 and 0<len(request.poses)<=1000
            valid=valid and all(p.header.frame_id=='map' and finite_pose(pose_values(p.pose)) for p in request.poses)
            if not valid: return GoalResponse.REJECT
            self.busy=True
            return GoalResponse.ACCEPT

    def path(self,msg):
        with self.lock:
            if len(msg.poses)>10000: return
            token=stamp_ns(msg.header.stamp)
            self.routes[token]=(msg,time.monotonic())
            while len(self.routes)>8: del self.routes[next(iter(self.routes))]
            status=self.pending_status.pop(token,None)
            if status: self.apply_route_status(status)

    def route_result(self,msg):
        if len(msg.data)>2048: return
        try: status=json.loads(msg.data)
        except (ValueError,TypeError): return
        with self.lock:
            if not isinstance(status,dict): return
            self.apply_route_status(status)

    def apply_route_status(self,status):
        if status.get('goal_stamp_ns')!=self.mission.token: return
        self.route_status=status.get('status','invalid-path')
        if not isinstance(self.route_status,str) or self.route_status not in {'ready','reached','planning','not-ready','unreachable','stale-input','cancelled'}:
            self.route_status='invalid-path'
        token=status.get('plan_stamp_ns')
        if isinstance(token,bool) or not isinstance(token,int) or token<=0:
            self.route_status='invalid-path'; self.route=None; return
        item=self.routes.get(token)
        if self.route_status in {'ready','reached'} and not item:
            token=status.get('plan_stamp_ns')
            if not isinstance(token,int): return
            self.pending_status[token]=status
            while len(self.pending_status)>8: del self.pending_status[next(iter(self.pending_status))]
            return
        if self.route_status in {'ready','reached'} and item:
            route,receipt=item
            valid=route.header.frame_id=='map' and 2<=len(route.poses)<=10000 and all(
                p.header.frame_id=='map' and p.header.stamp==route.header.stamp and finite_pose(pose_values(p.pose)) for p in route.poses)
            if valid and -.1<=self.ros_now()-stamp_ns(route.header.stamp)*1e-9<=self.path_timeout:
                self.route,self.route_time=route,receipt
                if self.controller_mode=='cmu' and not math.isfinite(self.cmu_route_fence):
                    self.cmu_route_fence=stamp_ns(route.header.stamp)
            else: self.route_status='invalid-path'; self.route=None
        else:
            self.route=None

    def heartbeat(self):
        with self.lock:
            now=time.monotonic()
            active=self.mission.state=='controlling' and self.control_feedback and self.data_ready and now-self.data_time<.3
            active=active and self.route is not None and now-self.route_time<=self.path_timeout
            if self.controller_mode=='cmu':
                active=active and self.waypoint_fresh.valid(now,self.ros_now()) and self.local_path_fresh.valid(now,self.ros_now())
            self.enabled.publish(Bool(data=active))
            self.status.publish(String(data=json.dumps({'mission_id':self.mission.generation,'waypoint':self.mission.index,
                'goal_stamp_ns':self.mission.token,'state':self.mission.state,'reason':self.mission.reason})))

    def cancel_control(self):
        self.enabled.publish(Bool(data=False)); self.control_feedback=False
        self.path_generation+=1
        handle=self.path_handle; self.path_handle=None
        if handle: handle.cancel_goal_async()

    def next_goal(self):
        self.cancel_control(); self.cancel_pub.publish(Int64(data=self.mission.token))
        self.token_counter=max(self.token_counter+1,self.get_clock().now().nanoseconds)
        self.mission.token=self.token_counter; self.route=None; self.route_time=-math.inf
        self.route_status='planning'; self.last_geometry=None; self.control_failure=''; self.routes.clear(); self.pending_status.clear()
        self.waypoint=None; self.waypoint_fresh=Freshness(.5)
        self.local_path_fresh=Freshness(.5); self.cmu_goal_fence=math.inf
        self.cmu_route_fence=math.inf; self.cmu_started=False
        self.send_goal()

    def cmu_waypoint(self,msg):
        with self.lock:
            stamp=stamp_ns(msg.header.stamp)
            if self.mission.state not in {'planning','controlling'} or stamp<max(self.mission.token,self.cmu_route_fence):
                return
            if msg.header.frame_id!='map' or not all(math.isfinite(v) for v in (msg.point.x,msg.point.y,msg.point.z)):
                return
            if self.waypoint_fresh.update(time.monotonic(),stamp*1e-9,self.ros_now()):
                self.waypoint=msg

    def cmu_path(self,msg):
        with self.lock:
            stamp=stamp_ns(msg.header.stamp)*1e-9
            valid=msg.header.frame_id=='base_link' and 2<=len(msg.poses)<=10000
            valid=valid and all(math.isfinite(v) for p in msg.poses for v in (
                p.pose.position.x,p.pose.position.y,p.pose.position.z))
            if valid and stamp>=self.cmu_goal_fence:
                self.local_path_fresh.update(time.monotonic(),stamp,self.ros_now())
            elif not valid:
                self.local_path_fresh.receipt=-math.inf

    def follow_cmu(self):
        now=time.monotonic(); ros_now=self.ros_now()
        self.control_feedback=False
        if self.waypoint is None or not self.waypoint_fresh.valid(now,ros_now):
            if self.cmu_started: self.control_failure='CMU waypoint expired'
            return
        try:
            transform=self.tf_buffer.lookup_transform('odom','map',rclpy.time.Time.from_msg(self.waypoint.header.stamp))
            goal=do_transform_point(self.waypoint,transform)
        except TransformException:
            if self.cmu_started: self.control_failure='CMU waypoint transform unavailable'
            return
        goal.header.frame_id='odom'
        self.local_goal_pub.publish(goal)
        if not math.isfinite(self.cmu_goal_fence):
            self.cmu_goal_fence=ros_now
        self.control_feedback=self.local_path_fresh.valid(now,ros_now)
        if self.cmu_started and not self.control_feedback:
            self.control_failure='CMU local path expired'
        self.cmu_started=self.cmu_started or self.control_feedback

    def send_goal(self):
        msg=PointStamped(); msg.header.frame_id='map'
        msg.header.stamp.sec=self.mission.token//1000000000
        msg.header.stamp.nanosec=self.mission.token%1000000000
        msg.point.x,msg.point.y,msg.point.z=self.mission.goals[self.mission.index][:3]
        self.goal_pub.publish(msg)

    def follow(self,route):
        if self.controller_mode=='cmu':
            self.follow_cmu()
            return
        # Only meaningful route changes preempt the current action; fresh stamps alone do not.
        geometry=tuple(round(v,2) for p in route.poses for v in pose_values(p.pose))
        if geometry==self.last_geometry: return
        self.cancel_control(); self.last_geometry=geometry
        seq=self.path_generation; generation=self.mission.generation; token=self.mission.token
        goal=FollowPath.Goal(); goal.path=copy.deepcopy(route); goal.controller_id='FollowPath'
        # Preserve FAR geometry and apply the task's terminal orientation.
        q=goal.path.poses[-1].pose.orientation
        q.x,q.y,q.z,q.w=self.mission.goals[self.mission.index][3:]
        goal.goal_checker_id='goal_checker'; goal.progress_checker_id='progress_checker'
        def current(): return seq==self.path_generation and generation==self.mission.generation and token==self.mission.token
        def feedback(_):
            with self.lock:
                if current(): self.control_feedback=True
        def result(future):
            with self.lock:
                if not current(): return
                try:
                    value=future.result()
                    if value.status!=GoalStatus.STATUS_SUCCEEDED or value.result.error_code:
                        self.control_failure='controller failed: '+value.result.error_msg
                    elif self.mission.state=='controlling':
                        # Arrival is checked from fresh localization in execute, never from this result alone.
                        self.control_feedback=False
                except Exception as exc: self.control_failure=str(exc)
        def accepted(future):
            with self.lock:
                try:
                    handle=future.result()
                    if not current():
                        if handle and handle.accepted: handle.cancel_goal_async()
                        return
                    if not handle.accepted: self.control_failure='controller rejected path'; return
                    self.path_handle=handle
                    handle.get_result_async().add_done_callback(result)
                except Exception as exc: self.control_failure=str(exc)
        self.controller.send_goal_async(goal,feedback_callback=feedback).add_done_callback(accepted)

    def execute(self,handle):
        result=FollowWaypoints.Result()
        try:
            with self.lock:
                generation=self.mission.start([pose_values(p.pose) for p in handle.request.poses],time.monotonic())
                self.next_goal(); sent=time.monotonic()
            while rclpy.ok():
                with self.lock:
                    now=time.monotonic(); state=self.mission.state
                    if handle.is_cancel_requested: self.mission.terminate(generation,'cancelled','user cancellation')
                    elif now-self.mission.started>self.timeout: self.mission.terminate(generation,'timeout','waypoint timeout')
                    elif not self.data_ready or now-self.data_time>.3 or not self.pose_fresh.valid(now,self.ros_now()):
                        reason=('odometry, terrain or TF unavailable' if not self.data_ready else
                                'data readiness heartbeat expired' if now-self.data_time>.3 else
                                'global odometry unavailable or expired')
                        self.mission.terminate(generation,'failed',reason)
                    elif self.control_failure: self.mission.terminate(generation,'failed',self.control_failure)
                    elif self.route_status in {'unreachable','stale-input','invalid-path','cancelled'}:
                        self.mission.terminate(generation,'failed','FAR '+self.route_status)
                    elif self.pose:
                        x,y,z,w=self.pose[3:]; yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
                        if self.mission.arrived(generation,self.pose[:3],now,self.xy,yaw,self.yaw):
                            self.cancel_control()
                            if self.mission.state=='planning': self.next_goal(); sent=now
                    state=self.mission.state
                    if state not in {'planning','controlling'}: break
                    if self.route is not None:
                        if now-self.route_time>self.path_timeout:
                            self.mission.terminate(generation,'failed','FAR route expired'); continue
                        if self.controller is not None and not self.controller.server_is_ready():
                            self.mission.terminate(generation,'failed','FollowPath unavailable'); continue
                        self.mission.state='controlling'; self.follow(self.route)
                    elif now-self.mission.started>self.planning_timeout:
                        self.mission.terminate(generation,'timeout','FAR planning timeout')
                    elif now-sent>=1:
                        self.send_goal(); sent=now
                    feedback=FollowWaypoints.Feedback(); feedback.current_waypoint=self.mission.index
                    handle.publish_feedback(feedback)
                time.sleep(.05)
            with self.lock:
                self.cancel_control(); self.cancel_pub.publish(Int64(data=self.mission.token)); self.heartbeat()
                if self.mission.state=='completed': handle.succeed()
                elif self.mission.state=='cancelled': handle.canceled()
                else:
                    result.error_code=FollowWaypoints.Result.TASK_EXECUTOR_FAILED
                    result.error_msg=self.mission.reason or 'node shutting down'; handle.abort()
            return result
        finally:
            with self.lock: self.cancel_control(); self.busy=False


def main():
    rclpy.init(); node=MissionNode(); executor=MultiThreadedExecutor(num_threads=4); executor.add_node(node)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok(): node.cancel_control()
        executor.shutdown(timeout_sec=2); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
