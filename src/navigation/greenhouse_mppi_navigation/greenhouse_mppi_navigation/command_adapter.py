"""The sole controller-to-gate command owner; receipt watchdogs use a steady clock."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
from .core import CommandGuard,finite_pose


def seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


class CommandAdapter(Node):
    def __init__(self):
        super().__init__('mppi_command_adapter')
        self.guard=CommandGuard()
        self.command_topic=self.declare_parameter('command_topic','/navigation/mppi_cmd').value
        if self.command_topic not in {'/navigation/mppi_cmd','/navigation/cmu_cmd'}:
            raise ValueError('command_topic must select the MPPI or CMU command input')
        self.buffer=Buffer(node=self)
        self.listener=TransformListener(self.buffer,self)
        self.dynamic_map_tf=self.declare_parameter('require_dynamic_map_tf',True).value
        self.estop_topic=self.declare_parameter('estop_topic','').value
        enable_topic=self.declare_parameter('enable_topic','').value
        self.hardware_enable=self.create_publisher(Bool,enable_topic,1) if enable_topic else None
        self.estop=True if self.estop_topic else False
        self.estop_receipt=-math.inf
        self.terrain_ready=False
        self.terrain_receipt=-math.inf
        self.publisher=self.create_publisher(TwistStamped,'/cmd_vel/nav',1)
        self.ready=self.create_publisher(Bool,'/navigation/data_ready',10)
        self.create_subscription(TwistStamped,self.command_topic,self.command,1)
        self.create_subscription(Bool,'/navigation/control_enabled',self.enable,10)
        self.create_subscription(Odometry,'/localization/odometry',self.odometry,qos_profile_sensor_data)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid,'/perception/terrain_grid',self.terrain,qos)
        self.create_subscription(Bool,'/perception/terrain_ready',self.terrain_status,10)
        if self.estop_topic:
            self.create_subscription(Bool,self.estop_topic,self.estop_status,10)
        self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def ros_now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def command(self,msg):
        v=msg.twist
        if msg.header.frame_id not in {'','base_link'} or any(abs(x)>1e-9 for x in
            (v.linear.y,v.linear.z,v.angular.x,v.angular.y)):
            self.guard.command.receipt=-math.inf
            return
        self.guard.accept((v.linear.x,v.angular.z),seconds(msg.header.stamp),time.monotonic(),self.ros_now())

    def enable(self,msg):
        self.guard.enable(msg.data,time.monotonic(),self.ros_now())

    def odometry(self,msg):
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        values=(p.x,p.y,p.z,q.x,q.y,q.z,q.w,msg.twist.twist.linear.x,msg.twist.twist.angular.z)
        if msg.header.frame_id!='odom' or msg.child_frame_id!='base_link' or not finite_pose(values[:7]) or not all(math.isfinite(v) for v in values):
            self.guard.odom.receipt=-math.inf
            return
        self.guard.odom.update(time.monotonic(),seconds(msg.header.stamp),self.ros_now())

    def terrain(self,msg):
        if msg.header.frame_id!='odom' or not msg.data:
            self.guard.terrain.receipt=-math.inf
            return
        self.guard.terrain.update(time.monotonic(),seconds(msg.header.stamp),self.ros_now())

    def terrain_status(self,msg):
        self.terrain_ready=msg.data
        self.terrain_receipt=time.monotonic()

    def estop_status(self,msg):
        self.estop=msg.data
        self.estop_receipt=time.monotonic()

    def tick(self):
        wall,now=time.monotonic(),self.ros_now()
        tf_ready=True
        try:
            for parent,child in [('map','odom'),('odom','base_link')]:
                t=self.buffer.lookup_transform(parent,child,rclpy.time.Time())
                age=now-seconds(t.header.stamp)
                static_map=parent=='map' and not self.dynamic_map_tf and seconds(t.header.stamp)==0
                if not static_map and not -.1 <= age <= .5:
                    tf_ready=False
        except Exception:
            tf_ready=False
        health=tf_ready and self.guard.odom.valid(wall,now) and self.guard.terrain.valid(wall,now)
        health=health and self.terrain_ready and wall-self.terrain_receipt<=.3
        estop=self.estop or bool(self.estop_topic and wall-self.estop_receipt>.3)
        owners=sum(self.count_publishers(topic) for topic in ('/navigation/mppi_cmd','/navigation/cmu_cmd'))
        health=health and owners==1 and self.count_publishers(self.command_topic)==1 and self.count_publishers('/cmd_vel/nav')==1 and not estop
        self.ready.publish(Bool(data=health))
        v,w=self.guard.output(wall,now,tf_ready=health,owners=owners,estop=estop)
        message=TwistStamped(); message.header.stamp=self.get_clock().now().to_msg(); message.header.frame_id='base_link'
        message.twist.linear.x=v; message.twist.angular.z=w
        self.publisher.publish(message)
        if self.hardware_enable:
            self.hardware_enable.publish(Bool(data=health and self.guard.enabled and wall-self.guard.enable_receipt<=self.guard.enable_timeout))

    def stop(self):
        self.guard.enable(False,time.monotonic(),self.ros_now())
        msg=TwistStamped(); msg.header.stamp=self.get_clock().now().to_msg(); msg.header.frame_id='base_link'
        self.publisher.publish(msg)
        if self.hardware_enable: self.hardware_enable.publish(Bool(data=False))


def main():
    rclpy.init(); node=CommandAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok(): node.stop()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
