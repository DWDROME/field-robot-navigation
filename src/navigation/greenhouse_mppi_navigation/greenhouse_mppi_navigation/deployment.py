"""Validate explicit deployment inputs before launch creates hardware or controller actions."""
import argparse
import ipaddress
import json
import math
import os
from pathlib import Path
import subprocess
import re
import yaml


class DeploymentError(ValueError): pass


def number(value,label,positive=False):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or (positive and value<=0):
        raise DeploymentError(label+' must be a finite '+('positive ' if positive else '')+'number')
    return float(value)


def vector(value,length,label):
    if not isinstance(value,list) or len(value)!=length: raise DeploymentError(f'{label} requires {length} entries')
    return [number(v,label) for v in value]


def rotation(value,label):
    values=vector(value,9,label)
    for i in range(3):
        for j in range(3):
            if abs(sum(values[k*3+i]*values[k*3+j] for k in range(3))-(i==j))>1e-4:
                raise DeploymentError(label+' must be orthonormal')
    a,b,c,d,e,f,g,h,i=values
    if abs(a*(e*i-f*h)-b*(d*i-f*g)+c*(d*h-e*g)-1)>1e-4:
        raise DeploymentError(label+' must have determinant +1')
    return values


def validate(document,resources=False,base_dir=None):
    try:
        mode=document['mode']
        if mode not in {'simulation','hardware'}: raise DeploymentError('mode must be simulation or hardware')
        if document['localization']!='fast_lio_super': raise DeploymentError('localization must be fast_lio_super')
        if document['controllers'] not in (['cmu'], ['mppi']):
            raise DeploymentError('controllers must select exactly one of cmu or mppi')
        if not isinstance(document['loop_enabled'],bool): raise DeploymentError('loop_enabled must be bool')
        sensor=document['sensor']; chassis=document['chassis']; limits=chassis['limits']
        sensor['imu_from_lidar_T']=vector(sensor['imu_from_lidar_T'],3,'imu_from_lidar_T')
        sensor['imu_from_lidar_R']=rotation(sensor['imu_from_lidar_R'],'imu_from_lidar_R')
        sensor['base_from_lidar_xyzrpy']=vector(sensor['base_from_lidar_xyzrpy'],6,'base_from_lidar_xyzrpy')
        footprint=chassis['footprint']
        if not isinstance(footprint,list) or not 3<=len(footprint)<=100:
            raise DeploymentError('footprint requires 3..100 vertices')
        footprint=[vector(p,2,'footprint') for p in footprint]
        area=sum(footprint[i][0]*footprint[(i+1)%len(footprint)][1]-footprint[(i+1)%len(footprint)][0]*footprint[i][1] for i in range(len(footprint)))
        if abs(area)<.01: raise DeploymentError('footprint area is too small')
        for name in ('linear_velocity','angular_velocity','linear_acceleration','angular_acceleration'):
            limits[name]=number(limits[name],name,True)
        mission=document.setdefault('mission',{})
        if not isinstance(mission,dict): raise DeploymentError('mission must be a parameter mapping')
        for name,default in (('xy_tolerance',.3),('yaw_tolerance',math.pi),('waypoint_timeout',120.),('planning_timeout',15.),('path_timeout',1.)):
            mission[name]=number(mission.get(name,default),'mission.'+name,True)
        if mission['yaw_tolerance']>math.pi: raise DeploymentError('mission.yaw_tolerance must be <= pi')
        if document['controllers']==['cmu'] and mission['yaw_tolerance']<math.pi:
            raise DeploymentError('CMU supports position-only goals; use mppi for terminal yaw control')
        if chassis['motion_model']!='DiffDrive': raise DeploymentError('tracked chassis uses DiffDrive')
        if mode=='hardware':
            network=sensor['network']
            addresses=[]
            for name in ('host_ip','lidar_ip'):
                if not isinstance(network[name],str): raise DeploymentError(name+' must be an IPv4 string')
                address=ipaddress.IPv4Address(network[name])
                if address.is_loopback or address.is_multicast or address.is_unspecified: raise DeploymentError(name+' must be unicast IPv4')
                addresses.append(address)
            if addresses[0]==addresses[1]: raise DeploymentError('host_ip and lidar_ip must differ')
            interface=network['interface']
            if not isinstance(interface,str) or not interface or '/' in interface: raise DeploymentError('network.interface is required')
            device=chassis['device']
            if not isinstance(device,str) or not device.startswith('/dev/serial/by-id/') or len(Path(device).parts)!=5 or Path(device).name in {'.','..'}:
                raise DeploymentError('chassis.device requires a stable /dev/serial/by-id path')
            if isinstance(chassis['baud'],bool) or chassis['baud'] not in {9600,19200,38400,57600,115200,230400,460800,921600}:
                raise DeploymentError('unsupported chassis baud')
            for name in ('estop_topic','enable_topic'):
                if not isinstance(chassis[name],str) or not re.fullmatch(r'/(?:[A-Za-z_][A-Za-z0-9_]*/)*[A-Za-z_][A-Za-z0-9_]*',chassis[name]) or chassis[name] in {'/cmd_vel','/cmd_vel/nav'}:
                    raise DeploymentError(name+' must name an absolute Bool hardware interface')
            if chassis['estop_topic']==chassis['enable_topic']: raise DeploymentError('estop and enable must differ')
            config=Path(sensor['driver_config']).expanduser()
            if not config.is_absolute(): config=Path(base_dir or '.')/config
            if not config.is_file(): raise DeploymentError('sensor.driver_config is missing')
            driver=json.loads(config.read_text())
            if driver['lidar_summary_info']['lidar_type']!=8 or len(driver['lidar_configs'])!=1:
                raise DeploymentError('driver config requires one Mid-360')
            if driver['lidar_configs'][0]['ip']!=str(addresses[1]): raise DeploymentError('driver lidar_ip mismatch')
            for name in ('cmd_data_ip','push_msg_ip','point_data_ip','imu_data_ip'):
                if driver['MID360']['host_net_info'][name]!=str(addresses[0]): raise DeploymentError('driver host_ip mismatch')
            for section in ('host_net_info','lidar_net_info'):
                ports=[]
                for name in ('cmd_data_port','push_msg_port','point_data_port','imu_data_port','log_data_port'):
                    port=driver['MID360'][section][name]
                    if isinstance(port,bool) or not isinstance(port,int) or not 1<=port<=65535:
                        raise DeploymentError(section+'.'+name+' requires a UDP port in 1..65535')
                    ports.append(port)
                if len(set(ports))!=len(ports): raise DeploymentError(section+' ports must differ')
            for name,allowed in (('pcl_data_type',{1,2}),('pattern_mode',{0,1,2})):
                value=driver['lidar_configs'][0][name]
                if isinstance(value,bool) or not isinstance(value,int) or value not in allowed:
                    raise DeploymentError('invalid Mid-360 '+name)
            extrinsic=driver['lidar_configs'][0]['extrinsic_parameter']
            if any(number(extrinsic[name],'driver extrinsic')!=0 for name in ('roll','pitch','yaw','x','y','z')):
                raise DeploymentError('driver extrinsic must remain zero; FAST-LIO and static TF own calibration')
            sensor['driver_config']=str(config.resolve())
            if resources:
                if not os.access(device,os.R_OK|os.W_OK): raise DeploymentError('chassis device is unavailable or not writable')
                interfaces=json.loads(subprocess.check_output(['ip','-j','address','show','dev',interface],text=True))
                found={a.get('local') for item in interfaces for a in item.get('addr_info',[])}
                if str(addresses[0]) not in found: raise DeploymentError('host_ip is absent from network interface')
        elif document.get('simulation_fixture') is not True:
            raise DeploymentError('simulation inputs must declare simulation_fixture: true')
        document['runtime_ready']=True
        document['hardware_qualified']=False
        return document
    except DeploymentError: raise
    except (KeyError,TypeError,ValueError,OSError,subprocess.CalledProcessError) as exc:
        raise DeploymentError('missing or invalid deployment input: '+str(exc)) from exc


def load(path,resources=False):
    p=Path(path).expanduser().resolve()
    try: document=yaml.safe_load(p.read_text())
    except (OSError,yaml.YAMLError) as exc: raise DeploymentError(str(exc)) from exc
    return validate(document,resources,p.parent)


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('deployment'); parser.add_argument('--resources',action='store_true')
    args=parser.parse_args()
    try:
        document=load(args.deployment,args.resources)
        print(json.dumps({'mode':document['mode'],'runtime_ready':True,'hardware_qualified':False}))
    except DeploymentError as exc: parser.exit(2,str(exc)+'\n')
