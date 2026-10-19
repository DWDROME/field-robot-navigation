#!/usr/bin/env python3
"""Resolve the real launch factories without starting nodes or hardware."""
import copy
import importlib.util
import math
from pathlib import Path
import tempfile
import yaml
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

ROOT=Path(__file__).resolve().parents[1]


def module(name):
    spec=importlib.util.spec_from_file_location(name,ROOT/'launch'/f'{name}.launch.py')
    result=importlib.util.module_from_spec(spec); spec.loader.exec_module(result); return result


def main():
    assembly=module('closed_loop'); controllers={name:module(name) for name in ('cmu','mppi')}
    sim=yaml.safe_load((ROOT/'config/deployment-sim.yaml').read_text())
    with tempfile.TemporaryDirectory(prefix='launch-contract-') as folder:
        deployment=Path(folder)/'deployment.yaml'
        for mode,loop in ((mode,loop) for mode in controllers for loop in (True,False)):
            current=copy.deepcopy(sim); current['loop_enabled']=loop
            current['controllers']=[mode]
            deployment.write_text(yaml.safe_dump(current))
            for backend in ('ikdtree','octvox'):
                context=LaunchContext(); context.launch_configurations.update(deployment=str(deployment),map_backend=backend)
                actions=assembly.setup(context)
                included=[a for a in actions if isinstance(a,IncludeLaunchDescription)]
                assert len(included)==1
                included[0].launch_description_source.get_launch_description(context)
                location=included[0].launch_description_source.location
                assert location.endswith(f'/{mode}.launch.py'),location
                nodes=[a for a in actions+controllers[mode].setup(context) if isinstance(a,Node)]
                packages=[n.node_package for n in nodes]
                assert packages.count('fast_lio')==1 and packages.count('far_planner')==1
                assert packages.count('nav2_controller')==int(mode=='mppi')
                assert packages.count('nav2_lifecycle_manager')==int(mode=='mppi')
                assert packages.count('cmu_local_planner')==int(mode=='cmu')
                assert packages.count('cmu_path_follower_generic')==int(mode=='cmu')
                assert packages.count('greenhouse_cmd_gate')==1
                assert packages.count('fast_lio_super_ros2')==int(loop)
                assert packages.count('tf2_ros')==(1 if loop else 2) # sensor static plus mutually exclusive identity
                assert not {'local_planner','dlrobot_robot','livox_ros_driver2'} & set(packages)
                other=controllers['mppi' if mode=='cmu' else 'cmu']
                try: other.setup(context)
                except ValueError: pass
                else: raise AssertionError('controller launch accepted incompatible deployment')
                identities=[n for n in nodes if n.node_executable=='identity_odometry']
                assert len(identities)==int(not loop)
                print(f'PASS Jazzy launch controller={mode} backend={backend} loop={loop}: exclusive TF/controller, one gate',flush=True)
    sensor=copy.deepcopy(sim['sensor']); sensor['base_from_lidar_xyzrpy']=[1.,2.,3.,0.,0.,math.pi/2]
    sensor['imu_from_lidar_T']=[.1,.2,.3]
    t,r=assembly.imu_from_body(sensor)
    assert all(abs(a-b)<1e-9 for a,b in zip(t,[-1.9,1.2,-2.7]))
    assert all(abs(a-b)<1e-9 for a,b in zip(r,[0.,1.,0.,-1.,0.,0.,0.,0.,1.]))
    print('PASS explicit IMU/LiDAR/body transform composition',flush=True)


if __name__=='__main__': main()
