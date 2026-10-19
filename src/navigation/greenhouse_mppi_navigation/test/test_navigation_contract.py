import copy
import math
import json
from pathlib import Path
import sys
import pytest
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from greenhouse_mppi_navigation.core import Mission,CommandGuard
from greenhouse_mppi_navigation.deployment import validate,DeploymentError


def test_mission_sequence_cancel_and_stale_generation():
    m=Mission(); poses=[(1.,0.,0.,0.,0.,0.,1.),(2.,0.,0.,0.,0.,0.,1.)]
    g=m.start(poses,0)
    assert not m.arrived(g,(0,0,0),1,.1)
    assert m.index==0
    assert m.arrived(g,(1,0,0),1,.1) and m.index==1
    assert m.arrived(g,(2,0,0),2,.1) and m.state=='completed'
    new=m.start(poses,3)
    assert not m.arrived(g,(1,0,0),4,.1)
    assert not m.terminate(g,'failed')
    assert m.terminate(new,'cancelled') and m.state=='cancelled'
    for state in ('failed','timeout'):
        new=m.start(poses,5); assert m.terminate(new,state)
    with pytest.raises(ValueError): m.start([(math.nan,0,0,0,0,0,1)],0)


def test_guard_stale_data_tf_estop_and_command_fence():
    g=CommandGuard()
    g.enable(True,10,100)
    assert not g.accept((.5,.1),99,10,100)
    g.odom.update(10,100,100); g.terrain.update(10,100,100)
    assert g.accept((.5,.1),100,10,100)
    assert g.output(10,100)==(.5,.1)
    assert g.output(10,100,tf_ready=False)==(0,0)
    assert g.output(10,100,owners=2)==(0,0)
    assert g.output(10,100,estop=True)==(0,0)
    assert g.output(11,101)==(0,0)
    assert not g.odom.update(10.2,100,100) and g.odom.receipt==10
    assert not g.accept((math.nan,0),100.1,10.1,100.1)
    assert g.output(10.1,100.1)==(0,0)
    g.enable(False,10.1,100.1)
    assert not g.accept((.5,0),100.2,10.2,100.2)
    assert g.output(10.2,100.2)==(0,0)
    g.enable(True,11,101)
    assert not g.accept((.5,0),100.9,11,101)


def test_deployment_rejects_missing_hardware_and_duplicate_controllers():
    config=Path(__file__).resolve().parents[1]/'config'
    sim=yaml.safe_load((config/'deployment-sim.yaml').read_text())
    assert validate(copy.deepcopy(sim))['runtime_ready']
    assert not validate(copy.deepcopy(sim))['hardware_qualified']
    for mode in ('cmu','mppi'):
        candidate=copy.deepcopy(sim); candidate['controllers']=[mode]
        assert validate(candidate)['controllers']==[mode]
    for update in ({'mode':'hardware'},{'controllers':['mppi','cmu']},{'controllers':[]},
                   {'controllers':['invalid']},{'simulation_fixture':False}):
        candidate=copy.deepcopy(sim); candidate.update(update)
        with pytest.raises(DeploymentError): validate(candidate)
    candidate=copy.deepcopy(sim); candidate['sensor']['imu_from_lidar_R'][0]=2
    with pytest.raises(DeploymentError): validate(candidate)
    with pytest.raises(DeploymentError): validate(yaml.safe_load((config/'deployment-hardware.template.yaml').read_text()))


def test_terminal_yaw_requires_mppi():
    config=Path(__file__).resolve().parents[1]/'config/deployment-sim.yaml'
    document=yaml.safe_load(config.read_text())
    document['mission']={'yaw_tolerance':0.1}
    with pytest.raises(DeploymentError,match='terminal yaw'): validate(copy.deepcopy(document))
    document['controllers']=['mppi']
    assert validate(document)['mission']['yaw_tolerance']==0.1


def test_hardware_inputs_and_udp_ports(tmp_path):
    document=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/deployment-sim.yaml').read_text())
    document['mode']='hardware'
    document['sensor']['network']={'host_ip':'192.168.1.5','lidar_ip':'192.168.1.12','interface':'fixture0'}
    document['chassis'].update(device='/dev/serial/by-id/fixture-device',baud=115200,estop_topic='/hardware/estop',enable_topic='/hardware/enable')
    driver={'lidar_summary_info':{'lidar_type':8},'MID360':{'host_net_info':{},'lidar_net_info':{}},
        'lidar_configs':[{'ip':'192.168.1.12','pcl_data_type':1,'pattern_mode':0,
            'extrinsic_parameter':{k:0 for k in ('roll','pitch','yaw','x','y','z')}}]}
    for i,name in enumerate(('cmd_data','push_msg','point_data','imu_data','log_data')):
        driver['MID360']['host_net_info'][name+'_ip']='192.168.1.5'
        driver['MID360']['host_net_info'][name+'_port']=56101+i*100
        driver['MID360']['lidar_net_info'][name+'_port']=56100+i*100
    path=tmp_path/'mid360.json'; path.write_text(json.dumps(driver)); document['sensor']['driver_config']=str(path)
    valid=validate(copy.deepcopy(document)); assert valid['runtime_ready'] and not valid['hardware_qualified']
    for value in (0,65536,True,'56301'):
        driver['MID360']['host_net_info']['point_data_port']=value; path.write_text(json.dumps(driver))
        with pytest.raises(DeploymentError,match='UDP port'): validate(copy.deepcopy(document))
