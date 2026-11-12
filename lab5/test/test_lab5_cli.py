"""CLI/read-only boundaries and live-adapter behavior with mocked feedback."""

import json
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('rclpy')
from ece470labs import lab5_exec as app
from ece470labs.lab5_func import default_settings

ROOT=Path(__file__).resolve().parents[2]


def test_offline_dry_run_never_initializes_ros(monkeypatch,tmp_path):
    def forbidden(*args,**kwargs):
        raise AssertionError('An offline dry run must not initialize ROS.')
    monkeypatch.setattr(app.rclpy,'init',forbidden)
    # Isolate task planning from camera/color processing.
    app.main(['--dry-run','--detections',str(ROOT/'lab5/examples/synthetic_detections.json'),
              '--config',str(ROOT/'lab5/examples/synthetic_config.json'),
              '--log-dir',str(tmp_path),'--no-gui'])
    record,=tmp_path.glob('*/run.json')
    data=json.loads(record.read_text())
    plans=[e for e in data['events'] if e['event']=='dry_run_plan']
    assert len(plans[0]['blocks'])==4
    assert not any(e['event']=='joint_command' for e in data['events'])


def test_first_only_plans_one_from_four_without_ros(monkeypatch,tmp_path):
    monkeypatch.setattr(app.rclpy,'init',lambda **kw:pytest.fail('Offline test initialized ROS'))
    app.main(['--dry-run','--first-only',
              '--detections',str(ROOT/'lab5/examples/synthetic_detections.json'),
              '--config',str(ROOT/'lab5/examples/synthetic_config.json'),
              '--log-dir',str(tmp_path),'--no-gui'])
    record,=tmp_path.glob('*/run.json')
    data=json.loads(record.read_text())
    snapshot=next(e for e in data['events'] if e['event']=='snapshot')
    selection=next(e for e in data['events'] if e['event']=='single_block_test')
    plan=next(e for e in data['events'] if e['event']=='dry_run_plan')
    assert len(snapshot['blocks'])==selection['scene_block_count']==4
    assert len(plan['blocks'])==1 and len(plan['blocks'][0]['poses'])==6
    assert data['settings']['task']['expected_blocks']==4
    assert not any(e['event'] in ['joint_command','suction_command'] for e in data['events'])


def test_first_only_still_rejects_an_incomplete_scene(monkeypatch,tmp_path):
    monkeypatch.setattr(app.rclpy,'init',lambda **kw:pytest.fail('Offline test initialized ROS'))
    blocks=json.loads((ROOT/'lab5/examples/synthetic_detections.json').read_text())[:3]
    source=tmp_path/'three.json';source.write_text(json.dumps(blocks))
    with pytest.raises(SystemExit) as exc:
        app.main(['--dry-run','--first-only','--detections',str(source),
                  '--config',str(ROOT/'lab5/examples/synthetic_config.json'),
                  '--log-dir',str(tmp_path/'logs'),'--no-gui'])
    assert exc.value.code==1
    record,=(tmp_path/'logs').glob('*/run.json')
    assert not any(e['event']=='single_block_test' for e in json.loads(record.read_text())['events'])


@pytest.mark.parametrize('outcome',['success','error','interrupt'])
def test_first_only_execution_attempts_one_and_closes_valve(monkeypatch,tmp_path,outcome):
    settings=json.loads((ROOT/'lab5/examples/synthetic_config.json').read_text())
    settings['simulation_only']=False
    config=tmp_path/'config.json';config.write_text(json.dumps(settings))
    blocks=[app.Detection(**d) for d in json.loads(
        (ROOT/'lab5/examples/synthetic_detections.json').read_text())]
    order=[];attempts=[]
    monkeypatch.setattr('builtins.input',lambda *a:'')
    monkeypatch.setattr(app.rclpy,'init',lambda **kw:None)
    monkeypatch.setattr(app.rclpy,'ok',lambda:True)
    monkeypatch.setattr(app.rclpy,'shutdown',lambda:order.append('ros_shutdown'))
    def fake_node(*a,**kw):
        image=np.zeros((480,640,3),np.uint8)
        return SimpleNamespace(
            latest=(image,image.copy(),{},blocks),image_time=time.monotonic(),
            image_error=None,io_client=object(),suction_enabled=False,
            wait_until_ready=lambda **kw:None,move_arm=lambda target:True,
            spin_for=lambda seconds:None,shutdown_suction=lambda:order.append('valve_closed'),
            destroy_node=lambda:order.append('node_destroyed'))
    monkeypatch.setattr(app,'UR3e',fake_node)
    def execute(robot,selected,run_settings):
        attempts.extend(selected)
        assert len(selected)==run_settings['task']['expected_blocks']==1
        if outcome=='error':raise RuntimeError('Test motion failure')
        if outcome=='interrupt':raise KeyboardInterrupt
        return dict(placed={'green':1,'yellow':0},missing=[])
    monkeypatch.setattr(app,'execute_tasks',execute)
    args=['--execute','--first-only','--config',str(config),
          '--log-dir',str(tmp_path/'logs'),'--no-gui']
    if outcome=='error':
        with pytest.raises(SystemExit) as exc:app.main(args)
        assert exc.value.code==1
    else:
        app.main(args)
    assert len(attempts)==1 and attempts[0].color=='green'
    assert order==['valve_closed','node_destroyed','ros_shutdown']


def test_saved_image_uses_the_native_detector_without_ros(monkeypatch,tmp_path):
    monkeypatch.setattr(app.rclpy,'init',lambda **kw:pytest.fail('Offline image must not initialize ROS'))
    calls=[]
    def native(image,settings,calibration):
        calls.append(image.shape)
        return image,image.copy(),{},[]
    monkeypatch.setattr(app,'analyze_image',native)
    app.main(['--vision-only','--image',str(ROOT/'lab5/examples/synthetic_scene.png'),
              '--log-dir',str(tmp_path),'--no-gui'])
    assert calls==[(480,640,3)]


def test_read_only_constructor_creates_no_command_interfaces(monkeypatch,tmp_path):
    monkeypatch.setattr(app.Node,'__init__',lambda *a,**kw:None)
    monkeypatch.setattr(app.UR3e,'create_publisher',lambda *a,**kw:pytest.fail('Read-only publisher'))
    monkeypatch.setattr(app.UR3e,'create_client',lambda *a,**kw:pytest.fail('Read-only service client'))
    monkeypatch.setattr(app.UR3e,'create_subscription',lambda *a,**kw:object())
    settings=default_settings()
    node=app.UR3e(settings,app.RunLog(tmp_path,settings,'test'))
    assert node.trajectory_pub is None and node.io_client is None


@pytest.mark.parametrize('args',[
    ['--execute'],
    ['--execute','--config','lab5/examples/synthetic_config.json'],
    ['--park-only','--image','lab5/examples/synthetic_scene.png'],
    ['--dry-run','--expected-blocks','-1'],
    ['--vision-only','--pause-after-snapshot'],
    ['--vision-only','--first-only'],
])
def test_invalid_or_synthetic_setup_fails_before_ros(monkeypatch,args):
    monkeypatch.chdir(ROOT)
    monkeypatch.setattr(app.rclpy,'init',lambda **kw:pytest.fail('Must fail before ROS initialization'))
    with pytest.raises(SystemExit) as exc:
        app.main(args)
    assert exc.value.code==2


def test_stale_or_missing_suction_feedback_cannot_be_treated_as_a_block():
    settings=default_settings()
    fake=SimpleNamespace(settings=settings,analog_value=None,io_time=0)
    with pytest.raises(RuntimeError,match='stale'):
        app.UR3e.grip_confirmed(fake)
    fake.analog_value=5
    fake.io_time=time.monotonic()-5
    with pytest.raises(RuntimeError,match='stale'):
        app.UR3e.grip_confirmed(fake)
    fake.io_time=time.monotonic()
    assert app.UR3e.grip_confirmed(fake)
    fake.analog_value=.5
    assert not app.UR3e.grip_confirmed(fake)


def test_read_only_adapter_has_no_motion_command_path():
    fake=SimpleNamespace(trajectory_pub=None)
    with pytest.raises(RuntimeError,match='disabled'):
        app.UR3e.move_arm(fake,[0]*6)
    fake.io_client=None
    with pytest.raises(RuntimeError,match='disabled'):
        app.UR3e.suction(fake,True)


def test_feedback_uses_pin_identity_not_message_order():
    fake=SimpleNamespace(settings=default_settings(),analog_value=None,io_time=0)
    msg=SimpleNamespace(analog_in_states=[SimpleNamespace(pin=1,state=99.),
                                         SimpleNamespace(pin=0,state=2.8)])
    app.UR3e.io_state_callback(fake,msg)
    assert fake.analog_value==2.8
    assert fake.io_time>0


def test_no_fresh_io_after_command_raises_instead_of_skipping_block():
    settings=default_settings()
    settings['gripper']['timeout_seconds']=.01
    fake=SimpleNamespace(settings=settings,io_time=1.,command_time=2.,spin_for=lambda _:None)
    with pytest.raises(RuntimeError,match='No new suction'):
        app.UR3e.wait_for_grip(fake,True)


def test_invalid_hsv_does_not_freeze_the_gui_event_loop(monkeypatch):
    calls=[]
    fake=SimpleNamespace(frozen_view=False,tune=None,
                         bridge=SimpleNamespace(imgmsg_to_cv2=lambda *a,**kw:np.zeros((4,4,3),np.uint8)),
                         settings=default_settings(),calibration=None,image_error=None,
                         get_logger=lambda:SimpleNamespace(error=lambda *_:None),
                         handle_key=lambda:calls.append('gui_pumped'))
    def invalid(*args):
        raise ValueError('HSV lower bound exceeds upper bound')
    monkeypatch.setattr(app,'analyze_image',invalid)
    app.UR3e.image_callback(fake,object())
    assert fake.image_error.startswith('HSV')
    assert calls==['gui_pumped']
