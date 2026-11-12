"""Synthetic vision and fake-hardware tests; no robot commands are sent."""

from pathlib import Path
import math
import json

import numpy as np
import pytest

from ece470labs.blob_search import (CameraCalibration, Detection, IMG2W,
                                   analyze_native_image, fit_calibration)
from ece470labs import blob_search as native_vision
from ece470labs.lab5_func import (default_settings, encoder_angles, execute_tasks,
                                 load_settings, move_block_vision, plan_tasks,
                                 validate_motion_settings, waypoints)

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture
def configured():
    return load_settings(ROOT/'lab5/examples/synthetic_config.json')


@pytest.fixture
def scene(configured):
    # Explicit fake coordinates isolate planning from deferred color selection.
    blocks = [Detection(**d) for d in json.loads(
        (ROOT/'lab5/examples/synthetic_detections.json').read_text())]
    return None, None, {}, blocks


def test_world_snapshot_uses_opencv_col_row_order(configured, scene):
    blocks = scene[3]
    calibration = CameraCalibration.from_dict(configured['camera'])
    assert len(blocks) == 4
    assert [b.color for b in blocks] == ['green','green','yellow','yellow']
    for block in blocks:
        np.testing.assert_allclose(calibration.image_to_world(block.col,block.row),
                                   block.world_xy,atol=1e-12)


def test_calibration_recovers_known_scale_rotation_translation():
    known=CameraCalibration(920,.31,-.08,.17,640,480)
    pixels=np.array([[110,130],[420,140],[430,360],[130,350]])
    world=np.array([known.image_to_world(*p) for p in pixels])
    fitted,errors=fit_calibration(pixels,world)
    np.testing.assert_allclose([fitted.beta,fitted.theta,fitted.tx,fitted.ty],
                               [known.beta,known.theta,known.tx,known.ty],atol=1e-10)
    assert max(errors)<1e-12
    np.testing.assert_allclose(IMG2W(240,160,fitted),known.image_to_world(240,160),atol=1e-12)


def test_appendix_b_translation_and_rotation_signs_independently():
    # At theta=+90deg: camera=(-world_y,world_x)+translation.
    calibration=CameraCalibration(1000,math.pi/2,.03,-.04)
    # world=(.10,.08) -> camera=(-.05,.06) -> pixel=(380,190).
    np.testing.assert_allclose(calibration.image_to_world(380,190),[.10,.08],atol=1e-12)
    pixels=np.array([[380,190],[480,190],[380,90],[480,90]],float)
    world=np.array([[.10,.08],[.20,.08],[.10,.18],[.20,.18]])
    fitted,errors=fit_calibration(pixels,world)
    np.testing.assert_allclose([fitted.beta,fitted.theta,fitted.tx,fitted.ty],
                               [1000,math.pi/2,.03,-.04],atol=1e-10)


def test_native_processing_keeps_starter_blob_search_calls(monkeypatch,configured):
    calls=[]
    def detector(image,color,**kwargs):
        calls.append((color,id(image),kwargs))
        detection=Detection(color,100,120,(.1,.2))
        return image,image.copy(),{color:np.zeros(image.shape[:2],np.uint8)},[detection]
    monkeypatch.setattr(native_vision,'blob_search',detector)
    frame=analyze_native_image(np.zeros((480,640,3),np.uint8),configured,
                               CameraCalibration.from_dict(configured['camera']))
    assert [c[0] for c in calls]==['green','yellow']
    assert calls[0][1]==calls[1][1]
    assert all(c[2]['show'] is False for c in calls)
    assert len(frame[3])==2


def test_duplicate_color_detections_are_rejected_before_motion(configured,scene):
    blocks=list(scene[3])
    blocks[2]=Detection('yellow',blocks[0].col,blocks[0].row,blocks[0].world_xy)
    robot=FakeRobot([True]*4)
    with pytest.raises(ValueError,match='Overlapping'):
        execute_tasks(robot,blocks,configured)
    assert robot.moves==[] and robot.outputs==[]


@pytest.mark.parametrize('color',['green','yellow'])
def test_occupied_stack_site_is_rejected_before_commands(configured,scene,color):
    blocks=list(scene[3])
    blocks[0]=Detection('green',100,100,configured['task']['stack_xy_m'][color])
    robot=FakeRobot([True]*4)
    with pytest.raises(ValueError,match='stack site empty'):
        execute_tasks(robot,blocks,configured)
    assert robot.moves==[] and robot.outputs==[]


@pytest.mark.parametrize('kind',['collinear','reflected','bad_units'])
def test_calibration_rejects_ambiguous_or_inconsistent_pairs(kind):
    pixels=np.array([[100.,100.],[400.,100.],[100.,350.],[400.,350.]])
    if kind=='collinear':
        pixels[:,1]=100
    calib=CameraCalibration(1000,0,0,0)
    world=np.array([calib.image_to_world(*p) for p in pixels])
    if kind=='reflected':
        world[:,0]*=-1
    if kind=='bad_units':
        world[0]+=[.05,.03]
    with pytest.raises(ValueError):
        fit_calibration(pixels,world)


def test_calibration_requires_measured_parameters_and_preserved_geometry(configured):
    with pytest.raises(ValueError):
        IMG2W(20,30)
    with pytest.raises(ValueError):
        CameraCalibration.from_dict(default_settings()['camera'])
    image=np.zeros((480,640,3),np.uint8)
    calib=CameraCalibration.from_dict(configured['camera'])
    with pytest.raises(ValueError,match='size'):
        analyze_native_image(image[:300],configured,calib)
    configured['vision']['flip_horizontal']=True
    with pytest.raises(ValueError,match='flip'):
        analyze_native_image(image,configured,calib)


def test_preflight_checks_all_colors_and_every_stack_height(configured,scene):
    selected=plan_tasks(scene[3],configured)
    assert len(selected)==4
    for block in selected:
        for _,point in waypoints(block.world_xy,[.1,.15,.06],configured):
            assert np.all(np.isfinite(encoder_angles(point,90)))


@pytest.mark.parametrize('problem',['missing_settings','count','outside','too_many','clearance','unreachable'])
def test_preflight_rejects_bad_setup_before_hardware(configured,scene,problem):
    blocks=scene[3]
    if problem=='missing_settings':
        configured['task']['block_height_m']=None
    elif problem=='count':
        blocks=blocks[:3]
    elif problem=='outside':
        blocks=[Detection('green',1,1,(4.,4.)),*blocks[1:]]
    elif problem=='too_many':
        configured['task']['max_per_color']=1
        configured['task']['expected_blocks']=2
        blocks=blocks[:2]
    elif problem=='clearance':
        configured['task']['safe_z_m']=.05
    else:
        configured['task']['workspace_xy_m']=[-3,3,-3,3]
        configured['task']['stack_xy_m']['green']=[2,2]
    with pytest.raises(ValueError):
        plan_tasks(blocks,configured)


class FakeRobot:
    def __init__(self,pick_outcomes):
        self.pick_outcomes=iter(pick_outcomes)
        self.holding=False
        self.carrying=False
        self.moves=[]
        self.events=[]
        self.outputs=[]

    def move_xyz(self,xyz):
        self.moves.append(list(xyz))
        return True

    def suction(self,enabled):
        self.outputs.append(enabled)
        self.holding=next(self.pick_outcomes) if enabled else False

    def wait_for_grip(self,expected):
        return self.holding==expected

    def grip_confirmed(self):
        return self.holding

    def emit(self,event,values):
        self.events.append((event,values))


def test_two_missing_blocks_are_reported_and_remaining_blocks_continue(configured,scene):
    robot=FakeRobot([False,True,True,False])
    result=execute_tasks(robot,scene[3],configured)
    assert result['placed']=={'green':1,'yellow':1}
    assert len(result['missing'])==2
    assert sum(name=='missing_block' for name,_ in robot.events)==2
    placed=[data for name,data in robot.events if name=='placed']
    assert all(math.isclose(data['target_xyz'][2],.03) for data in placed)
    assert robot.carrying is False
    assert robot.outputs[-1] is False


def test_successful_same_color_blocks_increase_stack_height(configured,scene):
    robot=FakeRobot([True]*4)
    result=execute_tasks(robot,scene[3],configured)
    assert not result['missing']
    placed=[data for name,data in robot.events if name=='placed']
    np.testing.assert_allclose([p['target_xyz'][2] for p in placed],[.03,.06,.03,.06])
    # execute_tasks freezes a separate list; caller's detections stay unchanged.
    assert len(scene[3])==4


def test_motion_failure_aborts_without_attempting_more_blocks(configured,scene):
    robot=FakeRobot([True]*4)
    robot.move_xyz=lambda xyz:False
    with pytest.raises(RuntimeError,match='Motion failed'):
        execute_tasks(robot,scene[3],configured)
    assert not any(event=='placed' for event,_ in robot.events)


def test_suction_loss_after_lift_aborts_transport(configured):
    robot=FakeRobot([True])
    robot.grip_confirmed=lambda:False
    with pytest.raises(RuntimeError,match='lost after lift'):
        move_block_vision(robot,[.12,.15],[.1,.25,.03],configured)
    assert robot.carrying  # adapter retains suction until operator resolves it
    assert len(robot.moves)==3  # no transfer or drop command after suction loss


def test_new_camera_detections_cannot_change_the_frozen_execution_list(configured,scene):
    detections=list(scene[3])
    robot=FakeRobot([True]*4)
    original_move=robot.move_xyz
    def update_live_camera(xyz):
        detections.clear()  # arm occludes the live view during pickup
        return original_move(xyz)
    robot.move_xyz=update_live_camera
    result=execute_tasks(robot,detections,configured)
    assert result['placed']=={'green':2,'yellow':2}
    assert detections==[]
