from pathlib import Path
from types import SimpleNamespace

from warehouse_mission_manager.cargo_manager import CargoManager
from warehouse_mission_manager.docking_controller import DockingController
from warehouse_mission_manager.mission_state import MissionState
import yaml


def test_missions_and_state_machine_contract():
    root = Path(__file__).parents[1]
    missions = yaml.safe_load((root / 'config' / 'missions.yaml').read_text())['missions']
    assert [mission['id'] for mission in missions] == ['M01', 'M02', 'M03', 'M04']
    assert [state.value for state in MissionState] == [
        'IDLE', 'ACCEPTED', 'GO_TO_PICKUP', 'WAIT_FOR_LOADING', 'CARGO_LOADED',
        'GO_TO_PRE_DOCK', 'DOCKING', 'DOCKED', 'UNLOADING', 'COMPLETED', 'NEXT_MISSION']


def test_cargo_invariant():
    cargo = CargoManager()
    cargo.load('M01_cargo')
    assert cargo.loaded and cargo.unload() == 'M01_cargo' and not cargo.loaded


def test_docking_acceptance_limits():
    pose = SimpleNamespace(position=SimpleNamespace(x=1.04, y=2.0),
                           orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
    twist = SimpleNamespace(linear=SimpleNamespace(x=0.0, y=0.0),
                            angular=SimpleNamespace(z=0.0))
    odom = SimpleNamespace(pose=SimpleNamespace(pose=pose),
                           twist=SimpleNamespace(twist=twist))
    assert DockingController.validate(odom, {'x': 1.0, 'y': 2.0, 'yaw': 0.0}, False)
    assert not DockingController.validate(odom, {'x': 1.0, 'y': 2.0, 'yaw': 0.0}, True)
