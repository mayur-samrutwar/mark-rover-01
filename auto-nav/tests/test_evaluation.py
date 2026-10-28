"""Independent mission acceptance and geometry checks."""
from types import SimpleNamespace
import numpy as np
from rover_nav.mission import Mission
from rover_nav.navigation import Navigator
from rover_nav.world import generate_world


def evaluator(seed=7):
    mission=Mission.__new__(Mission)
    mission.sensing="ideal"
    mission.localization_errors=[]
    mission.source_hashes={}
    mission.world=generate_world(seed)
    mission.rooms=[dict(room,visited=False) for room in mission.world["source"]["rooms"]]
    mission.nav=Navigator(bounds=mission.world["source"]["nav_bounds"],robot_radius=.25,safety_margin=.08)
    return mission


def test_coverage_connects_all_rooms_through_open_doorway_cells():
    mission=evaluator()
    # More than the start room's ~900 reachable cells must contribute.
    mission.nav.grid.fill(0)
    result=mission.coverage()
    assert result["reachable_cells"]>2000
    assert result["observed_reachable_fraction"]==1.
    # Erase the entire bedroom: coverage must fall even if the living room is known.
    for row,col in np.ndindex(mission.nav.grid.shape):
        x,y=mission.nav.cell_to_world((row,col))
        if x>=5 and y>=4: mission.nav.grid[row,col]=-1
    assert mission.coverage()["observed_reachable_fraction"]<.85


def test_no_observations_cannot_pass_coverage():
    assert evaluator().coverage()["observed_reachable_fraction"]==0.


def test_all_rooms_and_clean_final_braking_are_both_required():
    mission=evaluator()
    mission.nav.grid.fill(0)
    mission.nav.status="complete"
    mission.rooms=[dict(room,visited=True) for room in mission.rooms]
    mission.robot=SimpleNamespace(collision_count=0,capture_count=0,distance=10.,
                                 max_collision_force=0.,data=SimpleNamespace(time=100.))
    mission.capture_dir=None
    mission.finished=True;mission.failure=None;mission.steps=1000;mission.elapsed=1.
    mission.seed=7;mission.events=[]
    assert mission.report()["success"]
    mission.robot.collision_count=1
    assert not mission.report()["success"]
    mission.robot.collision_count=0;mission.rooms[-1]["visited"]=False
    assert not mission.report()["success"]


def test_seed_changes_clutter_without_changing_room_topology():
    a,b=generate_world(7),generate_world(19)
    assert a["source"]["rooms"]==b["source"]["rooms"]
    assert a["boxes"]!=b["boxes"]
    assert generate_world(7)==a
    rooms=a["source"]["rooms"]
    assert len(rooms)==3
    centers=[((r["bounds"][0]+r["bounds"][2])/2,(r["bounds"][1]+r["bounds"][3])/2) for r in rooms]
    cross=(centers[1][0]-centers[0][0])*(centers[2][1]-centers[0][1])-(centers[1][1]-centers[0][1])*(centers[2][0]-centers[0][0])
    assert abs(cross)>10  # The room centres do not lie on a straight line.
