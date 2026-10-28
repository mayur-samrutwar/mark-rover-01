"""Closed-loop mission runner and independent ground-truth evaluation."""
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np
from scipy.ndimage import distance_transform_edt, label

from .isaac import IsaacRover, ROOT
from .navigation import Navigator
from .world import generate_world


class Mission:
    def __init__(self, seed=7, *, capture_dir=None, world=None, sensing="ideal"):
        self.seed = seed
        self.sensing = sensing
        self.world = world if world is not None else generate_world(seed)
        if sensing == "rgb" and world is None:
            # Scene setup only: begin with clearance for the configured
            # footprint and the initial visible-floor calibration patch.
            # The estimator still starts at arbitrary (0,0,0); neither this
            # spawn nor its surrounding geometry enters the controller.
            self.world["source"]["spawn"] = [1.2, 1.3, 0.]
        if sensing == "rgb":
            from .vision_isaac import VisionRover
            self.robot=VisionRover(self.world)
        elif sensing == "ideal":
            self.robot=IsaacRover(self.world)
        else:
            raise ValueError("Unknown sensing mode")
        bounds=(-3.,-3.,13.,11.) if sensing == "rgb" else tuple(self.world["source"]["nav_bounds"])
        self.nav = Navigator(bounds=bounds,
                             resolution=.05 if sensing == "rgb" else .10,
                             robot_radius=.25, safety_margin=.08,
                             account_cell_area=sensing == "rgb",
                             survey_clearance=.60 if sensing == "rgb" else None,
                             survey_angular_speed=.30 if sensing == "rgb" else .65)
        if sensing == "rgb":
            self.nav.initialize_own_footprint()
        self.localization_errors=[]
        self.trajectory = []
        self.rooms = [dict(room, visited=False, first_visit_time=None)
                      for room in self.world["source"]["rooms"]]
        self.capture_dir = Path(capture_dir).resolve() if capture_dir else None
        self.seen_capture_events = 0
        self.steps = 0
        self.finished = False
        self.failure = None
        self.elapsed = 0.
        self.started = time.monotonic()
        self.latest_observation = None
        self.last_command = (0., 0.)
        self.events = []
        self._pause_since = None
        sources = ["navigation.py", "mission.py", "isaac.py", "world.py"]
        if sensing == "rgb":
            sources += ["vision.py", "vision_isaac.py", "vision_scene.py", "floor_vision.py", "visual_odometry.py"]
        self.source_hashes = {name:hashlib.sha256((ROOT/"rover_nav"/name).read_bytes()).hexdigest()
                              for name in sources}

    def tick(self, *, tracking=True):
        if self.finished:
            return
        self.latest_observation = self.robot.observe(tracking=tracking)
        if self.sensing == "rgb" and self.robot.perception.last_vo.reason == "reference_too_old_reset_required":
            self.finished=True
            self.failure="visual_tracking_reset_required"
        if self.sensing == "rgb":
            measured=self.robot.pose()  # evaluator only; never passed to perception/controller
            estimated=self._local_to_world(self.latest_observation.pose.x,self.latest_observation.pose.y)
            yaw=self.latest_observation.pose.yaw+self.world["source"]["spawn"][2]
            error=math.hypot(estimated[0]-measured.x,estimated[1]-measured.y)
            angle=(yaw-measured.yaw+math.pi)%(2*math.pi)-math.pi
            self.localization_errors.append([error,angle])
        self.last_command = self.nav.update(self.latest_observation)
        if self.nav.status == "paused":
            if self._pause_since is None:
                self._pause_since = self.latest_observation.timestamp
            elif self.latest_observation.timestamp - self._pause_since > 3.:
                self.finished, self.failure = True, "perception_unavailable"
        else:
            self._pause_since = None
        self.robot.drive(*self.last_command)
        pose = self.robot.advance(.1)
        capture_events = self.nav.snapshot().get("capture_count", 0)
        if capture_events > self.seen_capture_events:
            self.seen_capture_events = capture_events
            if self.capture_dir:
                # Stop, settle against measured base speed, then acquire using
                # the unchanged mounted camera. Keep timing in simulated time.
                self.robot.drive(0., 0.)
                for _ in range(3 if self.sensing == "rgb" else 10):
                    self.robot.advance(.1)
                    if self.sensing != "rgb" and np.linalg.norm(self.robot.data.qvel[:6]) < .035:
                        break
                metadata = self.robot.capture(self.capture_dir)
                self.events.append({"type":"rgb_saved" if self.sensing == "rgb" else "rgbd_saved","frame_id":metadata["frame_id"],
                                    "time":metadata["simulation_time"]})
                pose = self.robot.pose()
        self.steps += 1
        now = float(self.robot.data.time)
        self.trajectory.append([pose.x, pose.y, pose.yaw, now])
        for room in self.rooms:
            x0, y0, x1, y1 = room["bounds"]
            # Physical entry well inside the room, not a doorway/map inference.
            if x0 + .45 < pose.x < x1 - .45 and y0 + .45 < pose.y < y1 - .45:
                if not room["visited"]:
                    room["visited"] = True
                    room["first_visit_time"] = now
                    self.events.append({"type": "room_entered", "room": room["name"], "time": now})
        state = str(self.nav.snapshot().get("status", "" )).lower()
        if self.robot.collision_count:
            self.failure = "physical_collision"
            self.finished = True
        if not np.isfinite(self.robot.data.qpos).all():
            self.failure = "invalid_physics"
            self.finished = True
        if state in {"complete", "completed", "blocked", "failed", "stalled"}:
            self.finished = True
            if state not in {"complete", "completed"}:
                self.failure = state
        if now > 1800:
            self.finished, self.failure = True, "mission_time_limit"
        if self.finished:
            self.robot.drive(0., 0.)
            self.robot.advance(.5)
        self.elapsed = time.monotonic() - self.started

    def _local_to_world(self,x,y):
        if self.sensing != "rgb":
            return x,y
        # Evaluator/display gauge alignment only. No transform from this method
        # is ever sent to the visual estimator or the navigation controller.
        sx,sy,yaw=self.world["source"]["spawn"]
        c,s=math.cos(yaw),math.sin(yaw)
        return sx+c*x-s*y,sy+s*x+c*y

    def coverage(self):
        """Evaluator only: compare observed map to reachable, inflated truth."""
        grid = self.nav.grid
        resolution = self.nav.resolution
        ox, oy = self.nav.origin
        yy, xx = np.indices(grid.shape)
        wx, wy = ox + (xx + .5)*resolution, oy + (yy + .5)*resolution
        wx,wy=self._local_to_world(wx,wy)
        inside = np.zeros(grid.shape, bool)
        for room in self.rooms:
            x0,y0,x1,y1=room["bounds"]
            # Room boundaries are shared floor, including doorway cells.
            # Partition geometry below determines whether an opening is closed.
            inside |= (wx>=x0)&(wx<=x1)&(wy>=y0)&(wy<=y1)
        occupied = ~inside
        for box in self.world["boxes"]:
            cx,cy,cz=box["center"]; sx,sy,sz=box["size"]
            if cz-sz/2 >= .40 or cz+sz/2 <= .035:
                continue
            occupied |= (abs(wx-cx)<=sx/2)&(abs(wy-cy)<=sy/2)
        safe = distance_transform_edt(~occupied)*resolution > .33
        components, _ = label(safe)
        x,y=(0.,0.) if self.sensing == "rgb" else self.world["source"]["spawn"][:2]
        col,row=int((x-ox)/resolution),int((y-oy)/resolution)
        component = components[row,col]
        reachable = (components == component) & safe if component else np.zeros_like(safe)
        return {"reachable_cells":int(reachable.sum()),
                "observed_reachable_cells":int(((grid!=-1)&reachable).sum()),
                "observed_reachable_fraction":float(((grid!=-1)&reachable).sum()/max(1,reachable.sum()))}

    def report(self):
        nav = self.nav.snapshot()
        coverage = self.coverage()
        all_rooms = all(room["visited"] for room in self.rooms)
        complete = str(nav.get("status", "")).lower() in {"complete", "completed"}
        captures_complete = self.capture_dir is None or self.robot.capture_count == nav.get("capture_count",0)
        success = (self.finished and complete and all_rooms and not self.failure
                   and self.robot.collision_count == 0 and captures_complete
                   and coverage["observed_reachable_fraction"] >= .95)
        return {"success":success,"seed":self.seed,"finished":self.finished,
                "navigation_state":nav.get("status"),"failure":self.failure,
                "all_rooms_visited":all_rooms,"rooms":self.rooms,
                "simulation_seconds":float(self.robot.data.time),"wall_seconds":self.elapsed,
                "distance_m":self.robot.distance,"collision_count":self.robot.collision_count,
                "max_collision_force_n":self.robot.max_collision_force,"steps":self.steps,
                "coverage":coverage,"captures_saved":self.robot.capture_count,
                "capture_events":nav.get("capture_count",0),"capture_complete":captures_complete,
                "capture_directory":str(self.capture_dir) if self.capture_dir else None,
                "sensor_model":("RGB-only floor visual odometry and image-derived floor boundaries" if self.sensing == "rgb" else "Ideal forward perspective depth rays; exact simulated pose"),
                "sensing":self.sensing,
                "source_sha256_at_start":self.source_hashes,
                "planner_configuration":{"resolution_m":self.nav.resolution,
                    "robot_radius_m":self.nav.robot_radius,"safety_margin_m":self.nav.safety_margin,
                    "cell_area_margin_m":self.nav.cell_area_margin,"allocated_bounds":self.nav.bounds,
                    "survey_clearance_m":self.nav.survey_clearance,
                    "survey_angular_speed_rad_s":self.nav.survey_angular_speed,
                    "turn_limit_rad_s":self.nav.turn_limit},
                "online_sensor_contract":("RGB pixels and exposure timestamps, plus fixed camera calibration"
                    if self.sensing == "rgb" else "Exact simulated pose and ideal ray depths"),
                "visual_perception":self.robot.perception.status() if self.sensing == "rgb" else None,
                "localization_evaluation":({"position_rmse_m":float(np.sqrt(np.mean(np.array(self.localization_errors)[:,0]**2))),
                     "max_position_error_m":float(np.max(np.array(self.localization_errors)[:,0])),
                     "final_position_error_m":self.localization_errors[-1][0],
                     "final_yaw_error_degrees":math.degrees(self.localization_errors[-1][1])} if self.localization_errors else None),
                "motion_model":"Original IsaacProtocol rover MJCF, velocity wheel actuators, MuJoCo contacts",
                "planner_inputs":("Image-derived planar pose and visible floor intervals only; no room labels or truth map"
                                  if self.sensing == "rgb" else "Exact simulated pose and perspective depth rays; no room labels or truth map"),
                "rover_sha256":hashlib.sha256((ROOT/"assets"/"mark_rover.xml").read_bytes()).hexdigest(),
                "events":self.events,"navigator":{k:v for k,v in nav.items() if k not in {"grid","map","cells","events"}}}

    def status(self, *, scene=False):
        pose = self.robot.pose()
        nav = dict(self.nav.snapshot())
        nav["state"] = nav["status"]
        nav["captures"] = self.robot.capture_count
        nav["coverage"] = self.coverage()["observed_reachable_fraction"] if self.finished else None
        nav["grid"] = {"origin":list(self.nav.origin),"resolution":self.nav.resolution,
                       "width":self.nav.grid.shape[1],"height":self.nav.grid.shape[0],
                       "cells":self.nav.grid.tolist()}
        if hasattr(self.nav, "path"):
            nav["path"] = [list(p) for p in self.nav.path]
        if self.sensing == "rgb":
            nav["path"]=[list(self._local_to_world(*p)) for p in nav["path"]]
            if nav.get("goal"):
                nav["goal"]=list(self._local_to_world(*nav["goal"]))
            nav["grid"]["origin"]=list(self._local_to_world(*self.nav.origin))
            nav["perception"]=self.robot.perception.status()
            if self.latest_observation is not None:
                estimated=self.latest_observation.pose
                ex,ey=self._local_to_world(estimated.x,estimated.y)
                nav["display_pose"]={"x":ex,"y":ey,"yaw":estimated.yaw+self.world["source"]["spawn"][2]}
        result = {"seed":self.seed,"sensing":self.sensing,"state":nav.get("status"),"sim_time":float(self.robot.data.time),
                  "pose":asdict(pose),"rooms":self.rooms,"trajectory":[p[:2] for p in self.trajectory],
                  "distance_m":self.robot.distance,"collision_count":self.robot.collision_count,
                  "navigation":nav,"frame":self.robot.frame(),"world":self.world,
                  "report":self.report() if self.finished else None}
        if scene:
            result["meshData"] = self.robot.mesh_data
        return result

    def save(self, directory):
        directory = Path(directory)
        directory.mkdir(parents=True,exist_ok=True)
        report = self.report()
        (directory/"report.json").write_text(json.dumps(report,indent=2))
        (directory/"trajectory.json").write_text(json.dumps(self.trajectory))
        if self.localization_errors:
            (directory/"localization_errors.json").write_text(json.dumps(self.localization_errors))
        (directory/"world.json").write_text(json.dumps(self.world,indent=2))
        (directory/"final_state.json").write_text(json.dumps(self.status(scene=True)))
        np.save(directory/"occupancy.npy",self.nav.grid)
        if self.sensing == "rgb":
            from PIL import Image
            packet=self.robot.rgb.last_packet
            if packet is not None:
                Image.fromarray(packet.pixels).save(directory/"last_camera.png")
            floor=self.robot.perception.last_floor
            if floor is not None:
                Image.fromarray(floor.mask.astype(np.uint8)*255).save(directory/"last_floor_mask.png")
            (directory/"camera_calibration.json").write_text(json.dumps(self.robot.calibration,indent=2))
        plot_mission(self,directory/"trajectory.png")
        return report

    def close(self):
        self.robot.close()


def plot_mission(mission,path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    fig,ax=plt.subplots(figsize=(11,9))
    ax.set_facecolor("#f3f5f6")
    for box in mission.world["boxes"]:
        cx,cy,cz=box["center"];sx,sy,sz=box["size"]
        if cz-sz/2 > .40: continue
        ax.add_patch(Rectangle((cx-sx/2,cy-sy/2),sx,sy,facecolor=box["color"][:3],edgecolor="#34404b",linewidth=.4))
    if mission.trajectory:
        points=np.array(mission.trajectory)
        ax.plot(points[:,0],points[:,1],color="#007e90",linewidth=1.4,label="Measured wheel-driven trajectory")
        ax.scatter(*points[0,:2],color="#258a45",s=65,zorder=5,label="Start")
        ax.scatter(*points[-1,:2],color="#de8226",s=65,zorder=5,label="Finish")
    for room in mission.rooms:
        x0,y0,x1,y1=room["bounds"]
        ax.text((x0+x1)/2,y1-.35,room["name"]+(" · visited" if room["visited"] else " · unvisited"),ha="center",weight="bold",fontsize=10)
    ax.set_aspect("equal"); ax.set_xlabel("World X (m)");ax.set_ylabel("World Y (m)")
    report=mission.report()
    ax.set_title(f"IsaacProtocol · Mark Rover · Seed {mission.seed}\n{mission.nav.snapshot()['status']} | {mission.robot.distance:.1f} m | collisions {mission.robot.collision_count} | observed reachable floor {report['coverage']['observed_reachable_fraction']:.1%}")
    ax.legend(loc="upper left");fig.tight_layout();fig.savefig(path,dpi=160);plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed",type=int,default=7)
    parser.add_argument("--output",type=Path)
    parser.add_argument("--max-seconds",type=float,default=1800)
    parser.add_argument("--capture",action="store_true")
    parser.add_argument("--sensing",choices=["ideal","rgb"],default="rgb")
    args=parser.parse_args()
    output=args.output or ROOT/"artifacts"/f"seed-{args.seed}"
    mission=Mission(args.seed,capture_dir=output/"captures" if args.capture else None,sensing=args.sensing)
    last=""
    try:
        while not mission.finished and mission.robot.data.time<args.max_seconds:
            mission.tick()
            state=str(mission.nav.snapshot().get("status"))
            if state != last or mission.steps%500==0:
                print(json.dumps({"time":round(mission.robot.data.time,1),"state":state,"reason":mission.nav.reason,
                    "perception":mission.robot.perception.status() if args.sensing == "rgb" else None,
                    "estimated_local_pose":asdict(mission.latest_observation.pose),"goal":mission.nav.goal,
                    "evaluation_position_error_m":mission.localization_errors[-1][0] if mission.localization_errors else None,
                    "pose":asdict(mission.robot.pose()),"visited":[r["name"] for r in mission.rooms if r["visited"]],"distance":round(mission.robot.distance,2)}),flush=True)
                last=state
        if not mission.finished:
            mission.finished,mission.failure=True,"test_time_limit"
            mission.robot.drive(0,0)
        report=mission.save(output)
        print(json.dumps(report,indent=2),flush=True)
        return 0 if report["success"] else 1
    finally:
        mission.close()


if __name__=="__main__":
    raise SystemExit(main())
