"""Compare image estimators on identical actual RGB; truth is evaluation only.

A fixed test drive travels straight for 3 seconds, settles for one second,
then turns in place. Neither estimator receives those commands. Both receive
the same rendered RGB exposures, timestamps and strict floor tracking masks.
"""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import sys

import numpy as np
import cv2

from rover_nav.floor_vision import FloorVision
from rover_nav.navigation import wrap_angle
from rover_nav.vision_isaac import VisionRover, ROOT
from rover_nav.visual_odometry import VisualOdometry
from rover_nav.world import generate_world


def run(output, legacy_path, seconds=30., near_wall=False):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    specification=importlib.util.spec_from_file_location("rover_nav._ab_legacy_se2",legacy_path)
    legacy=importlib.util.module_from_spec(specification)
    sys.modules[specification.name]=legacy;specification.loader.exec_module(legacy)
    (output/"legacy_visual_odometry.py").write_text(Path(legacy_path).read_text())
    world=generate_world(7)
    if near_wall:
        world["source"]["spawn"]=[.37,2.3,math.pi/2]
    robot=VisionRover(world)
    calibration=robot.calibration
    arguments=(calibration["K"],calibration["T_base_camera"],calibration["width"],calibration["height"])
    estimators={"legacy_se2":legacy.VisualOdometry(*arguments),"camera_pnp":VisualOdometry(*arguments),
                "camera_pnp_fresh_reseed":VisualOdometry(*arguments)}
    floor=FloorVision(*arguments)
    frames=[];records=[];start=float(robot.data.time)
    initial=robot.pose()
    try:
        while float(robot.data.time)-start<seconds-1e-7:
            packet=robot.rgb.read()
            sensing=floor.process(packet.pixels)
            mask=sensing.tracking_mask if sensing.tracking_mask is not None else sensing.mask
            results={name:estimator.update(packet.pixels,packet.timestamp,mask)
                     for name,estimator in estimators.items()}
            reseed=results["camera_pnp_fresh_reseed"]
            if reseed.tracking and reseed.inliers < .55*reseed.features:
                estimator=estimators["camera_pnp_fresh_reseed"]
                gray=cv2.cvtColor(packet.pixels,cv2.COLOR_RGB2GRAY)
                fresh=estimator._features(gray,estimator._mask(mask))
                estimator._seed_reference(gray,fresh,packet.timestamp)
            # Evaluation only, after both estimators have consumed their RGB.
            truth=robot.pose()
            c,s=math.cos(initial.yaw),math.sin(initial.yaw)
            dx,dy=truth.x-initial.x,truth.y-initial.y
            local=[c*dx+s*dy,-s*dx+c*dy,wrap_angle(truth.yaw-initial.yaw)]
            world_camera=np.eye(4)
            world_camera[:3,:3]=robot.data.cam_xmat[0].reshape(3,3)@np.diag([1.,-1.,-1.])
            world_camera[:3,3]=robot.data.cam_xpos[0]
            world_level=np.eye(4)
            c,s=math.cos(truth.yaw),math.sin(truth.yaw)
            world_level[:3,:3]=[[c,-s,0.],[s,c,0.],[0.,0.,1.]]
            world_level[:2,3]=[truth.x,truth.y]
            level_camera=np.linalg.inv(world_level)@world_camera
            level_base=level_camera@np.linalg.inv(np.asarray(calibration["T_base_camera"]))
            record={"timestamp":packet.timestamp,"truth_local_evaluation_only":local,
                "truth_camera_level_evaluation_only":level_camera.tolist(),
                "truth_base_tilt_deg":math.degrees(math.acos(float(np.clip(level_base[2,2],-1.,1.)))),
                "floor_reason":sensing.reason,"tracking_pixels":int(mask.sum())}
            for name,result in results.items():
                record[name]={"tracking":result.tracking,"reason":result.reason,
                    "pose":vars(result.pose),"inliers":result.inliers,
                    "position_error_m":math.hypot(result.pose.x-local[0],result.pose.y-local[1]),
                    "yaw_error_deg":math.degrees(wrap_angle(result.pose.yaw-local[2]))}
                if hasattr(estimators[name],"T_level_camera"):
                    record[name]["estimated_camera_level"]=estimators[name].T_level_camera.tolist()
            frames.append(packet.pixels);records.append(record)
            t=packet.timestamp-start
            command=(0.,.5) if near_wall else ((.15,0.) if t<3. else ((0.,0.) if t<4. else (0.,.5)))
            robot.drive(*command);robot.advance(.1)
            if robot.collision_count:
                break
        report={"frames":len(records),"collision_count":robot.collision_count,
            "seconds":float(robot.data.time)-start,
            "maximum_true_camera_tilt_deg":max(record["truth_base_tilt_deg"] for record in records),
            "true_camera_height_range_m":[min(record["truth_camera_level_evaluation_only"][2][3] for record in records),
                max(record["truth_camera_level_evaluation_only"][2][3] for record in records)]}
        camera=robot.rgb._renderer.scene.camera[0]
        report["renderer_frustum_calibration_evaluation_only"]={name:float(getattr(camera,name)) for name in
            ["frustum_center","frustum_width","frustum_bottom","frustum_top","frustum_near","frustum_far"]}
        for name in estimators:
            values=[record[name] for record in records]
            report[name]={"tracking_failures":sum(not value["tracking"] for value in values),
                "max_position_error_m":max(value["position_error_m"] for value in values),
                "final_position_error_m":values[-1]["position_error_m"],
                "max_yaw_error_deg":max(abs(value["yaw_error_deg"]) for value in values),
                "final_yaw_error_deg":values[-1]["yaw_error_deg"]}
        (output/"report.json").write_text(json.dumps(report,indent=2))
        (output/"diagnostics.json").write_text(json.dumps(records,indent=2,allow_nan=False))
        (output/"calibration.json").write_text(json.dumps(calibration,indent=2))
        np.savez_compressed(output/"rgb_sequence.npz",rgb=np.array(frames),
            timestamps=np.array([record["timestamp"] for record in records]))
        print(json.dumps(report,indent=2),flush=True)
        return report
    finally:
        robot.close()


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,default=ROOT/"artifacts"/"vo-pose-trial")
    parser.add_argument("--legacy",type=Path,default=Path("/private/tmp/mark-rover-visual-odometry-se2.py"))
    parser.add_argument("--seconds",type=float,default=30.)
    parser.add_argument("--near-wall",action="store_true")
    args=parser.parse_args();run(args.output,args.legacy,args.seconds,args.near_wall)
