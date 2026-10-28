"""One actual 640 RGB exposure stream, compared at full and half resolution.

The physical drive is predetermined; estimators receive only their RGB images,
fixed calibrations and exposure time. Physical pose is recorded for evaluation.
"""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import sys

import cv2
import numpy as np

from rover_nav.floor_vision import FloorVision
from rover_nav.navigation import wrap_angle
from rover_nav.vision_isaac import VisionRover, ROOT
from rover_nav.visual_odometry import VisualOdometry
from rover_nav.world import generate_world
import rover_nav.vision_scene as scene


def run(output, seconds=30., legacy_path=Path("/private/tmp/mark-rover-visual-odometry-se2.py")):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    specification=importlib.util.spec_from_file_location("rover_nav._resolution_legacy",legacy_path)
    legacy=importlib.util.module_from_spec(specification);sys.modules[specification.name]=legacy
    specification.loader.exec_module(legacy)
    scene.CAMERA_WIDTH,scene.CAMERA_HEIGHT=640,480
    world=generate_world(7);world["source"]["spawn"]=[.37,2.3,math.pi/2]
    robot=VisionRover(world)
    calibrations={640:robot.calibration}
    low=dict(robot.calibration)
    low["width"],low["height"]=320,240
    intrinsic=np.asarray(robot.calibration["K"],dtype=float).copy()
    intrinsic[:2,:2]*=.5
    intrinsic[:2,2]=(intrinsic[:2,2]+.5)*.5-.5
    low["K"]=intrinsic.tolist();calibrations[320]=low
    floors={};estimators={}
    for width,calibration in calibrations.items():
        args=(calibration["K"],calibration["T_base_camera"],calibration["width"],calibration["height"])
        floors[width]=FloorVision(*args)
        estimators[f"pnp_{width}"]=VisualOdometry(*args)
        estimators[f"se2_{width}"]=legacy.VisualOdometry(*args)
    start=float(robot.data.time);initial=robot.pose();frames=[];records=[]
    try:
        while float(robot.data.time)-start<seconds-1e-7:
            packet=robot.rgb.read()
            images={640:packet.pixels,320:cv2.resize(packet.pixels,(320,240),interpolation=cv2.INTER_AREA)}
            floor_results={width:floors[width].process(image) for width,image in images.items()}
            results={}
            for name,estimator in estimators.items():
                width=int(name.split("_")[-1]);floor=floor_results[width]
                mask=floor.tracking_mask if floor.tracking_mask is not None else floor.mask
                results[name]=estimator.update(images[width],packet.timestamp,mask)
            # Truth never crosses either image estimator's input boundary.
            truth=robot.pose();c,s=math.cos(initial.yaw),math.sin(initial.yaw)
            dx,dy=truth.x-initial.x,truth.y-initial.y
            expected=[c*dx+s*dy,-s*dx+c*dy,wrap_angle(truth.yaw-initial.yaw)]
            record={"timestamp":packet.timestamp,"truth_local_evaluation_only":expected}
            for name,result in results.items():
                width=int(name.split("_")[-1]);floor=floor_results[width]
                record[name]={"tracking":result.tracking,"reason":result.reason,"inliers":result.inliers,
                    "features":result.features,"pose":vars(result.pose),
                    "position_error_m":math.hypot(result.pose.x-expected[0],result.pose.y-expected[1]),
                    "yaw_error_deg":math.degrees(wrap_angle(result.pose.yaw-expected[2])),
                    "floor_reason":floor.reason,"floor_tracking_pixels":int(np.sum(floor.tracking_mask))
                        if floor.tracking_mask is not None else int(np.sum(floor.mask))}
            frames.append(packet.pixels);records.append(record)
            if len(records)%50==0:
                print(json.dumps({"frames":len(records),"yaw_errors":{name:round(record[name]["yaw_error_deg"],3)
                    for name in estimators}}),flush=True)
            robot.drive(0.,.5);robot.advance(.1)
            if robot.collision_count:break
        report={"frames":len(records),"collision_count":robot.collision_count,
                "seconds":float(robot.data.time)-start,"same_underlying_exposures":True,
                "downsample":"640 RGB -> 320 RGB using area interpolation; pixel-center-correct K scaling"}
        for name in estimators:
            values=[record[name] for record in records]
            report[name]={"tracking_failures":sum(not value["tracking"] for value in values),
                "max_position_error_m":max(value["position_error_m"] for value in values),
                "final_position_error_m":values[-1]["position_error_m"],
                "max_yaw_error_deg":max(abs(value["yaw_error_deg"]) for value in values),
                "final_yaw_error_deg":values[-1]["yaw_error_deg"]}
        (output/"report.json").write_text(json.dumps(report,indent=2))
        (output/"diagnostics.json").write_text(json.dumps(records,indent=2,allow_nan=False))
        for width,calibration in calibrations.items():
            (output/f"calibration_{width}.json").write_text(json.dumps(calibration,indent=2))
        np.savez_compressed(output/"rgb_640_sequence.npz",rgb=np.array(frames),
            timestamps=np.array([record["timestamp"] for record in records]))
        print(json.dumps(report,indent=2),flush=True)
        return report
    finally:robot.close()


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,default=ROOT/"artifacts"/"vo-resolution-trial")
    parser.add_argument("--seconds",type=float,default=30.)
    args=parser.parse_args();run(args.output,args.seconds)
