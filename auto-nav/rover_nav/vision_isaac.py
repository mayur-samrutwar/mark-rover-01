"""Actual RGB transport and motor adapter for camera-only experiments.

The rendering engine necessarily owns scene state. Its transport boundary
returns only RGB pixels and the exposure clock; no pose, depth, segmentation,
body identity, ray casts, IMU or encoder measurement crosses into perception.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np

from .isaac import IsaacRover, ROOT, isaac
from .vision import CameraPerception, RGBPacket
from .vision_scene import vision_robot_xml, add_floor_texture
from .world import scene_xml, combine_rover


class RGBTransport:
    def __init__(self, sim, width, height):
        self._sim = sim
        self.width, self.height = width, height
        self._renderer = None
        self.frame_count = 0
        self.last_packet = None

    def read(self):
        runtime = self._sim["_camera_runtime"]
        def render():
            with runtime.lock:
                model, data = self._sim["model"], self._sim["data"]
                if self._renderer is None:
                    model.vis.global_.offwidth=max(self.width, model.vis.global_.offwidth)
                    model.vis.global_.offheight=max(self.height, model.vis.global_.offheight)
                    self._renderer=mujoco.Renderer(model,height=self.height,width=self.width)
                self._renderer.disable_depth_rendering()
                self._renderer.update_scene(data,camera="cam_mark_phone_rear")
                image=self._renderer.render().copy()
                return RGBPacket(image,float(data.time))
        self.frame_count += 1
        self.last_packet=runtime.submit(render)
        return self.last_packet

    def close(self):
        if self._renderer is not None:
            self._sim["_camera_runtime"].submit(self._renderer.close)
            self._renderer=None


class VisionRover(IsaacRover):
    def __init__(self, world):
        import uuid
        import xml.etree.ElementTree as ET
        xml, self.calibration = vision_robot_xml((ROOT/"assets"/"mark_rover.xml").read_text())
        placed=ET.fromstring(combine_rover(xml,world))
        worldbody=placed.find("worldbody")
        for child in list(worldbody):
            if child.tag != "body":
                worldbody.remove(child)
        xml=ET.tostring(placed,encoding="unicode")
        # Use Isaac's existing trusted-environment route for built-in texture
        # assets. The robot still passes its unchanged inline model validator.
        self.environment_id="vision_"+uuid.uuid4().hex
        directory=ROOT/"artifacts"/"runtime"/self.environment_id
        directory.mkdir(parents=True,exist_ok=True)
        path=directory/"scene.xml"
        path.write_text(add_floor_texture(scene_xml(world)))
        isaac.ENVIRONMENTS[self.environment_id]={"id":self.environment_id,"name":world["name"]+" · RGB", "path":str(path)}
        super().__init__(world,compiled_xml=xml,environment=self.environment_id)
        self.rgb=RGBTransport(self.sim,self.calibration["width"],self.calibration["height"])
        self.perception=CameraPerception(self.calibration)
        self._exposure_timestamp = None
        self._exposure_observation = None

    def _consume(self, packet):
        # Capture and control can request the same simulated exposure. Feed
        # that image to the estimator once; the pure perception boundary still
        # rejects genuinely repeated/nonmonotonic external frame timestamps.
        if getattr(self, "_exposure_timestamp", None) != packet.timestamp:
            self._exposure_observation = self.perception.process(packet)
            self._exposure_timestamp = packet.timestamp
        return self._exposure_observation

    def observe(self, *, tracking=True):
        # tracking argument is only an explicit fault-injection switch. All
        # actual visual tracking decisions are made from RGB features.
        result=self._consume(self.rgb.read())
        if not tracking:
            from dataclasses import replace
            result=replace(result,tracking=False)
        return result

    def capture(self, directory):
        from PIL import Image
        import io
        packet=self.rgb.read()
        observation=self._consume(packet)
        directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
        stem=f"capture_{self.capture_count:04d}"
        encoded=io.BytesIO();Image.fromarray(packet.pixels).save(encoded,format="PNG")
        pixels=encoded.getvalue();(directory/f"{stem}.png").write_bytes(pixels)
        metadata={"frame_id":self.capture_count,"simulation_time":packet.timestamp,
                  "image":f"{stem}.png","image_sha256":hashlib.sha256(pixels).hexdigest(),
                  "width":self.rgb.width,"height":self.rgb.height,"K":self.calibration["K"],
                  "T_base_camera":self.calibration["T_base_camera"],
                  "estimated_base_pose":vars(observation.pose),"tracking":observation.tracking,
                  "pose_source":"monocular_floor_visual_odometry","depth_source":None}
        (directory/f"{stem}.json").write_text(json.dumps(metadata,indent=2))
        self.capture_count+=1;self.capture_metadata.append(metadata)
        return metadata

    def close(self):
        self.rgb.close()
        super().close()
        isaac.ENVIRONMENTS.pop(self.environment_id,None)
