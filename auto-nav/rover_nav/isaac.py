"""Adapter to the existing IsaacProtocol MuJoCo runtime, without hardware I/O."""
from __future__ import annotations

import math
import os
from pathlib import Path
import sys

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
ISAAC_ROOT = Path(os.environ.get("ISAAC_PROTOCOL_ROOT", ROOT.parent.parent / "isaacprotocol"))
if not (ISAAC_ROOT / "simulation" / "main.py").is_file():
    raise RuntimeError("Set ISAAC_PROTOCOL_ROOT to the existing isaacprotocol checkout.")
sys.path.insert(0, str(ISAAC_ROOT))
from simulation import main as isaac
from simulation.camera_runtime import session_lock

from .navigation import Observation, Pose
from .world import combine_rover


class IsaacRover:
    """Real wheel actuators/contact physics; exact pose and ideal forward depth.

    Ray tracing uses only the phone's perspective frustum. Geometry and room
    labels are never passed to Navigator. This models ideal depth/pose, not
    ARCore drift, missing depth, appearance or physical phone performance.
    """

    def __init__(self, world, *, compiled_xml=None, environment="default"):
        self.world = world
        xml = compiled_xml or combine_rover((ROOT / "assets" / "mark_rover.xml").read_text(), world)
        started = isaac.start_custom_sim(isaac.CustomRobotRequest(xml=xml,environment=environment))
        self.sim = isaac.SIMULATIONS[started["simId"]]
        self.model, self.data = self.sim["model"], self.sim["data"]
        free = np.flatnonzero(self.model.jnt_type == mujoco.mjtJoint.mjJNT_FREE)
        if len(free) != 1 or self.model.nu != 4 or self.model.ncam != 1:
            raise ValueError("Expected original floating four-wheel rover and mounted phone camera.")
        self.root = int(self.model.jnt_bodyid[free[0]])
        self.robot_geoms = self.model.geom_bodyid != 0
        # Separate ray visibility only; contact masks/physics remain untouched.
        self.model.geom_group[self.robot_geoms & (self.model.geom_group < 3)] = 1
        self.model.geom_group[~self.robot_geoms] = 0
        self.mask = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
        self.left = []
        for actuator in range(self.model.nu):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator)
            self.left.append("left" in name.lower())
        if sum(self.left) != 2:
            raise ValueError("Could not identify both left wheel drives.")
        self.left = np.array(self.left)
        self.mesh_data = started["meshData"]
        runtime = self.sim["_camera_runtime"]
        for config in runtime.configs.values():
            config.update(width=320, height=240, fps=5, far=5.)
        self.collision_count = 0
        self.max_collision_force = 0.0
        self.distance = 0.0
        self.last_pose = self.pose()
        self.wheel_radius, self.track = .05, .268
        # Measured original skid-steer model: ideal .65 rad/s mix gives only
        # .116 rad/s in a 3s in-place trial. Feedforward compensates scrub;
        # the outer heading loop still uses measured pose, and wheel speeds
        # remain bounded by the unchanged model's 8 rad/s limit.
        self.turn_feedforward = 3.5
        self.max_range = 3.0
        # Perspective samples match the saved camera's vertical FoV, 4:3 image.
        self.fov_y = float(self.model.cam_fovy[0])
        self.half_y = math.tan(math.radians(self.fov_y) / 2)
        self.x_samples = np.linspace(-self.half_y * 4/3, self.half_y * 4/3, 81)
        yy = np.linspace(-self.half_y, self.half_y, 25)
        local = np.array([[x, y, -1.] for x in self.x_samples for y in yy])
        self.local_rays = local / np.linalg.norm(local, axis=1)[:, None]
        self.ray_dist = np.zeros(len(local))
        self.ray_ids = np.zeros(len(local), dtype=np.int32)
        self.ray_rows = len(yy)
        self.command = (0., 0.)
        self.command_time = float(self.data.time)
        self.capture_count = 0
        self.capture_metadata = []
        self.settle()

    def pose(self):
        mujoco.mj_forward(self.model, self.data)
        p = self.data.xpos[self.root]
        # Native CAD +Y up and -Z forward are transformed by MJCF export.
        forward = self.data.xmat[self.root].reshape(3, 3) @ np.array([0., 0., -1.])
        return Pose(float(p[0]), float(p[1]), math.atan2(forward[1], forward[0]))

    def settle(self):
        for _ in range(round(.5 / self.model.opt.timestep)):
            mujoco.mj_step(self.model, self.data)
        self.last_pose = self.pose()

    def observe(self, *, tracking=True):
        pose = self.pose()
        origin = self.data.cam_xpos[0].copy()
        rotation = self.data.cam_xmat[0].reshape(3, 3)
        rays = self.local_rays @ rotation.T
        mujoco.mj_multiRay(self.model, self.data, origin, rays.ravel(), self.mask,
                          1, -1, self.ray_ids, self.ray_dist, None, len(rays), 4.0)
        endpoints = origin + rays * np.maximum(self.ray_dist, 0)[:, None]
        planar = np.linalg.norm(endpoints[:, :2] - origin[:2], axis=1)
        # Floor returns are support, not upright obstacles. Project all other
        # occluders conservatively, including raised furniture; an occluder
        # above the robot does not justify clearing hidden space behind it.
        obstacle = ((self.ray_dist >= 0) & (endpoints[:, 2] > .035)
                    & (planar < self.max_range))
        ranges = np.where(obstacle, planar, self.max_range).reshape(-1, self.ray_rows).min(axis=1)
        middle = rays.reshape(-1, self.ray_rows, 3)[:, self.ray_rows // 2]
        angles = np.arctan2(middle[:, 1], middle[:, 0]) - pose.yaw
        angles = (angles + math.pi) % (2 * math.pi) - math.pi
        return Observation(pose=pose, angles=angles, ranges=ranges,
                           max_range=self.max_range, timestamp=float(self.data.time),
                           tracking=tracking, sensor_origin=(float(origin[0]), float(origin[1])))

    def drive(self, linear, angular):
        if not math.isfinite(linear) or not math.isfinite(angular):
            linear, angular = 0., 0.
        linear = float(np.clip(linear, -.25, .25))
        angular = float(np.clip(angular, -.8, .8))
        self.command = (linear, angular)
        self.command_time = float(self.data.time)
        left = (linear - angular * self.track / 2 * self.turn_feedforward) / self.wheel_radius
        right = (linear + angular * self.track / 2 * self.turn_feedforward) / self.wheel_radius
        scale = max(1., abs(left)/8., abs(right)/8.)
        left, right = left/scale, right/scale
        self.data.ctrl[:] = np.where(self.left, left, right)

    def advance(self, duration=.1):
        for _ in range(round(duration / self.model.opt.timestep)):
            if self.data.time - self.command_time > .35:
                self.data.ctrl[:] = 0  # independent command-age watchdog
            mujoco.mj_step(self.model, self.data)
            for i in range(self.data.ncon):
                contact = self.data.contact[i]
                a, b = int(contact.geom1), int(contact.geom2)
                if self.robot_geoms[a] == self.robot_geoms[b]:
                    continue
                static = b if self.robot_geoms[a] else a
                if self.model.geom_type[static] == mujoco.mjtGeom.mjGEOM_PLANE:
                    continue
                wrench = np.zeros(6)
                mujoco.mj_contactForce(self.model, self.data, i, wrench)
                force = float(np.linalg.norm(wrench[:3]))
                if force > .05:
                    self.collision_count += 1
                    self.max_collision_force = max(self.max_collision_force, force)
        pose = self.pose()
        self.distance += math.hypot(pose.x - self.last_pose.x, pose.y - self.last_pose.y)
        self.last_pose = pose
        return pose

    def frame(self):
        return isaac._build_frame(self.sim)

    def capture(self, directory):
        """Capture original mounted RGB-D camera through Isaac's GL worker."""
        runtime = self.sim["_camera_runtime"]
        frame = runtime.capture(next(iter(runtime.configs)))
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        stem = f"capture_{self.capture_count:04d}"
        # CameraRuntime returns encoded JPEG and float32 depth bytes.
        (directory / f"{stem}.png").write_bytes(frame["png"])
        (directory / f"{stem}.depth.f32").write_bytes(frame["depth"])
        import json
        import hashlib
        metadata = {**frame["metadata"],
                    "image":f"{stem}.png", "depth":f"{stem}.depth.f32",
                    "image_sha256":hashlib.sha256(frame["png"]).hexdigest(),
                    "depth_sha256":hashlib.sha256(frame["depth"]).hexdigest(),
                    "pose_source":"exact_simulated_pose", "depth_source":"MuJoCo_rendered_depth"}
        (directory / f"{stem}.json").write_text(json.dumps(metadata, indent=2))
        self.capture_metadata.append(metadata)
        self.capture_count += 1
        return metadata

    def close(self):
        self.drive(0., 0.)
        self.sim["_camera_runtime"].close()
        isaac.SIMULATIONS.pop(self.sim["id"], None)
