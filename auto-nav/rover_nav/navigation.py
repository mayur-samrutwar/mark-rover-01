"""Sensor-driven, conservative indoor frontier exploration.

The controller has no environment model or room coordinates. All traversability
comes from finite, timestamped range observations. Coordinates are metres and
angles are radians, with positive angular velocity counter-clockwise.
"""
from __future__ import annotations

from dataclasses import dataclass
from collections import deque
import heapq
import math
from typing import Any

import numpy as np
from scipy import ndimage


UNKNOWN, FREE, OCCUPIED = -1, 0, 1


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Observation:
    pose: Pose
    angles: np.ndarray
    ranges: np.ndarray
    max_range: float
    timestamp: float
    tracking: bool = True
    sensor_origin: tuple[float, float] | None = None
    min_ranges: np.ndarray | None = None
    hits: np.ndarray | None = None
    hit_ranges: np.ndarray | None = None
    free_intervals: tuple[tuple[tuple[float, float], ...], ...] | None = None


class Navigator:
    """Occupancy mapping, frontier selection, planning and closed-loop motion.

    ``update`` must be called for each new sensor frame. A command authority
    must also call ``check(now)`` independently: when frames stop, its zero
    command takes precedence over the last output of ``update``.
    """

    def __init__(self, bounds=(-.5, -.5, 10.5, 10.5), resolution=.10,
                 robot_radius=.25, safety_margin=.10, account_cell_area=False,
                 survey_clearance=None, survey_angular_speed=.65, turn_limit=.8):
        self.bounds = tuple(float(v) for v in bounds)
        self.resolution = float(resolution)
        self.robot_radius = float(robot_radius)
        self.safety_margin = float(safety_margin)
        self.inflation_radius = self.robot_radius + self.safety_margin
        # A ray passing through a grid cell proves a point inside that cell,
        # not its entire square. Reserve the half diagonal when using image
        # evidence with uncertain (rather than occupied) boundary intervals.
        self.cell_area_margin = math.sqrt(.5) * self.resolution if account_cell_area else 0.
        self.survey_clearance = float(survey_clearance) if survey_clearance is not None else None
        self.survey_angular_speed = float(survey_angular_speed)
        self.turn_limit = float(turn_limit)
        if not 0. < self.survey_angular_speed <= .8 or not 0. < self.turn_limit <= .8:
            raise ValueError("Invalid angular speed limits")
        if len(self.bounds) != 4 or resolution <= 0 or robot_radius <= 0 or safety_margin < 0:
            raise ValueError("Invalid map geometry or footprint")
        x0, y0, x1, y1 = self.bounds
        if x1 <= x0 or y1 <= y0:
            raise ValueError("Map bounds must have positive extent")
        self.origin = (x0, y0)
        self.shape = (math.ceil((y1 - y0) / resolution), math.ceil((x1 - x0) / resolution))
        self.grid = np.full(self.shape, UNKNOWN, dtype=np.int8)
        self._evidence = np.zeros(self.shape, dtype=np.int16)
        self._observed = np.zeros(self.shape, dtype=bool)
        self.status = "initial_survey"
        self.reason = "awaiting_observation"
        self.pose: Pose | None = None
        self.path: list[tuple[float, float]] = []
        self.goal: tuple[float, float] | None = None
        self.events: list[dict[str, Any]] = []
        self.stations: list[dict[str, Any]] = []
        self.frontier_count = 0
        self._undersized_frontier_cells = 0
        self._exhausted_frontier_candidates = 0
        self._minimum_frontier_cells = max(2, math.ceil(2 * self.inflation_radius / self.resolution) - 2)
        self._last_timestamp: float | None = None
        self._last_yaw: float | None = None
        self._sweep_progress = 0.
        self._next_capture = 0.
        self._survey_kind = "initial"
        self._survey_start_known = 0
        self._survey_start_time = 0.
        self._last_plan_time = -math.inf
        self._last_progress_time = 0.
        self._last_progress_xy: tuple[float, float] | None = None
        self._blacklist: list[tuple[float, float]] = []
        self._failed_goals = 0
        self._failure_anchor: tuple[float, float] | None = None
        self._blocked_replans = 0
        self._resume_status: str | None = None
        self._last_command = (0., 0.)
        self._final_scan_done = False
        self._mission_distance = 0.
        self._last_pose: Pose | None = None
        self._command_count = 0
        self._invalid_observations = 0
        self._captures = 0
        # Raw accepted RGB evidence only; the occupancy grid is not a temporal
        # substitute. At the 10 Hz mission cadence 64 entries cover the 5 s
        # age bound, while also bounding memory for faster callers.
        self._rgb_recent_observations: deque[Observation] = deque(maxlen=64)
        self._rgb_witness_used_recent = False
        self._rgb_temporal_corroboration_checks = 0

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        return (math.floor((y - self.origin[1]) / self.resolution),
                math.floor((x - self.origin[0]) / self.resolution))

    def cell_to_world(self, cell: tuple[int, int]) -> tuple[float, float]:
        row, col = cell
        return (self.origin[0] + (col + .5) * self.resolution,
                self.origin[1] + (row + .5) * self.resolution)

    def initialize_own_footprint(self):
        """Preflight prior: the placed rover's own swept footprint is clear.

        Only its physical radius is initialized, with no surrounding safe-start
        region. This prior is distinct from an image observation and cannot
        bridge a camera blind strip outside the robot's own footprint.
        """
        yy,xx=np.indices(self.shape)
        x=self.origin[0]+(xx+.5)*self.resolution
        y=self.origin[1]+(yy+.5)*self.resolution
        own=x*x+y*y <= self.robot_radius**2
        self._observed[own]=True
        self._evidence[own]=-12
        self.grid[own]=FREE

    def _inside(self, cell: tuple[int, int]) -> bool:
        return 0 <= cell[0] < self.shape[0] and 0 <= cell[1] < self.shape[1]

    def traversable(self) -> np.ndarray:
        """Unknown cells and occupied cells both constrain footprint clearance."""
        # Pad the map, so outside-map space is treated as unknown as well.
        free = np.pad(self.grid == FREE, 1, constant_values=False)
        clearance = ndimage.distance_transform_edt(free)[1:-1, 1:-1] * self.resolution
        return (self.grid == FREE) & (clearance >= self.inflation_radius + .02 + self.cell_area_margin)

    def _ray_cells(self, x: float, y: float, angle: float, length: float) -> np.ndarray:
        count = max(2, int(math.ceil(length / (self.resolution * .4))) + 1)
        distances = np.linspace(0, length, count)
        cols = np.floor((x + np.cos(angle) * distances - self.origin[0]) / self.resolution).astype(int)
        rows = np.floor((y + np.sin(angle) * distances - self.origin[1]) / self.resolution).astype(int)
        valid = (rows >= 0) & (rows < self.shape[0]) & (cols >= 0) & (cols < self.shape[1])
        return np.unique(np.column_stack((rows[valid], cols[valid])), axis=0)

    @staticmethod
    def _valid_free_intervals(observation: Observation) -> bool:
        """Validate immutable, ordered, disjoint floor segments per bearing."""
        intervals = observation.free_intervals
        if intervals is None:
            return True
        if (not isinstance(intervals, tuple) or len(intervals) != len(observation.angles)
                or observation.hits is None):
            return False
        for bearing in intervals:
            if not isinstance(bearing, tuple):
                return False
            previous_far = -math.inf
            for segment in bearing:
                if not isinstance(segment, tuple) or len(segment) != 2:
                    return False
                if any(not isinstance(value, (int, float, np.integer, np.floating))
                       or isinstance(value, (bool, np.bool_)) for value in segment):
                    return False
                near, far = (float(value) for value in segment)
                if (not math.isfinite(near) or not math.isfinite(far)
                        or near < 0. or far <= near or far > observation.max_range
                        or near < previous_far):
                    return False
                previous_far = far
        return True

    @staticmethod
    def _free_segments(observation: Observation):
        """Return only actual free segments, clipped before positive contacts.

        Explicit unions are authoritative, including empty unions. The old
        near/far pair is a compatibility fallback only when the union is None.
        Malformed explicit data never creates free evidence, even when this
        helper is called independently of update's frame validation.
        """
        count = len(observation.angles)
        if not Navigator._valid_free_intervals(observation):
            return ((),) * count
        result = []
        for index in range(count):
            if observation.free_intervals is None:
                far = float(observation.ranges[index])
                near = (0. if observation.min_ranges is None
                        else float(observation.min_ranges[index]))
                segments = (((near, far),) if math.isfinite(near) and math.isfinite(far)
                            and near >= 0. and far > near else ())
            else:
                segments = observation.free_intervals[index]
            limit = observation.max_range
            if observation.hits is not None and observation.hits[index]:
                contact = float(observation.ranges[index] if observation.hit_ranges is None
                                else observation.hit_ranges[index])
                if math.isfinite(contact) and contact > .02:
                    limit = min(limit, contact)
            result.append(tuple((float(near), min(float(far), limit))
                                for near, far in segments if min(float(far), limit) > near))
        return tuple(result)

    def integrate(self, observation: Observation) -> None:
        """Integrate actual segments without filling gaps or occluded shadows."""
        pose = observation.pose
        sensor_x, sensor_y = observation.sensor_origin or (pose.x, pose.y)
        free_cells, hit_cells = [], []
        all_segments = self._free_segments(observation)
        for index, (angle, measured) in enumerate(zip(observation.angles, observation.ranges)):
            if not math.isfinite(float(angle)):
                continue
            direction = pose.yaw + float(angle)
            valid_free = math.isfinite(float(measured)) and measured > .02
            distance = min(float(measured), observation.max_range) if valid_free else math.nan
            hit = (bool(observation.hits[index]) if observation.hits is not None
                   else valid_free and measured < observation.max_range - .025)
            hit_distance = (float(observation.hit_ranges[index])
                            if observation.hit_ranges is not None else distance)
            if hit and (not math.isfinite(hit_distance) or hit_distance <= .02):
                hit=False
            endpoint = None
            if hit and hit_distance <= observation.max_range:
                endpoint = self.world_to_cell(sensor_x + hit_distance * math.cos(direction),
                                              sensor_y + hit_distance * math.sin(direction))
                if self._inside(endpoint):
                    hit_cells.append(endpoint)
            for minimum, distance in all_segments[index]:
                # Preserve the baseline's inferred positive endpoint too.
                if hit and hit_distance <= observation.max_range:
                    distance = min(distance, hit_distance)
                if distance <= minimum or distance <= .02:
                    continue
                start_x = sensor_x + minimum * math.cos(direction)
                start_y = sensor_y + minimum * math.sin(direction)
                cells = self._ray_cells(start_x, start_y, direction, distance - minimum)
                if len(cells):
                    if endpoint is not None:
                        cells = cells[np.any(cells != endpoint, axis=1)]
                    free_cells.append(cells)
        if free_cells:
            cells = np.unique(np.concatenate(free_cells, axis=0), axis=0)
            if len(cells):
                rows, cols = cells.T
                self._observed[rows, cols] = True
                self._evidence[rows, cols] = np.maximum(-12, self._evidence[rows, cols] - 1)
        # Endpoint evidence wins over crossing rays from this same frame.
        if hit_cells:
            cells = np.unique(np.asarray(hit_cells), axis=0)
            rows, cols = cells.T
            self._observed[rows, cols] = True
            self._evidence[rows, cols] = np.minimum(20, np.maximum(0, self._evidence[rows, cols]) + 5)
        self.grid.fill(UNKNOWN)
        self.grid[self._observed] = FREE
        self.grid[self._observed & (self._evidence > 0)] = OCCUPIED

    @staticmethod
    def _neighbors(cell, safe):
        row, col = cell
        height, width = safe.shape
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1),
                       (-1, -1), (-1, 1), (1, -1), (1, 1)):
            nr, nc = row + dr, col + dc
            if not (0 <= nr < height and 0 <= nc < width and safe[nr, nc]):
                continue
            if dr and dc and not (safe[row + dr, col] and safe[row, col + dc]):
                continue
            yield (nr, nc), math.sqrt(2) if dr and dc else 1.

    def astar(self, start, goal, safe: np.ndarray | None = None) -> list[tuple[int, int]]:
        """Eight-connected A*, rejecting diagonal corner cuts."""
        safe = self.traversable() if safe is None else safe
        if not self._inside(start) or not self._inside(goal) or not safe[start] or not safe[goal]:
            return []
        queue = [(0., 0., start)]
        costs, parents = {start: 0.}, {}
        while queue:
            _, distance, cell = heapq.heappop(queue)
            if distance > costs[cell] + 1e-9:
                continue
            if cell == goal:
                path = [cell]
                while cell in parents:
                    cell = parents[cell]
                    path.append(cell)
                return path[::-1]
            for neighbor, step in self._neighbors(cell, safe):
                new_cost = distance + step
                if new_cost < costs.get(neighbor, math.inf):
                    costs[neighbor], parents[neighbor] = new_cost, cell
                    heuristic = math.hypot(goal[0] - neighbor[0], goal[1] - neighbor[1])
                    heapq.heappush(queue, (new_cost + heuristic, new_cost, neighbor))
        return []

    def _reachable_distances(self, start, safe):
        distances = np.full(self.shape, math.inf)
        distances[start] = 0.
        queue = [(0., start)]
        while queue:
            distance, cell = heapq.heappop(queue)
            if distance > distances[cell] + 1e-9:
                continue
            for neighbor, step in self._neighbors(cell, safe):
                new = distance + step * self.resolution
                if new < distances[neighbor]:
                    distances[neighbor] = new
                    heapq.heappush(queue, (new, neighbor))
        return distances

    def _start_cell(self, safe):
        if self.pose is None:
            return None
        cell = self.world_to_cell(self.pose.x, self.pose.y)
        if self._inside(cell) and safe[cell]:
            return cell
        # Quantization may put the centre just outside the conservative margin.
        # Only bridge one neighbouring cell, never a wall or an unknown region.
        for neighbor in ((cell[0] + dr, cell[1] + dc) for dr in (-1, 0, 1) for dc in (-1, 0, 1)):
            if self._inside(neighbor) and safe[neighbor] and self._inside(cell) and self.grid[cell] == FREE:
                return neighbor
        return None

    def select_frontier(self, safe: np.ndarray | None = None):
        """Return a reachable, verified-free viewpoint near a frontier cluster."""
        safe = self.traversable() if safe is None else safe
        start = self._start_cell(safe)
        if start is None:
            self.frontier_count = 0
            return None
        distances = self._reachable_distances(start, safe)
        reachable = np.isfinite(distances)
        raw = (self.grid == FREE) & ndimage.binary_dilation(self.grid == UNKNOWN, structure=np.ones((3, 3)))
        labels, count = ndimage.label(raw, structure=np.ones((3, 3)))
        self.frontier_count = 0
        self._undersized_frontier_cells = 0
        self._exhausted_frontier_candidates = 0
        if not np.any(reachable):
            return None
        gain_window = max(3, int(round(3.1 / self.resolution)) | 1)
        unknown_gain = (ndimage.uniform_filter((self.grid == UNKNOWN).astype(float), size=gain_window)
                        * (gain_window * self.resolution) ** 2)
        boundaries = []
        for label in range(1, count + 1):
            boundary = np.argwhere(labels == label)
            # Interfaces much narrower than the inflated footprint cannot be
            # crossed. Count their residual cells explicitly in the report.
            if len(boundary) < self._minimum_frontier_cells:
                self._undersized_frontier_cells += len(boundary)
                continue
            self.frontier_count += 1
            boundaries.append(boundary[::max(1, len(boundary) // 100)])
        # A scan station needs floor visible in every heading. Prefer open
        # viewpoints while allowing the path itself to pass narrow doors.
        # Low-clearance candidates remain a fallback when no open viewpoint
        # can reveal a reachable frontier.
        clearance = ndimage.distance_transform_edt(self.grid == FREE) * self.resolution
        tiers = [self.survey_clearance, .50, None] if self.survey_clearance is not None else [None]
        for required in tiers:
            viewpoints = reachable if required is None else reachable & (clearance >= required)
            if not np.any(viewpoints):
                continue
            _, nearest = ndimage.distance_transform_edt(~viewpoints, return_indices=True)
            candidates = {}
            for boundary in boundaries:
                for row, col in boundary:
                    candidate = (int(nearest[0, row, col]), int(nearest[1, row, col]))
                    if math.hypot(candidate[0] - row, candidate[1] - col) * self.resolution > max(self.inflation_radius + .30, (required or 0.) + .12):
                        continue
                    x, y = self.cell_to_world(candidate)
                    if any(math.hypot(x - bx, y - by) < .70 for bx, by in self._blacklist):
                        continue
                    if any(math.hypot(x - station["x"], y - station["y"]) < .60 for station in self.stations):
                        self._exhausted_frontier_candidates += 1
                        continue
                    length = float(distances[candidate])
                    if length < .35 or unknown_gain[candidate] < .08:
                        continue
                    score = (1. + min(float(unknown_gain[candidate]), 1.8) / 1.8) / (.8 + length)
                    candidates[candidate] = score
            if candidates:
                return max(candidates, key=candidates.get)
        return None

    def _begin_survey(self, kind: str, now: float):
        self.status = "initial_survey" if kind == "initial" else "capture_sweep"
        self.reason = kind + "_survey"
        self._survey_kind = kind
        self._sweep_progress = 0.
        self._next_capture = 0.
        self._survey_start_known = int(np.sum(self.grid != UNKNOWN))
        self._survey_start_time = now
        self._last_yaw = self.pose.yaw
        self.path = []
        # A survey deliberately holds translation for a full revolution. Its
        # duration is not evidence that the next drive goal has stalled.
        self._last_progress_xy = None
        self._last_progress_time = now

    def _survey(self, now: float):
        if self._sweep_progress >= 2 * math.pi - .035:
            self.stations.append({"id": len(self.stations), "x": self.pose.x, "y": self.pose.y,
                                  "timestamp": now, "kind": self._survey_kind,
                                  "new_cells": int(np.sum(self.grid != UNKNOWN)) - self._survey_start_known})
            self.events.append({"type": "survey_complete", **self.stations[-1]})
            if self._survey_kind == "final":
                self._final_scan_done = True
            self.status = "planning"
            self.reason = "survey_complete"
            return (0., 0.)
        if self._sweep_progress + .025 >= self._next_capture:
            self.events.append({"type": "capture", "station": len(self.stations), "timestamp": now,
                                "x": self.pose.x, "y": self.pose.y, "yaw": self.pose.yaw,
                                "heading_index": self._captures % 12})
            self._captures += 1
            self._next_capture += 2 * math.pi / 12
            # Translation and rotation are both stopped for each capture frame.
            return (0., 0.)
        return (0., self.survey_angular_speed)

    def _line_safe(self, start_xy, end_xy, safe):
        length = math.dist(start_xy, end_xy)
        count = max(2, int(length / (self.resolution * .4)) + 1)
        for t in np.linspace(0., 1., count):
            cell = self.world_to_cell(start_xy[0] + t * (end_xy[0] - start_xy[0]),
                                      start_xy[1] + t * (end_xy[1] - start_xy[1]))
            if not self._inside(cell):
                return False
            if not safe[cell]:
                # A measured centre can quantize into the edge of the safety
                # margin between frames. Permit a short return to the planned
                # safe corridor only while the physical footprint stays clear.
                if t * length > self.resolution * 1.6 or self.grid[cell] != FREE:
                    return False
                physical_clearance = self.robot_radius + .025 + self.cell_area_margin
                radius_cells = math.ceil(physical_clearance / self.resolution)
                row, col = cell
                for dr in range(-radius_cells, radius_cells + 1):
                    for dc in range(-radius_cells, radius_cells + 1):
                        if math.hypot(dr, dc) * self.resolution > physical_clearance:
                            continue
                        nearby = (row + dr, col + dc)
                        if not self._inside(nearby) or self.grid[nearby] != FREE:
                            return False
        return True

    @staticmethod
    def _rgb_braking_paths(speed, angular, previous_angular=0., *, observation_age=0.):
        """Bounded simulated stopping trajectories, expressed in base XY.

        The 0.35 s reaction bound matches the independent motor watchdog. A
        provisional 0.5 m/s² minimum deceleration is deliberately below the
        measured simulated braking performance; it is not a hardware rating.
        During braking consider both retained yaw rate and retained curvature,
        plus the previous commanded turn and straight travel. A final 6 cm
        tangent extension is a distance buffer, not more time spent turning.
        """
        speed = max(0., float(speed))
        if speed <= 1e-9:
            return [np.zeros((1, 2))]
        reaction = .35 + max(0., float(observation_age))
        deceleration = .5
        duration = reaction + speed / deceleration
        count = max(1, math.ceil(duration / .02))
        dt = duration / count
        paths = []
        for turn in np.unique([0., float(angular), float(previous_angular)]):
            for keep_curvature in (False, True):
                points = [np.zeros(2)]
                heading = 0.
                for index in range(count):
                    t = (index + .5) * dt
                    velocity = speed if t <= reaction else max(0., speed - deceleration * (t - reaction))
                    rate = turn * (velocity / speed if keep_curvature else 1.)
                    midpoint_heading = heading + .5 * rate * dt
                    points.append(points[-1] + velocity * dt * np.array(
                        [math.cos(midpoint_heading), math.sin(midpoint_heading)]))
                    heading += rate * dt
                tangent = np.array([math.cos(heading), math.sin(heading)])
                end = points[-1].copy()
                points.extend(end + distance * tangent for distance in (.015, .030, .045, .060))
                paths.append(np.asarray(points))
        return paths

    def _rgb_swept_stop_reason(self, observation, linear, angular, safe,
                               *, observation_age=0.):
        """Check positive hits, historical clearance and current floor evidence.

        A non-hit floor interval ends at uncertainty, which remains unknown in
        the map; it never becomes a fabricated obstacle. Historical verified
        free floor is usable only in the documented static-environment domain.
        The robot must fit inside the configured circular physical footprint.
        """
        pose = observation.pose
        previous_linear, previous_angular = self._last_command
        speed = max(0., float(linear), float(previous_linear))
        paths = self._rgb_braking_paths(speed, angular, previous_angular,
                                        observation_age=observation_age)
        centers = np.concatenate(paths)
        angles = np.asarray(observation.angles, dtype=float)
        ranges = np.asarray(observation.hit_ranges if observation.hit_ranges is not None
                            else observation.ranges, dtype=float)
        hits = (np.asarray(observation.hits, dtype=bool)
                & np.isfinite(angles) & np.isfinite(ranges) & (ranges > .02))
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        offset = np.zeros(2)
        if observation.sensor_origin is not None:
            origin = np.asarray(observation.sensor_origin, dtype=float)
            if origin.shape != (2,) or not np.all(np.isfinite(origin)):
                return "invalid_rgb_sensor_origin"
            dx, dy = origin - [pose.x, pose.y]
            offset = np.array([c * dx + s * dy, -s * dx + c * dy])
        if np.any(hits):
            # Use every positive bearing: a close side object can intersect
            # the footprint even when it lies outside the central camera cone.
            points = offset + ranges[hits, None] * np.column_stack(
                (np.cos(angles[hits]), np.sin(angles[hits])))
            # 5 mm covers integration/sample spacing at the capped 0.25 m/s.
            radius = self.robot_radius + .04 + .005
            distance_squared = np.sum((points[:, None, :] - centers[None, :, :]) ** 2, axis=2)
            if np.any(distance_squared <= radius ** 2):
                return "local_obstacle_stop"
        world_centers = centers @ np.array([[c, s], [-s, c]]) + [pose.x, pose.y]
        # Check the *whole* braking path, including beyond the selected path
        # waypoint. A safe waypoint alone cannot authorize coasting into unknown.
        rows = np.floor((world_centers[:, 1] - self.origin[1]) / self.resolution).astype(int)
        cols = np.floor((world_centers[:, 0] - self.origin[0]) / self.resolution).astype(int)
        inside = ((rows >= 0) & (rows < self.shape[0]) & (cols >= 0) & (cols < self.shape[1]))
        if not np.all(inside):
            return "braking_path_unknown"
        unsafe = ~safe[rows, cols]
        for point in world_centers[unsafe]:
            # Retain the existing narrowly bounded quantization recovery only
            # while the measured physical footprint itself remains known free.
            if not self._line_safe((pose.x, pose.y), tuple(point), safe):
                return "braking_path_unknown"
        if speed > 1e-9:
            if not self._rgb_forward_floor_witness(observation, paths, offset):
                return "insufficient_current_forward_floor"
            if self._rgb_witness_used_recent:
                self._rgb_temporal_corroboration_checks += 1
        return None

    def _rgb_forward_floor_witness(self, observation, paths, sensor_offset):
        """Corroborate each leading stop direction with this frame's floor.

        The historical grid still proves the entire physical stopping tube,
        including the camera's near blind strip. This additional image check
        prevents an old, erroneously free map from authorizing travel while
        the camera currently sees only an unknown wall in that direction.

        A probe starts at the physical footprint's leading modeled stop point,
        before the path's extra 6 cm distance buffer. It may confirm floor
        anywhere through the buffered leading cap plus one current grid-cell
        diagonal. The whole buffered tube must still be historically clear.
        This is an observation confirmation band, not extra travel or an
        extension of a free interval.
        Both adjacent sensor bearings must contain the actual probe radius;
        an isolated valid ray or an angular gap cannot supply that sector.
        """
        self._rgb_witness_used_recent = False
        used_recent = False
        sectors = self._rgb_floor_sectors(observation)
        if not sectors:
            return False
        self._prune_rgb_history(observation)
        for path in paths:
            tangent = path[-1] - path[-2]
            tangent = tangent / np.linalg.norm(tangent)
            physical_stop = path[-5]
            leading = physical_stop + self.robot_radius * tangent - sensor_offset
            band = float(np.linalg.norm(path[-1] - physical_stop)) + math.sqrt(2.) * self.resolution
            if any(self._rgb_probe_intervals(leading, tangent, band, sector) for sector in sectors):
                continue
            if not self._rgb_recent_floor_witness(observation, leading, tangent, band,
                                                  sectors, sensor_offset):
                return False
            used_recent = True
        self._rgb_witness_used_recent = used_recent
        return True

    def _rgb_floor_sectors(self, observation):
        """Intersect disjoint unions at adjacent original angular samples."""
        angles = np.asarray(observation.angles, dtype=float)
        all_segments = self._free_segments(observation)
        # Keep invalid intervals in the angular grid: removing them would
        # incorrectly interpolate across an unobserved bearing.
        order = np.argsort(angles[np.isfinite(angles)])
        finite = np.flatnonzero(np.isfinite(angles))[order]
        if len(finite) < 3:
            return []
        angles = angles[finite]
        intervals = [all_segments[index] for index in finite]
        spacing = np.diff(angles)
        positive_spacing = spacing[spacing > 1e-10]
        if not len(positive_spacing):
            return []
        normal_spacing = float(np.median(positive_spacing))
        sectors = []
        for index, gap in enumerate(spacing):
            if (not intervals[index] or not intervals[index + 1] or gap <= 1e-10
                    or gap > normal_spacing * (1. + 1e-6) or gap >= math.pi):
                continue
            left, right = intervals[index], intervals[index + 1]
            first, second = 0, 0
            # Intersect the two unions with a linear merge. Taking their
            # convex hull would fabricate free floor across texture gaps.
            while first < len(left) and second < len(right):
                low = max(left[first][0], right[second][0])
                high = min(left[first][1], right[second][1])
                if high >= low:
                    sectors.append((np.array([math.cos(angles[index]), math.sin(angles[index])]),
                                    np.array([math.cos(angles[index + 1]), math.sin(angles[index + 1])]),
                                    low, high))
                if left[first][1] <= right[second][1]:
                    first += 1
                else:
                    second += 1
        return sectors

    @staticmethod
    def _rgb_probe_intervals(start, direction, length, sector, *, beyond_near=False):
        """Exact parameter intervals where a segment meets one image sector.

        Normal mode requires its actual annulus. ``beyond_near`` tests only
        whether CURRENT independently seen floor extends beyond this same
        direction; it does not label the intervening radial prefix free.
        """
        lower, upper, near, far = sector
        first, last = 0., float(length)

        def cross(a, b):
            return float(a[0] * b[1] - a[1] * b[0])

        for value, derivative in ((cross(lower, start), cross(lower, direction)),
                                  (cross(start, upper), cross(direction, upper))):
            if abs(derivative) <= 1e-12:
                if value < -1e-12:
                    return ()
            elif derivative > 0.:
                first = max(first, -value / derivative)
            else:
                last = min(last, -value / derivative)
        squared_direction = float(np.dot(direction, direction))
        projection = float(np.dot(start, direction))
        squared_start = float(np.dot(start, start))
        discriminant = projection ** 2 - squared_direction * (squared_start - far ** 2)
        if first > last or discriminant < -1e-12:
            return ()
        root = math.sqrt(max(0., discriminant))
        first = max(first, (-projection - root) / squared_direction)
        last = min(last, (-projection + root) / squared_direction)
        if first > last:
            return ()
        if beyond_near or near <= 0.:
            return ((first, last),)
        discriminant = projection ** 2 - squared_direction * (squared_start - near ** 2)
        if discriminant <= 0.:
            return ((first, last),)
        root = math.sqrt(discriminant)
        inner_first = (-projection - root) / squared_direction
        inner_last = (-projection + root) / squared_direction
        result = []
        if first <= min(last, inner_first):
            result.append((first, min(last, inner_first)))
        if max(first, inner_last) <= last:
            result.append((max(first, inner_last), last))
        return tuple(result)

    def _prune_rgb_history(self, observation):
        if observation.hits is None:
            self._rgb_recent_observations.clear()
            return
        self._rgb_recent_observations = deque((past for past in self._rgb_recent_observations
            if 0. <= observation.timestamp - past.timestamp <= 5.
            and math.hypot(observation.pose.x - past.pose.x,
                           observation.pose.y - past.pose.y) <= .4), maxlen=64)

    def _remember_rgb_observation(self, observation):
        """Deep-copy accepted image evidence; never retain caller-owned arrays."""
        if not observation.tracking:
            self._rgb_recent_observations.clear()
            return
        self._prune_rgb_history(observation)
        if observation.hits is None or not any(self._free_segments(observation)):
            return

        def frozen(values, dtype):
            if values is None:
                return None
            copied = np.array(values, dtype=dtype, copy=True)
            copied.setflags(write=False)
            return copied

        copied = Observation(Pose(float(observation.pose.x), float(observation.pose.y), float(observation.pose.yaw)),
            frozen(observation.angles, float), frozen(observation.ranges, float),
            float(observation.max_range), float(observation.timestamp), tracking=True,
            sensor_origin=(None if observation.sensor_origin is None else
                           tuple(float(value) for value in observation.sensor_origin)),
            min_ranges=frozen(observation.min_ranges, float), hits=frozen(observation.hits, bool),
            hit_ranges=frozen(observation.hit_ranges, float),
            free_intervals=(None if observation.free_intervals is None else
                tuple(tuple((float(near), float(far)) for near, far in segments)
                      for segments in observation.free_intervals)))
        self._rgb_recent_observations.append(copied)

    def _rgb_recent_floor_witness(self, observation, leading, tangent, band,
                                  current_sectors, sensor_offset):
        """Require current far-floor direction AND recent proof of one world point."""
        current = [interval for sector in current_sectors if sector[3] > sector[2]
                   for interval in self._rgb_probe_intervals(leading, tangent, band, sector,
                                                             beyond_near=True)]
        if not current:
            return False
        yaw = observation.pose.yaw
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)],
                             [math.sin(yaw), math.cos(yaw)]])
        world_start = rotation @ (leading + sensor_offset) + [observation.pose.x, observation.pose.y]
        world_direction = rotation @ tangent
        for past in reversed(self._rgb_recent_observations):
            if not 0. < observation.timestamp - past.timestamp <= 5.:
                continue
            origin = past.sensor_origin or (past.pose.x, past.pose.y)
            c, s = math.cos(past.pose.yaw), math.sin(past.pose.yaw)
            inverse = np.array([[c, s], [-s, c]])
            past_start = inverse @ (world_start - origin)
            past_direction = inverse @ world_direction
            for sector in self._rgb_floor_sectors(past):
                for first, last in self._rgb_probe_intervals(past_start, past_direction, band, sector):
                    if any(max(first, a) <= min(last, b) for a, b in current):
                        return True
        return False

    def _reset_failures_after_progress(self, pose, now):
        """Renew consecutive recovery budget only after entering a new region."""
        if self._failure_anchor is None or self._failed_goals == 0:
            return False
        if math.dist((pose.x, pose.y), self._failure_anchor) <= .8:
            return False
        previous = self._failed_goals
        self._failed_goals = 0
        self._failure_anchor = None
        self.events.append({"type": "recovery_budget_renewed", "timestamp": now,
                            "previous_failures": previous, "x": pose.x, "y": pose.y})
        return True

    def _follow(self, observation, safe):
        now, pose = observation.timestamp, observation.pose
        if self.goal is None or not self.path:
            self.status = "planning"
            return (0., 0.)
        if math.hypot(pose.x - self.goal[0], pose.y - self.goal[1]) < .16:
            self._failed_goals = 0
            self._failure_anchor = None
            self._begin_survey("station", now)
            return (0., 0.)
        if self._last_progress_xy is None or math.dist((pose.x, pose.y), self._last_progress_xy) > .08:
            self._last_progress_time = now
            self._last_progress_xy = (pose.x, pose.y)
            self._blocked_replans = 0
        elif now - self._last_progress_time > 14.:
            self._fail_goal("no_motion_progress", now)
            return (0., 0.)
        nearest = min(range(min(len(self.path), 20)), key=lambda i: math.dist((pose.x, pose.y), self.path[i]))
        self.path = self.path[nearest:]
        target = self.path[0]
        for waypoint in self.path:
            if math.dist((pose.x, pose.y), waypoint) > .55:
                break
            if self._line_safe((pose.x, pose.y), waypoint, safe):
                target = waypoint
        if math.dist((pose.x, pose.y), target) < .10 and len(self.path) > 1:
            target = self.path[1]
        if not self._line_safe((pose.x, pose.y), target, safe):
            self._request_replan("path_obstructed", now)
            return (0., 0.)
        bearing = math.atan2(target[1] - pose.y, target[0] - pose.x)
        error = wrap_angle(bearing - pose.yaw)
        angular = float(np.clip(1.7 * error, -self.turn_limit, self.turn_limit))
        linear = .25 * max(0., math.cos(error))
        if math.dist((pose.x, pose.y), self.goal) < .45:
            linear = min(linear, .13)
        if abs(error) > .45:
            linear = 0.
            # A forward sensor can face a nearby wall while the planned path
            # turns away from it. A circular, verified-clear footprint may
            # rotate in place: a translational braking envelope must not lock
            # out the very turn needed to point the sensor along the route.
            if self._line_safe((pose.x, pose.y), (pose.x, pose.y), safe):
                if observation.hits is not None:
                    stop_reason = self._rgb_swept_stop_reason(observation, linear, angular, safe)
                    if stop_reason:
                        self._request_replan(stop_reason, now)
                        return (0., 0.)
                return (0., angular)
        # Braking envelope at low speed, independent of the global path.
        finite_free = (np.isfinite(observation.ranges) & (observation.ranges > 0.)
                       if observation.free_intervals is None else
                       np.array([bool(segments) for segments in self._free_segments(observation)]))
        central = (np.abs(observation.angles) < math.radians(28)) & finite_free
        if not np.any(central):
            linear = 0.
        if observation.hits is not None:
            stop_reason = self._rgb_swept_stop_reason(observation, linear, angular, safe)
            if stop_reason:
                # Clearance can admit a slower approach even when the nominal
                # speed's full reaction/braking sweep does not fit. Every
                # candidate is checked against the same map and positive hits;
                # this never overrides the stop envelope or previous speed.
                if linear > .035:
                    for speed in sorted({linear * .75, linear * .5, linear * .25, .035}, reverse=True):
                        if speed < linear and self._rgb_swept_stop_reason(observation, speed, angular, safe) is None:
                            self.reason = "clearance_limited_speed"
                            return (float(speed), angular)
                self._request_replan(stop_reason, now)
                return (0., 0.)
        elif np.any(central) and float(np.min(observation.ranges[central])) < self.robot_radius + .15:
            self._request_replan("local_obstacle_stop", now)
            return (0., 0.)
        return (float(linear), angular)

    def _request_replan(self, reason, now):
        self._blocked_replans += 1
        self.path = []
        if self._blocked_replans >= 4:
            self._blocked_replans = 0
            self._fail_goal(reason, now)
        else:
            self.status, self.reason = "planning", reason

    def _fail_goal(self, reason, now):
        self._reset_failures_after_progress(self.pose, now)
        if self.goal:
            self._blacklist.append(self.goal)
        if self._failure_anchor is None:
            self._failure_anchor = (self.pose.x, self.pose.y)
        self._failed_goals += 1
        self.path = []
        self.goal = None
        self.events.append({"type": "goal_failed", "reason": reason, "timestamp": now})
        if self._failed_goals >= 4:
            self.status, self.reason = "stalled", "recovery_budget_exhausted"
        else:
            self._begin_survey("recovery", now)

    def _pause(self, reason):
        self._rgb_recent_observations.clear()
        if self.status != "paused":
            self._resume_status = self.status
        self.status, self.reason = "paused", reason
        self._last_command = (0., 0.)
        return self._last_command

    def check(self, now: float):
        if not math.isfinite(now):
            return self._pause("invalid_controller_clock")
        if self.status in ("complete", "stalled", "blocked"):
            return (0., 0.)
        if self._last_timestamp is not None and now + 1e-6 < self._last_timestamp:
            return self._pause("future_observation")
        if self._last_timestamp is None or now - self._last_timestamp > .5:
            return self._pause("stale_observation")
        if self.status in ("paused", "complete", "stalled", "blocked"):
            return (0., 0.)
        return self._last_command

    def update(self, observation: Observation) -> tuple[float, float]:
        pose = observation.pose
        if not observation.tracking:
            self._invalid_observations += 1
            return self._pause("tracking_lost")
        if not all(math.isfinite(v) for v in (pose.x, pose.y, pose.yaw, observation.timestamp, observation.max_range)):
            return self._pause("invalid_pose_or_timestamp")
        if observation.sensor_origin is not None and (
            np.shape(observation.sensor_origin) != (2,) or
            not np.all(np.isfinite(observation.sensor_origin))
        ):
            return self._pause("invalid_sensor_origin")
        if observation.max_range <= self.inflation_radius or len(observation.angles) != len(observation.ranges) or len(observation.ranges) < 3:
            return self._pause("invalid_range_frame")
        if any(extra is not None and np.shape(extra) != np.shape(observation.ranges)
               for extra in (observation.min_ranges,observation.hits,observation.hit_ranges)):
            return self._pause("invalid_range_intervals")
        if not self._valid_free_intervals(observation):
            return self._pause("invalid_range_intervals")
        finite = np.isfinite(observation.ranges) & (observation.ranges > .02)
        # RGB localization and floor clearance are independent measurements.
        # A visually tracked wall-facing frame may contain no conservative
        # free intervals. Keep its unknown bearings unknown and permit a view
        # change; _follow still requires a known-free braking sweep to move.
        required = 0 if observation.hits is not None else max(3, int(.25 * len(observation.ranges)))
        if int(np.sum(finite)) < required:
            return self._pause("insufficient_visible_floor" if observation.hits is not None else "insufficient_depth")
        if self._last_timestamp is not None and observation.timestamp <= self._last_timestamp:
            return self._pause("nonmonotonic_timestamp")
        if self._last_timestamp is not None and observation.timestamp - self._last_timestamp > .5:
            # A caller may have skipped the independent watchdog. Do not carry
            # temporal image authority across an unobserved tracking interval.
            self._rgb_recent_observations.clear()
        if self._last_pose is not None and self._last_timestamp is not None:
            dt = observation.timestamp - self._last_timestamp
            if math.hypot(pose.x - self._last_pose.x, pose.y - self._last_pose.y) > max(.50, 1.0 * dt):
                return self._pause("pose_discontinuity")
            if abs(wrap_angle(pose.yaw - self._last_pose.yaw)) > max(.60, 2.0 * dt):
                return self._pause("yaw_discontinuity")
        if not self._inside(self.world_to_cell(pose.x, pose.y)):
            return self._pause("outside_map_bounds")
        if self.status == "paused":
            self.status = self._resume_status or "initial_survey"
            self._last_yaw = pose.yaw
            self._last_progress_time = observation.timestamp
        if self._last_pose:
            self._mission_distance += math.hypot(pose.x - self._last_pose.x, pose.y - self._last_pose.y)
        if self._last_yaw is not None and self.status in ("initial_survey", "capture_sweep"):
            self._sweep_progress += max(0., wrap_angle(pose.yaw - self._last_yaw))
        self.pose, self._last_pose, self._last_yaw = pose, pose, pose.yaw
        self._last_timestamp = observation.timestamp
        self._reset_failures_after_progress(pose, observation.timestamp)
        self.integrate(observation)
        if observation.hits is None:
            self._rgb_recent_observations.clear()
        if self.status in ("complete", "stalled", "blocked"):
            self._last_command = (0., 0.)
            return self._last_command
        if self.status in ("initial_survey", "capture_sweep"):
            self._last_command = self._survey(observation.timestamp)
        elif self.status == "planning":
            safe = self.traversable()
            start = self._start_cell(safe)
            if start is None:
                self.status, self.reason = "blocked", "insufficient_footprint_clearance"
                self._last_command = (0., 0.)
            else:
                goal_cell = self.select_frontier(safe)
                if goal_cell is None:
                    if not self._final_scan_done:
                        self._begin_survey("final", observation.timestamp)
                    elif self._failed_goals:
                        self.status, self.reason = "stalled", "unreachable_after_recovery"
                    else:
                        self.status, self.reason = "complete", "reachable_frontiers_exhausted"
                    self._last_command = (0., 0.)
                else:
                    cells = self.astar(start, goal_cell, safe)
                    if not cells:
                        self.goal = self.cell_to_world(goal_cell)
                        self._fail_goal("planner_failed", observation.timestamp)
                    else:
                        self.path = [self.cell_to_world(cell) for cell in cells]
                        self.goal = self.path[-1]
                        self.status, self.reason = "following", "frontier_goal"
                        if self._last_progress_xy is None or math.dist((pose.x, pose.y), self._last_progress_xy) > .08:
                            self._last_progress_xy = (pose.x, pose.y)
                            self._last_progress_time = observation.timestamp
                        self._final_scan_done = False
                        self.events.append({"type": "goal", "timestamp": observation.timestamp,
                                            "x": self.goal[0], "y": self.goal[1]})
                    self._last_command = (0., 0.)
        else:
            self._last_command = self._follow(observation, self.traversable())
        self._remember_rgb_observation(observation)
        self._command_count += 1
        return self._last_command

    def snapshot(self, include_grid: bool = False) -> dict[str, Any]:
        result = {"status": self.status, "reason": self.reason,
                "pose": None if self.pose is None else {"x": self.pose.x, "y": self.pose.y, "yaw": self.pose.yaw},
                "goal": self.goal, "path": list(self.path), "station_count": len(self.stations),
                "capture_count": self._captures, "frontier_clusters": self.frontier_count,
                "minimum_frontier_cluster_cells": self._minimum_frontier_cells,
                "undersized_frontier_cells": self._undersized_frontier_cells,
                "exhausted_frontier_candidates": self._exhausted_frontier_candidates,
                "known_cells": int(np.sum(self.grid != UNKNOWN)),
                "free_cells": int(np.sum(self.grid == FREE)),
                "occupied_cells": int(np.sum(self.grid == OCCUPIED)),
                "unknown_cells": int(np.sum(self.grid == UNKNOWN)),
                "distance_m": self._mission_distance, "failed_goals": self._failed_goals,
                "invalid_observations": self._invalid_observations,
                "last_command": list(self._last_command), "command_count": self._command_count,
                "temporal_rgb_corroboration_checks": self._rgb_temporal_corroboration_checks,
                "recent_rgb_observation_count": len(self._rgb_recent_observations),
                "sweep_degrees": math.degrees(self._sweep_progress),
                "remaining_unknown_area_m2": float(np.sum(self.grid == UNKNOWN) * self.resolution ** 2),
                "origin": list(self.origin), "resolution": self.resolution, "shape": list(self.shape),
                "events": list(self.events[-32:]), "stations": list(self.stations)}
        if include_grid:
            result["grid"] = self.grid.tolist()
        return result
