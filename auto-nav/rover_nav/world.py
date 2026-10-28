"""Seeded three-room Isaac worlds, with evaluation metadata kept in ``source``.

The rooms form an L. Door positions are offset, so entering every room requires
turning around furnishings and then making a right-angle transition. All sizes
are SI full extents, as required by Isaac's bounded world descriptor contract.
The navigation system receives observations, never this geometry or room list.
"""
from __future__ import annotations

import math
import random
import xml.etree.ElementTree as ET
from copy import deepcopy

WALL_THICKNESS = 0.12
WALL_HEIGHT = 1.8


def _box(center, size, color):
    return {"center": [round(v, 5) for v in center],
            "size": [round(v, 5) for v in size], "color": list(color)}


def _point_segment_distance(point, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    ratio = 0 if length2 == 0 else max(0, min(1,
        ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / length2))
    return math.hypot(point[0] - a[0] - ratio * dx,
                      point[1] - a[1] - ratio * dy)


def generate_world(seed=7):
    """Return an importable ``isaac.world`` descriptor with deterministic clutter.

    A 1.16 m wide protected circulation route keeps all doors and rooms
    connected for a rover with a 0.27 m conservative swept radius. Furniture has
    genuine floor-level legs and raised tops; collision is handled by MuJoCo.
    Room labels, bounds and spawn are evaluator metadata, not planner inputs.
    """
    rng = random.Random(seed)
    boxes = []
    objects = []
    wall_color = [.66, .7, .74, 1]
    wood = [.46, .27, .15, 1]
    upholstery = [.13, .34, .43, 1]

    def add(name, center, size, color):
        objects.append({"name": name, "first_box": len(boxes), "box_count": 1})
        boxes.append(_box(center, size, color))

    def wall(name, a, b):
        add(name, [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, WALL_HEIGHT / 2],
            [abs(b[0] - a[0]) + WALL_THICKNESS,
             abs(b[1] - a[1]) + WALL_THICKNESS, WALL_HEIGHT], wall_color)

    # The six outer segments describe an L, not three collinear rooms.
    perimeter = [(0, 0), (10, 0), (10, 8), (5, 8), (5, 4), (0, 4), (0, 0)]
    for index, (a, b) in enumerate(zip(perimeter, perimeter[1:])):
        wall(f"outer_wall_{index}", a, b)
    # Door A-B is high in the east wall; B-C is far east in the north wall.
    wall("living_study_partition_south", (5, 0), (5, 2.4))
    wall("living_study_partition_north", (5, 3.6), (5, 4))
    wall("study_bedroom_partition_west", (5, 4), (7.7, 4))
    wall("study_bedroom_partition_east", (8.9, 4), (10, 4))

    def sofa(name, x, y, sx=2.0, sy=.7):
        first = len(boxes)
        add(name + " plinth", [x, y, .2], [sx, sy, .4], upholstery)
        add(name + " back", [x, y + sy * .39, .58], [sx, sy * .22, .76], upholstery)
        for sign in [-1, 1]:
            add(name + f" arm {sign}", [x + sign * (sx / 2 - .08), y, .48], [.16, sy, .56], upholstery)
        objects.append({"name": name, "first_box": first, "box_count": len(boxes) - first})

    def table(name, x, y, sx=1.3, sy=.85, height=.76):
        first = len(boxes)
        add(name + " top", [x, y, height - .035], [sx, sy, .07], wood)
        for dx in [-sx / 2 + .09, sx / 2 - .09]:
            for dy in [-sy / 2 + .09, sy / 2 - .09]:
                add(name + " leg", [x + dx, y + dy, (height - .07) / 2], [.09, .09, height - .07], wood)
        objects.append({"name": name, "first_box": first, "box_count": len(boxes) - first})

    def chair(name, x, y):
        first = len(boxes)
        add(name + " seat", [x, y, .43], [.5, .5, .08], [.61, .39, .18, 1])
        add(name + " back", [x, y + .22, .64], [.5, .06, .5], [.61, .39, .18, 1])
        for dx in [-.19, .19]:
            for dy in [-.19, .19]:
                add(name + " leg", [x + dx, y + dy, .2], [.065, .065, .4], wood)
        objects.append({"name": name, "first_box": first, "box_count": len(boxes) - first})

    sofa("living room sofa", 1.55, 3.55, 2.2, .65)
    table("living room coffee table", 2.9, 1.05, 1.3, .85, .6)
    chair("living room chair", 4.25, .6)
    add("living cabinet", [.4, .5, .48], [.58, .72, .96], wood)
    table("study desk", 6.35, .85, 1.5, .85)
    chair("study chair", 6.35, 1.8)
    add("study tall bookcase", [9.6, 1.55, .75], [.58, 2.15, 1.5], wood)
    sofa("bedroom settee", 6.35, 7.48, 1.9, .65)
    add("bed frame", [6.3, 5.8, .23], [1.4, 1.6, .46], [.4, .35, .48, 1])
    add("bed mattress", [6.3, 5.8, .5], [1.37, 1.57, .12], [.66, .62, .79, 1])
    add("bedroom wardrobe", [9.55, 6.75, .82], [.65, 1.55, 1.64], wood)

    rooms = [{"name": "living_room", "bounds": [0, 0, 5, 4]},
             {"name": "study", "bounds": [5, 0, 10, 4]},
             {"name": "bedroom", "bounds": [5, 4, 10, 8]}]
    circulation = [(1, 1), (1, 2.5), (4.2, 2.5), (4.35, 3),
                   (8.3, 3), (8.3, 5), (8.4, 6.5), (8.4, 7.1)]
    # Random boxes are rejected near the guaranteed circulation route and all
    # existing furniture. This changes the scene while preserving reachability.
    for room in rooms:
        xmin, ymin, xmax, ymax = room["bounds"]
        placed = 0
        for _ in range(4000):
            sx, sy, sz = rng.uniform(.28, .48), rng.uniform(.28, .48), rng.uniform(.2, .65)
            x, y = rng.uniform(xmin + .5, xmax - .5), rng.uniform(ymin + .5, ymax - .5)
            radius = math.hypot(sx, sy) / 2
            if min(_point_segment_distance((x, y), a, b)
                   for a, b in zip(circulation, circulation[1:])) < .58 + radius:
                continue
            if any(abs(x - box["center"][0]) < (sx + box["size"][0]) / 2 + .12
                   and abs(y - box["center"][1]) < (sy + box["size"][1]) / 2 + .12
                   for box in boxes):
                continue
            add(f"{room['name']} random box {placed}", [x, y, sz / 2], [sx, sy, sz],
                [rng.uniform(.4, .7), rng.uniform(.24, .5), rng.uniform(.12, .3), 1])
            placed += 1
            if placed == 2:
                break
        if placed < 2:
            raise RuntimeError(f"Could not place {room['name']} clutter without obstructing circulation")

    return {"kind": "isaac.world", "version": 1,
            "name": f"Mark Rover · three furnished rooms · seed {seed}",
            "units": "m", "upAxis": "Z", "ground_z": 0., "boxes": boxes,
            "source": {"generator": "mark-rover autonomous navigation", "seed": seed,
                       "rooms": rooms, "spawn": [1., 1., 0.],
                       "nav_bounds": [-.25, -.25, 10.25, 8.25],
                       "doors": [{"rooms": ["living_room", "study"], "center": [5., 3.], "width": 1.2},
                                 {"rooms": ["study", "bedroom"], "center": [8.3, 4.], "width": 1.2}],
                       "objects": objects,
                       "evaluation_only": "Room labels and collision geometry are withheld from the planner."}}


def scene_xml(world):
    """Render a static descriptor as self-contained XML accepted by Isaac."""
    root = ET.Element("mujoco", model="mark_rover_three_rooms")
    body = ET.SubElement(root, "worldbody")
    ET.SubElement(body, "light", name="nav_key_light", pos="5 4 7", dir="0 0 -1",
                  diffuse=".9 .9 .9")
    ET.SubElement(body, "light", name="nav_fill_light", pos="1 1 4", dir="1 1 -1",
                  diffuse=".45 .45 .45")
    if world.get("ground_z") is not None:
        ET.SubElement(body, "geom", name="nav_floor", type="plane",
                      pos=f"0 0 {world['ground_z']}", size="0 0 .1",
                      rgba=".22 .25 .29 1", friction="1 .01 .001")
    for index, box in enumerate(world["boxes"]):
        ET.SubElement(body, "geom", name=f"nav_world_{index}", type="box",
                      pos=" ".join(map(str, box["center"])),
                      size=" ".join(str(v / 2) for v in box["size"]),
                      rgba=" ".join(map(str, box["color"])), friction="1 .01 .001")
    return ET.tostring(root, encoding="unicode")


def combine_rover(robot_xml, world):
    """Add static world geometry while retaining every authored rover part.

    Spawn moves the free base and its nominal keyframes together. The rover's
    authored local forward is -Z, so yaw zero makes that direction world +X.
    Internal CAD transforms, masses, collision meshes and drives stay intact.
    """
    root = ET.fromstring(robot_xml)
    target = root.find("worldbody")
    if target is None:
        raise ValueError("The rover MJCF must have a worldbody")
    for geom in list(target.findall("geom")):
        if geom.get("type") == "plane":
            target.remove(geom)
    base = next((body for body in target.findall("body")
                 if body.find("freejoint") is not None), None)
    if base is None:
        raise ValueError("The rover must have a floating base")
    spawn_x, spawn_y, spawn_yaw = world["source"]["spawn"]
    position = [float(v) for v in base.get("pos", "0 0 0").split()]
    w, x, y, z = [float(v) for v in base.get("quat", "1 0 0 0").split()]
    forward_x, forward_y = -2 * (x * z + w * y), -2 * (y * z - w * x)
    delta = spawn_yaw - math.atan2(forward_y, forward_x)
    c, s = math.cos(delta / 2), math.sin(delta / 2)
    quaternion = [c * w - s * z, c * x - s * y,
                  c * y + s * x, c * z + s * w]
    pose = [spawn_x, spawn_y, position[2], *quaternion]
    base.set("pos", " ".join(map(str, pose[:3])))
    base.set("quat", " ".join(map(str, quaternion)))
    for key in root.findall("./keyframe/key"):
        values = key.get("qpos", "").split()
        if len(values) >= 7:
            key.set("qpos", " ".join([*map(str, pose), *values[7:]]))
    environment = ET.fromstring(scene_xml(world)).find("worldbody")
    for element in environment:
        target.append(deepcopy(element))
    return ET.tostring(root, encoding="unicode")
