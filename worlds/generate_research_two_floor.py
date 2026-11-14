"""Build a self-contained Gazebo Classic two-storey indoor world for Go2.

The stairwell and atrium are real openings in the upper collision floor.
All structural parts and furnishings have box collisions and use no external
meshes, models, or plugins.
"""

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path


LEVEL = 2.4
WALL_HEIGHT = 2.35
STAIR_RISE = 0.06
STAIR_TREAD = 0.2
COLORS = {
    "floor": "0.71 0.77 0.78 1",
    "upper_floor": "0.75 0.79 0.77 1",
    "wall": "0.86 0.86 0.82 1",
    "partition": "0.66 0.73 0.77 1",
    "stair": "0.66 0.60 0.52 1",
    "rail": "0.25 0.33 0.37 1",
    "furniture": "0.55 0.42 0.32 1",
    "cabinet": "0.40 0.51 0.58 1",
    "column": "0.58 0.63 0.65 1",
}


def sub(parent: ET.Element, tag: str, value: str | None = None, **attrs: str) -> ET.Element:
    node = ET.SubElement(parent, tag, attrs)
    if value is not None:
        node.text = value
    return node


def fmt(values) -> str:
    return " ".join(f"{value:.4f}" for value in values)


def box(model: ET.Element, name: str, xyz: tuple[float, float, float],
        size: tuple[float, float, float], material: str, friction: float = 2.0) -> None:
    link = sub(model, "link", name=name)
    sub(link, "pose", fmt((*xyz, 0, 0, 0)))
    collision = sub(link, "collision", name=f"{name}_collision")
    geometry = sub(collision, "geometry")
    sub(sub(geometry, "box"), "size", fmt(size))
    surface = sub(collision, "surface")
    ode = sub(sub(surface, "friction"), "ode")
    sub(ode, "mu", str(friction))
    sub(ode, "mu2", str(friction))
    visual = sub(link, "visual", name=f"{name}_visual")
    geometry = sub(visual, "geometry")
    sub(sub(geometry, "box"), "size", fmt(size))
    appearance = sub(visual, "material")
    sub(appearance, "ambient", COLORS[material])
    sub(appearance, "diffuse", COLORS[material])


def slab(model: ET.Element, name: str, xmin: float, xmax: float,
         ymin: float, ymax: float, top: float, material: str) -> None:
    box(model, name, ((xmin + xmax) / 2, (ymin + ymax) / 2, top - 0.1),
        (xmax - xmin, ymax - ymin, 0.2), material)


def wall_x(model: ET.Element, name: str, xmin: float, xmax: float,
           y: float, base: float, material: str = "wall") -> None:
    box(model, name, ((xmin + xmax) / 2, y, base + WALL_HEIGHT / 2),
        (xmax - xmin, 0.13, WALL_HEIGHT), material)


def wall_y(model: ET.Element, name: str, x: float, ymin: float, ymax: float,
           base: float, material: str = "wall") -> None:
    box(model, name, (x, (ymin + ymax) / 2, base + WALL_HEIGHT / 2),
        (0.13, ymax - ymin, WALL_HEIGHT), material)


def room_fronts(model: ET.Element, floor: int, y: float) -> None:
    """Five 1.2 m doors open from each room row to the central corridor."""
    left = -10.0
    for number, center in enumerate((-8.0, -4.0, 0.0, 4.0, 8.0), 1):
        opening_left = center - 0.6
        if opening_left > left:
            wall_x(model, f"f{floor}_front_{'north' if y > 0 else 'south'}_{number}",
                   left, opening_left, y, floor * LEVEL, "partition")
        left = center + 0.6
    wall_x(model, f"f{floor}_front_{'north' if y > 0 else 'south'}_end",
           left, 10, y, floor * LEVEL, "partition")


def room_furniture(model: ET.Element, floor: int) -> None:
    base = floor * LEVEL
    for row, y in (("north", 5.6), ("south", -5.6)):
        for number, x in enumerate((-8.0, -4.0, 0.0, 4.0, 8.0), 1):
            box(model, f"f{floor}_{row}_table_{number}", (x - 0.45, y, base + 0.38),
                (1.15, 0.68, 0.76), "furniture")
            box(model, f"f{floor}_{row}_cabinet_{number}",
                (x + 1.18, y + (0.8 if y > 0 else -0.8), base + 0.72),
                (0.46, 0.82, 1.44), "cabinet")


def point_light(world: ET.Element, name: str, x: float, y: float, z: float) -> None:
    light = sub(world, "light", name=name, type="point")
    sub(light, "pose", fmt((x, y, z, 0, 0, 0)))
    sub(light, "diffuse", "0.95 0.93 0.86 1")
    sub(light, "specular", "0.1 0.1 0.1 1")
    attenuation = sub(light, "attenuation")
    sub(attenuation, "range", "18")
    sub(attenuation, "constant", "0.4")
    sub(attenuation, "linear", "0.05")
    sub(attenuation, "quadratic", "0.01")


def build() -> ET.ElementTree:
    root = ET.Element("sdf", version="1.6")
    # Gazebo Classic's GUI in this workspace subscribes under /gazebo/default.
    world = sub(root, "world", name="default")
    physics = sub(world, "physics", name="ode_physics", type="ode")
    sub(physics, "max_step_size", "0.001")
    sub(physics, "real_time_update_rate", "1000")
    sub(physics, "real_time_factor", "1")
    sub(world, "gravity", "0 0 -9.8")
    scene = sub(world, "scene")
    sub(scene, "ambient", "0.55 0.55 0.55 1")
    sub(scene, "background", "0.72 0.82 0.90 1")
    gui = sub(world, "gui", fullscreen="false")
    camera = sub(gui, "camera", name="user_camera")
    sub(camera, "pose", "25 -25 18 0 0.42 2.356")
    sub(camera, "view_controller", "orbit")
    sun = sub(world, "light", name="sun", type="directional")
    sub(sun, "pose", "0 0 15 0 0 0")
    sub(sun, "direction", "-0.4 0.2 -0.9")
    sub(sun, "diffuse", "0.85 0.85 0.85 1")
    sub(sun, "specular", "0.1 0.1 0.1 1")
    for floor in (0, 1):
        for index, (x, y) in enumerate(((-6, 0), (3, 0), (-3, 5), (5, -5)), 1):
            point_light(world, f"f{floor}_light_{index}", x, y,
                        floor * LEVEL + 2.1)

    model = sub(world, "model", name="two_floor_research_building")
    sub(model, "static", "true")
    sub(model, "self_collide", "false")
    # Ground slab top = 0. Upper floor is tiled around two real openings.
    slab(model, "ground_floor", -10, 10, -7, 7, 0, "floor")
    slab(model, "upper_west", -10, -2.5, -7, 7, LEVEL, "upper_floor")
    slab(model, "upper_east_of_atrium", 2, 5, -7, 7, LEVEL, "upper_floor")
    slab(model, "upper_atrium_north", -2.5, 2, 2, 7, LEVEL, "upper_floor")
    slab(model, "upper_atrium_south", -2.5, 2, -7, -2, LEVEL, "upper_floor")
    slab(model, "upper_east_north", 5, 10, 3, 7, LEVEL, "upper_floor")
    slab(model, "upper_east_south", 5, 10, -7, -3, LEVEL, "upper_floor")

    for floor in (0, 1):
        base = floor * LEVEL
        # Main west entrance is open on the ground floor.
        if floor == 0:
            wall_y(model, "f0_west_north", -10, 1.0, 7, base)
            wall_y(model, "f0_west_south", -10, -7, -1.0, base)
        else:
            wall_y(model, "f1_west", -10, -7, 7, base)
        wall_y(model, f"f{floor}_east", 10, -7, 7, base)
        wall_x(model, f"f{floor}_north", -10, 10, 7, base)
        wall_x(model, f"f{floor}_south", -10, 10, -7, base)
        room_fronts(model, floor, 4.5)
        room_fronts(model, floor, -4.5)
        for x in (-6, -2, 2, 6):
            wall_y(model, f"f{floor}_north_divider_{x}", x, 4.5, 7, base, "partition")
            wall_y(model, f"f{floor}_south_divider_{x}", x, -7, -4.5, base, "partition")
        room_furniture(model, floor)

    # Ground-floor atrium landmarks; keep x=y=0 clear for Go2 spawn.
    for ix, x in enumerate((-2.1, 1.6)):
        for iy, y in enumerate((-1.5, 1.5)):
            box(model, f"atrium_column_{ix}_{iy}", (x, y, 1.18),
                (0.26, 0.26, 2.36), "column")
    box(model, "reception_desk", (0.0, 1.2, 0.48),
        (1.3, 0.55, 0.96), "furniture")
    # Guard the upper-floor atrium opening.
    for name, xyz, size in (
        ("atrium_west_rail", (-2.5, 0, 2.85), (0.08, 4.0, 0.9)),
        ("atrium_east_rail", (2.0, 0, 2.85), (0.08, 4.0, 0.9)),
        ("atrium_north_rail", (-0.25, 2, 2.85), (4.5, 0.08, 0.9)),
        ("atrium_south_rail", (-0.25, -2, 2.85), (4.5, 0.08, 0.9)),
        ("stair_north_rail", (7.5, 3, 2.85), (5, 0.08, 0.9)),
        ("stair_south_rail", (7.5, -3, 2.85), (5, 0.08, 0.9)),
        ("stair_outer_east_rail", (9.95, 0, 2.85), (0.08, 6, 0.9)),
    ):
        box(model, name, xyz, size, "rail")

    # Two opposed flights, 20 robot-scale collision steps each, with a 1 m turnaround.
    # Ground entry: x=5, y=-1.4. Upper exit: x=5, y=+1.4.
    for i in range(20):
        x_first = 5.0 + (i + 0.5) * STAIR_TREAD
        top_first = (i + 1) * STAIR_RISE
        box(model, f"first_flight_{i + 1:02d}",
            (x_first, -1.4, top_first / 2),
            (STAIR_TREAD, 1.6, top_first), "stair", 3.0)
        x_second = 9.0 - (i + 0.5) * STAIR_TREAD
        top_second = 1.2 + (i + 1) * STAIR_RISE
        box(model, f"second_flight_{i + 1:02d}",
            (x_second, 1.4, top_second / 2),
            (STAIR_TREAD, 1.6, top_second), "stair", 3.0)
    # Static underside supports at z<=1.2; landing surface is z=1.2.
    box(model, "stair_turnaround", (9.5, 0, 0.6), (1.0, 4.4, 1.2),
        "stair", 3.0)
    box(model, "stair_lane_divider", (7.0, 0, 1.52),
        (4.0, 0.12, 3.04), "rail")

    ET.indent(root, space="  ")
    return ET.ElementTree(root)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    build().write(args.output, encoding="utf-8", xml_declaration=True)
    print(f"Wrote {args.output}; 2 floors, 20 rooms, 20 stairs, "
          f"{LEVEL:.1f} m floor separation")


if __name__ == "__main__":
    main()
