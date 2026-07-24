#!/usr/bin/env python3
"""Generate Gazebo lab4.world from Unity StreamingAssets CSV (MIT lab4.zip).

Unity uses left-handed Y-up coordinates; ROS/Gazebo uses right-handed Z-up.
The conversion matches traj_vertices_publisher.cpp.
"""

from __future__ import annotations

import argparse
import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class ObjectSpec:
    size: tuple[float, float, float]
    collision: bool = True
    visual: bool = True
    color: tuple[float, float, float, float] = (0.72, 0.72, 0.76, 1.0)
    z_center_offset: float = 0.0


# Approximate Unity prefab dimensions on the 4.5 m campus grid.
SPECS: dict[str, ObjectSpec] = {
    "Wall_Simple_01": ObjectSpec((4.5, 0.15, 3.0), z_center_offset=1.5),
    "Wall_Arc_90_01": ObjectSpec((2.25, 0.15, 3.0), z_center_offset=1.5),
    "Wall_Arc_90_02": ObjectSpec((2.25, 0.15, 3.0), z_center_offset=1.5),
    "Wall_HalfArc_90_02": ObjectSpec((1.2, 0.15, 3.0), z_center_offset=1.5),
    "Wall_HalfArc_90_03": ObjectSpec((1.2, 0.15, 3.0), z_center_offset=1.5),
    "Column_01_Top": ObjectSpec((0.45, 0.45, 3.0), z_center_offset=1.5, color=(0.65, 0.63, 0.60, 1.0)),
    "Collider_01": ObjectSpec((4.5, 4.5, 3.0), visual=False, z_center_offset=1.5),
    "Collider_02": ObjectSpec((4.5, 4.5, 3.0), visual=False, z_center_offset=1.5),
    "Collider_03": ObjectSpec((4.5, 4.5, 3.0), visual=False, z_center_offset=1.5),
    "Collider_04": ObjectSpec((4.5, 4.5, 3.0), visual=False, z_center_offset=1.5),
    "Collider_05": ObjectSpec((4.5, 4.5, 3.0), visual=False, z_center_offset=1.5),
    "Roof_01": ObjectSpec((4.5, 4.5, 0.15), color=(0.55, 0.52, 0.50, 0.85), z_center_offset=0.0),
    "Roof_02": ObjectSpec((4.5, 4.5, 0.15), color=(0.55, 0.52, 0.50, 0.85), z_center_offset=0.0),
    "Roof_Angle_01": ObjectSpec((4.5, 4.5, 0.15), color=(0.55, 0.52, 0.50, 0.85)),
    "Roof_Angle_Large_01": ObjectSpec((9.0, 4.5, 0.15), color=(0.55, 0.52, 0.50, 0.85)),
    "Roof_Angle_Smal_01": ObjectSpec((2.25, 4.5, 0.15), color=(0.55, 0.52, 0.50, 0.85)),
    "Roof_Door_01": ObjectSpec((4.5, 4.5, 0.15), color=(0.55, 0.52, 0.50, 0.85)),
    "Door_Arch_01": ObjectSpec((2.0, 0.12, 2.8), color=(0.78, 0.74, 0.70, 1.0), z_center_offset=1.4),
    "Door_Left_01": ObjectSpec((0.12, 1.0, 2.8), color=(0.78, 0.74, 0.70, 1.0), z_center_offset=1.4),
}

GATE_SPEC = ObjectSpec((0.35, 0.05, 0.8), collision=False, color=(0.95, 0.25, 0.15, 0.9), z_center_offset=0.0)


def quat_to_rpy(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float]:
    sinr = 2.0 * (qw * qx + qy * qz)
    cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr, cosr)

    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))

    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def unity_to_ros(x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float):
    rx, ry, rz = x, z, y
    rqx, rqy, rqz, rqw = -qx, -qz, -qy, qw
    roll, pitch, yaw = quat_to_rpy(rqx, rqy, rqz, rqw)
    return rx, ry, rz, roll, pitch, yaw


def read_csv(csv_path: Path) -> list[tuple[str, float, float, float, float, float, float, float]]:
    rows: list[tuple[str, float, float, float, float, float, float, float]] = []
    with csv_path.open(encoding="utf-8") as handle:
        for line in handle:
            parts = line.strip().split(",")
            if len(parts) < 8:
                continue
            name = parts[0]
            values = tuple(float(v) for v in parts[1:8])
            rows.append((name, *values))
    return rows


def match_spec(name: str) -> ObjectSpec | None:
    if name in SPECS:
        return SPECS[name]
    for key, spec in SPECS.items():
        if name.startswith(key):
            return spec
    return None


def gate_index(name: str) -> int | None:
    if not name.startswith("red_square_drone_door"):
        return None
    match = re.search(r"[\[\(](\d+)[\]\)]", name)
    return int(match.group(1)) if match else 0


def dedupe_key(name: str, rx: float, ry: float, rz: float, yaw: float) -> tuple:
    spec = match_spec(name)
    if spec is None:
        return ("other", name, round(rx, 2), round(ry, 2), round(rz, 2), round(yaw, 1))
    return (name.split("_")[0] + "_" + name.split("_")[1] if name.startswith("Wall") else name,
            round(rx, 2), round(ry, 2), round(rz, 2), round(yaw, 1))


def append_box_model(parent: ET.Element, model_name: str, x: float, y: float, z: float,
                     roll: float, pitch: float, yaw: float, spec: ObjectSpec) -> None:
    sx, sy, sz = spec.size
    z_pose = z + spec.z_center_offset

    model = ET.SubElement(parent, "model", {"name": model_name})
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = f"{x} {y} {z_pose} {roll} {pitch} {yaw}"

    link = ET.SubElement(model, "link", {"name": "link"})
    if spec.collision:
        collision = ET.SubElement(link, "collision", {"name": "collision"})
        ET.SubElement(collision, "pose").text = "0 0 0 0 0 0"
        c_geom = ET.SubElement(collision, "geometry")
        ET.SubElement(c_geom, "box").append(ET.Element("size"))
        c_geom.find("box/size").text = f"{sx} {sy} {sz}"

    if spec.visual:
        visual = ET.SubElement(link, "visual", {"name": "visual"})
        ET.SubElement(visual, "pose").text = "0 0 0 0 0 0"
        v_geom = ET.SubElement(visual, "geometry")
        ET.SubElement(v_geom, "box").append(ET.Element("size"))
        v_geom.find("box/size").text = f"{sx} {sy} {sz}"
        material = ET.SubElement(visual, "material")
        ambient = ET.SubElement(material, "ambient")
        ambient.text = f"{spec.color[0]} {spec.color[1]} {spec.color[2]} {spec.color[3]}"
        diffuse = ET.SubElement(material, "diffuse")
        diffuse.text = ambient.text


def build_world(csv_path: Path) -> ET.ElementTree:
    rows = read_csv(csv_path)
    root = ET.Element("sdf", {"version": "1.6"})
    world = ET.SubElement(root, "world", {"name": "lab4_world"})

    physics = ET.SubElement(world, "physics", {"type": "ode"})
    ET.SubElement(physics, "max_step_size").text = "0.001"
    ET.SubElement(physics, "real_time_factor").text = "1.0"
    ET.SubElement(physics, "real_time_update_rate").text = "1000"
    ET.SubElement(world, "gravity").text = "0 0 -9.81"

    ET.SubElement(world, "include").append(ET.fromstring("<uri>model://ground_plane</uri>"))
    ET.SubElement(world, "include").append(ET.fromstring("<uri>model://sun</uri>"))

    seen: set[tuple] = set()
    model_count = 0

    for name, x, y, z, qx, qy, qz, qw in rows:
        spec = match_spec(name)
        if spec is None:
            continue
        rx, ry, rz, roll, pitch, yaw = unity_to_ros(x, y, z, qx, qy, qz, qw)
        key = dedupe_key(name, rx, ry, rz, yaw)
        if key in seen:
            continue
        seen.add(key)
        append_box_model(world, f"lab4_{model_count:04d}_{name}", rx, ry, rz, roll, pitch, yaw, spec)
        model_count += 1

    gates: list[tuple[int, float, float, float, float, float, float]] = []
    for name, x, y, z, qx, qy, qz, qw in rows:
        idx = gate_index(name)
        if idx is None:
            continue
        rx, ry, rz, roll, pitch, yaw = unity_to_ros(x, y, z, qx, qy, qz, qw)
        gates.append((idx, rx, ry, rz, roll, pitch, yaw))

    for idx, rx, ry, rz, roll, pitch, yaw in sorted(gates):
        append_box_model(
            world,
            f"lab4_gate_{idx:02d}",
            rx,
            ry,
            rz,
            roll,
            pitch,
            yaw,
            GATE_SPEC,
        )
        model_count += 1

    spawn = ET.SubElement(world, "model", {"name": "spawn_marker"})
    ET.SubElement(spawn, "static").text = "true"
    ET.SubElement(spawn, "pose").text = "0.4 8.0 0.05 0 0 0"
    spawn_link = ET.SubElement(spawn, "link", {"name": "link"})
    spawn_visual = ET.SubElement(spawn_link, "visual", {"name": "visual"})
    spawn_geom = ET.SubElement(spawn_visual, "geometry")
    ET.SubElement(spawn_geom, "cylinder").append(ET.Element("radius"))
    cyl = spawn_geom.find("cylinder")
    ET.SubElement(cyl, "length").text = "0.1"
    cyl.find("radius").text = "0.25"
    spawn_mat = ET.SubElement(spawn_visual, "material")
    ET.SubElement(spawn_mat, "ambient").text = "0.2 0.85 0.35 1"
    ET.SubElement(spawn_mat, "diffuse").text = "0.2 0.85 0.35 1"

    print(f"[generate_lab4_world] csv={csv_path}")
    print(f"[generate_lab4_world] placed {model_count} models ({len(gates)} gates)")
    return ET.ElementTree(root)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    default_csv = Path(__file__).resolve().parents[1] / "data/StreamingAssets/vnav-2020-lab4_static_tfs.csv"
    default_out = (
        Path(__file__).resolve().parents[2]
        / "lab3/gazebo_quadrotor_pkg/worlds/lab4.world"
    )
    parser.add_argument("--csv", type=Path, default=default_csv)
    parser.add_argument("--output", type=Path, default=default_out)
    args = parser.parse_args()

    tree = build_world(args.csv)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(args.output, encoding="unicode", xml_declaration=True)
    print(f"[generate_lab4_world] wrote {args.output}")


if __name__ == "__main__":
    main()
