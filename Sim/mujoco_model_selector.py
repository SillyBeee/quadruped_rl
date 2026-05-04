#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import sys
import time
import tempfile
import warnings
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import mujoco
import mujoco.viewer
import numpy as np
import trimesh


ROOT_DIR = Path(__file__).resolve().parents[1]
DESCRIPTION_DIR = ROOT_DIR / "descriptions"


@dataclass(frozen=True)
class RobotModel:
    package_name: str
    package_dir: Path
    urdf_path: Path

    @property
    def display_name(self) -> str:
        relative_dir = self.package_dir.relative_to(ROOT_DIR)
        return f"{self.package_name} ({relative_dir})"


@dataclass(frozen=True)
class JointSpec:
    name: str
    joint_type: str
    parent: str
    child: str
    origin_xyz: Tuple[float, float, float]
    origin_quat: Tuple[float, float, float, float]
    axis: Tuple[float, float, float]
    lower: Optional[float]
    upper: Optional[float]
    damping: Optional[float]


@dataclass(frozen=True)
class InertialSpec:
    mass: float
    pos: Tuple[float, float, float]
    fullinertia: Tuple[float, float, float, float, float, float]


@dataclass(frozen=True)
class MeshAssetSpec:
    source_path: Path
    scale: Tuple[float, float, float]
    export_path: Path


def parse_float_list(value: Optional[str], expected: int) -> Tuple[float, ...]:
    if not value:
        return tuple(0.0 for _ in range(expected))
    parts = value.split()
    if len(parts) != expected:
        raise ValueError(f"expected {expected} values, got {value!r}")
    return tuple(float(part) for part in parts)


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5

    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (w, x, y, z)


def format_vec(values: Iterable[float]) -> str:
    return " ".join(f"{value:.8g}" for value in values)


def parse_origin(element: Optional[ET.Element]) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
    if element is None:
        return (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)
    xyz = parse_float_list(element.attrib.get("xyz"), 3)
    rpy = parse_float_list(element.attrib.get("rpy"), 3)
    quat = rpy_to_quat(*rpy)
    return xyz, quat


def parse_inertial(link: ET.Element) -> Optional[InertialSpec]:
    inertial = link.find("inertial")
    if inertial is None:
        return None

    origin_xyz, origin_quat = parse_origin(inertial.find("origin"))
    mass_elem = inertial.find("mass")
    inertia_elem = inertial.find("inertia")
    if mass_elem is None or inertia_elem is None:
        return None

    mass = max(float(mass_elem.attrib["value"]), 1e-6)
    fullinertia = (
        max(float(inertia_elem.attrib.get("ixx", "0")), 1e-8),
        max(float(inertia_elem.attrib.get("iyy", "0")), 1e-8),
        max(float(inertia_elem.attrib.get("izz", "0")), 1e-8),
        float(inertia_elem.attrib.get("ixy", "0")),
        float(inertia_elem.attrib.get("ixz", "0")),
        float(inertia_elem.attrib.get("iyz", "0")),
    )
    return InertialSpec(mass=mass, pos=origin_xyz, fullinertia=fullinertia)


def parse_materials(robot: ET.Element) -> Dict[str, Tuple[float, float, float, float]]:
    materials: Dict[str, Tuple[float, float, float, float]] = {}
    for material in robot.findall("material"):
        name = material.attrib.get("name")
        color_elem = material.find("color")
        if not name or color_elem is None:
            continue
        rgba = color_elem.attrib.get("rgba")
        if rgba is None:
            continue
        materials[name] = parse_float_list(rgba, 4)
    return materials


def resolve_mesh_source(filename: str, package_dir: Path) -> Path:
    raw_path = filename
    if raw_path.startswith("file://"):
        raw_path = raw_path[len("file://"):]
    if raw_path.startswith("package://"):
        raw_path = raw_path.split("/", 2)[-1]

    mesh_name = Path(raw_path).name
    direct_path = package_dir / "meshes" / mesh_name
    if direct_path.exists():
        return direct_path

    absolute_path = Path(raw_path)
    if absolute_path.exists():
        return absolute_path

    for candidate in (package_dir / "meshes").rglob(mesh_name):
        return candidate

    return direct_path


def export_mesh_as_obj(source_path: Path, scale: Tuple[float, float, float], export_dir: Path) -> Path:
    export_dir.mkdir(parents=True, exist_ok=True)
    mesh_or_scene = trimesh.load(source_path, force="mesh", process=False)
    if isinstance(mesh_or_scene, trimesh.Scene):
        geometries = [geometry for geometry in mesh_or_scene.dump() if geometry is not None]
        if not geometries:
            raise RuntimeError(f"failed to load mesh scene from {source_path}")
        mesh = trimesh.util.concatenate(geometries)
    else:
        mesh = mesh_or_scene

    mesh = mesh.copy()
    mesh.apply_scale(scale)

    scale_tag = "_".join(f"{value:.4g}" for value in scale)
    export_path = export_dir / f"{source_path.stem}_{scale_tag}.obj"
    mesh.export(export_path)
    return export_path


def visual_rgba(visual_elem: ET.Element, materials: Dict[str, Tuple[float, float, float, float]]) -> Tuple[float, float, float, float]:
    default_rgba = (0.8, 0.8, 0.8, 1.0)
    material_elem = visual_elem.find("material")
    if material_elem is None:
        return default_rgba

    rgba_elem = material_elem.find("color")
    if rgba_elem is not None:
        rgba = rgba_elem.attrib.get("rgba")
        if rgba is not None:
            return parse_float_list(rgba, 4)

    material_name = material_elem.attrib.get("name")
    if material_name and material_name in materials:
        return materials[material_name]

    return default_rgba


def discover_models() -> List[RobotModel]:
    models: List[RobotModel] = []
    for package_xml in sorted(DESCRIPTION_DIR.rglob("package.xml")):
        package_dir = package_xml.parent
        urdf_path = package_dir / "urdf" / "robot.urdf"
        if not urdf_path.exists():
            continue

        try:
            tree = ET.parse(package_xml)
            package_name = tree.getroot().findtext("name")
        except ET.ParseError:
            continue

        if not package_name:
            continue

        models.append(RobotModel(package_name=package_name.strip(), package_dir=package_dir, urdf_path=urdf_path))

    return models


def choose_model(models: List[RobotModel], package_name: Optional[str]) -> RobotModel:
    if not models:
        raise RuntimeError(f"no robot models found under {DESCRIPTION_DIR}")

    if package_name:
        for model in models:
            if model.package_name == package_name:
                return model
        raise ValueError(f"unknown package name: {package_name}")

    if len(models) == 1 or not sys.stdin.isatty():
        return models[0]

    print("可用模型:")
    for index, model in enumerate(models, start=1):
        print(f"  {index}. {model.display_name}")

    while True:
        raw = input("选择模型编号: ").strip()
        try:
            choice = int(raw)
        except ValueError:
            print("请输入有效的数字编号。")
            continue

        if 1 <= choice <= len(models):
            return models[choice - 1]

        print(f"请输入 1 到 {len(models)} 之间的编号。")


def parse_urdf(urdf_path: Path) -> Tuple[Dict[str, ET.Element], Dict[str, List[JointSpec]], Dict[str, Optional[InertialSpec]], str]:
    tree = ET.parse(urdf_path)
    robot = tree.getroot()
    robot_name = robot.attrib.get("name", urdf_path.parent.parent.name)

    links: Dict[str, ET.Element] = {}
    inertials: Dict[str, Optional[InertialSpec]] = {}
    child_map: Dict[str, List[JointSpec]] = {}
    child_links: set[str] = set()

    for link in robot.findall("link"):
        name = link.attrib["name"]
        links[name] = link
        inertials[name] = parse_inertial(link)

    for joint in robot.findall("joint"):
        joint_name = joint.attrib["name"]
        joint_type = joint.attrib.get("type", "fixed")
        parent_elem = joint.find("parent")
        child_elem = joint.find("child")
        if parent_elem is None or child_elem is None:
            continue

        origin_xyz, origin_quat = parse_origin(joint.find("origin"))
        axis_elem = joint.find("axis")
        axis = parse_float_list(axis_elem.attrib.get("xyz") if axis_elem is not None else "0 0 1", 3)

        lower = upper = damping = None
        limit_elem = joint.find("limit")
        if limit_elem is not None:
            if "lower" in limit_elem.attrib:
                lower = float(limit_elem.attrib["lower"])
            if "upper" in limit_elem.attrib:
                upper = float(limit_elem.attrib["upper"])

        dynamics_elem = joint.find("dynamics")
        if dynamics_elem is not None and "damping" in dynamics_elem.attrib:
            damping = float(dynamics_elem.attrib["damping"])

        spec = JointSpec(
            name=joint_name,
            joint_type=joint_type,
            parent=parent_elem.attrib["link"],
            child=child_elem.attrib["link"],
            origin_xyz=origin_xyz,
            origin_quat=origin_quat,
            axis=axis,
            lower=lower,
            upper=upper,
            damping=damping,
        )
        child_map.setdefault(spec.parent, []).append(spec)
        child_links.add(spec.child)

    root_links = [name for name in links if name not in child_links]
    if not root_links:
        raise RuntimeError(f"cannot find root link in {urdf_path}")

    return links, child_map, inertials, root_links[0]


def collision_geoms(link: ET.Element) -> List[Tuple[str, Dict[str, str]]]:
    geoms: List[Tuple[str, Dict[str, str]]] = []
    for collision in link.findall("collision"):
        origin_xyz, origin_quat = parse_origin(collision.find("origin"))
        geom_elem = collision.find("geometry")
        if geom_elem is None:
            continue

        geom: Optional[Tuple[str, Dict[str, str]]] = None
        if (box := geom_elem.find("box")) is not None:
            size = parse_float_list(box.attrib.get("size"), 3)
            geom = (
                "box",
                {
                    "size": format_vec(value * 0.5 for value in size),
                    "pos": format_vec(origin_xyz),
                    "quat": format_vec(origin_quat),
                    "rgba": "0.5 0.5 0.5 0.12",
                },
            )
        elif (cylinder := geom_elem.find("cylinder")) is not None:
            radius = float(cylinder.attrib["radius"])
            length = float(cylinder.attrib["length"])
            geom = (
                "cylinder",
                {
                    "size": format_vec((radius, length * 0.5)),
                    "pos": format_vec(origin_xyz),
                    "quat": format_vec(origin_quat),
                    "rgba": "0.5 0.5 0.5 0.12",
                },
            )
        elif (sphere := geom_elem.find("sphere")) is not None:
            radius = float(sphere.attrib["radius"])
            geom = (
                "sphere",
                {
                    "size": f"{radius:.8g}",
                    "pos": format_vec(origin_xyz),
                    "quat": format_vec(origin_quat),
                    "rgba": "0.5 0.5 0.5 0.12",
                },
            )
        elif geom_elem.find("capsule") is not None:
            capsule = geom_elem.find("capsule")
            assert capsule is not None
            radius = float(capsule.attrib["radius"])
            length = float(capsule.attrib["length"])
            geom = (
                "capsule",
                {
                    "size": format_vec((radius, length * 0.5)),
                    "pos": format_vec(origin_xyz),
                    "quat": format_vec(origin_quat),
                    "rgba": "0.5 0.5 0.5 0.12",
                },
            )
        else:
            warnings.warn("skip unsupported collision geometry")

        if geom is not None:
            geoms.append(geom)

    return geoms


def visual_geoms(
    link: ET.Element,
    package_dir: Path,
    materials: Dict[str, Tuple[float, float, float, float]],
    mesh_asset_root: ET.Element,
    mesh_assets: Dict[Tuple[str, Tuple[float, float, float]], str],
    export_dir: Path,
) -> List[Tuple[str, Dict[str, str]]]:
    geoms: List[Tuple[str, Dict[str, str]]] = []
    for visual in link.findall("visual"):
        origin_xyz, origin_quat = parse_origin(visual.find("origin"))
        rgba = visual_rgba(visual, materials)
        geom_elem = visual.find("geometry")
        if geom_elem is None:
            continue

        geom: Optional[Tuple[str, Dict[str, str]]] = None
        if (mesh_elem := geom_elem.find("mesh")) is not None:
            filename = mesh_elem.attrib.get("filename")
            if not filename:
                continue
            scale = parse_float_list(mesh_elem.attrib.get("scale", "1 1 1"), 3)
            source_path = resolve_mesh_source(filename, package_dir)
            asset_key = (str(source_path), scale)
            mesh_name = mesh_assets.get(asset_key)
            if mesh_name is None:
                mesh_name = f"mesh_{len(mesh_assets) + 1}"
                export_path = export_mesh_as_obj(source_path, scale, export_dir)
                ET.SubElement(mesh_asset_root, "mesh", name=mesh_name, file=str(export_path))
                mesh_assets[asset_key] = mesh_name

            geom = (
                "mesh",
                {
                    "mesh": mesh_name,
                    "pos": format_vec(origin_xyz),
                    "quat": format_vec(origin_quat),
                    "rgba": format_vec(rgba),
                    "contype": "0",
                    "conaffinity": "0",
                },
            )
        elif (box := geom_elem.find("box")) is not None:
            size = parse_float_list(box.attrib.get("size"), 3)
            geom = (
                "box",
                {
                    "size": format_vec(value * 0.5 for value in size),
                    "pos": format_vec(origin_xyz),
                    "quat": format_vec(origin_quat),
                    "rgba": format_vec(rgba),
                    "contype": "0",
                    "conaffinity": "0",
                },
            )
        elif (cylinder := geom_elem.find("cylinder")) is not None:
            radius = float(cylinder.attrib["radius"])
            length = float(cylinder.attrib["length"])
            geom = (
                "cylinder",
                {
                    "size": format_vec((radius, length * 0.5)),
                    "pos": format_vec(origin_xyz),
                    "quat": format_vec(origin_quat),
                    "rgba": format_vec(rgba),
                    "contype": "0",
                    "conaffinity": "0",
                },
            )
        elif (sphere := geom_elem.find("sphere")) is not None:
            radius = float(sphere.attrib["radius"])
            geom = (
                "sphere",
                {
                    "size": f"{radius:.8g}",
                    "pos": format_vec(origin_xyz),
                    "quat": format_vec(origin_quat),
                    "rgba": format_vec(rgba),
                    "contype": "0",
                    "conaffinity": "0",
                },
            )
        elif geom_elem.find("capsule") is not None:
            capsule = geom_elem.find("capsule")
            assert capsule is not None
            radius = float(capsule.attrib["radius"])
            length = float(capsule.attrib["length"])
            geom = (
                "capsule",
                {
                    "size": format_vec((radius, length * 0.5)),
                    "pos": format_vec(origin_xyz),
                    "quat": format_vec(origin_quat),
                    "rgba": format_vec(rgba),
                    "contype": "0",
                    "conaffinity": "0",
                },
            )
        else:
            warnings.warn("skip unsupported visual geometry")

        if geom is not None:
            geoms.append(geom)

    return geoms


def build_mjcf(
    model: RobotModel,
    spawn_xyz: Tuple[float, float, float],
    spawn_yaw: float,
    export_dir: Path,
) -> str:
    links, child_map, inertials, root_link_name = parse_urdf(model.urdf_path)
    tree = ET.parse(model.urdf_path)
    robot = tree.getroot()
    materials = parse_materials(robot)
    root_quat = rpy_to_quat(0.0, 0.0, spawn_yaw)

    mujoco_root = ET.Element("mujoco", model=model.package_name)
    ET.SubElement(mujoco_root, "compiler", angle="radian", coordinate="local", discardvisual="false")
    ET.SubElement(
        mujoco_root,
        "option",
        timestep="0.002",
        gravity="0 0 -9.81",
        iterations="50",
        ls_iterations="10",
        cone="elliptic",
    )

    default = ET.SubElement(mujoco_root, "default")
    ET.SubElement(
        default,
        "geom",
        friction="1.0 0.01 0.001",
        density="650",
        condim="3",
        rgba="0.75 0.75 0.78 1",
    )
    ET.SubElement(default, "joint", damping="1", armature="0.01")

    asset_root = ET.SubElement(mujoco_root, "asset")
    mesh_assets: Dict[Tuple[str, Tuple[float, float, float]], str] = {}

    worldbody = ET.SubElement(mujoco_root, "worldbody")
    ET.SubElement(
        worldbody,
        "light",
        pos="0 0 4",
        dir="0 0 -1",
        diffuse="0.8 0.8 0.8",
        ambient="0.25 0.25 0.25",
    )
    ET.SubElement(
        worldbody,
        "geom",
        name="floor",
        type="plane",
        size="0 0 0.1",
        pos="0 0 0",
        rgba="0.9 0.9 0.92 1",
    )

    root_body = ET.SubElement(
        worldbody,
        "body",
        name=model.package_name,
        pos=format_vec(spawn_xyz),
        quat=format_vec(root_quat),
    )
    ET.SubElement(root_body, "freejoint")

    def emit_link(body: ET.Element, link_name: str) -> None:
        link = links[link_name]
        inertial = inertials.get(link_name)
        if inertial is not None:
            ET.SubElement(
                body,
                "inertial",
                pos=format_vec(inertial.pos),
                mass=f"{inertial.mass:.8g}",
                fullinertia=format_vec(inertial.fullinertia),
            )

        for geom_type, geom_attrs in collision_geoms(link):
            ET.SubElement(body, "geom", type=geom_type, **geom_attrs)

        for geom_type, geom_attrs in visual_geoms(link, model.package_dir, materials, asset_root, mesh_assets, export_dir):
            ET.SubElement(body, "geom", type=geom_type, **geom_attrs)

        for joint in child_map.get(link_name, []):
            child_body = ET.SubElement(
                body,
                "body",
                name=joint.child,
                pos=format_vec(joint.origin_xyz),
                quat=format_vec(joint.origin_quat),
            )

            joint_attrs = {"name": joint.name}
            if joint.joint_type in {"revolute", "continuous"}:
                joint_attrs.update({"type": "hinge", "axis": format_vec(joint.axis)})
                if joint.joint_type == "revolute" and joint.lower is not None and joint.upper is not None:
                    joint_attrs["limited"] = "true"
                    joint_attrs["range"] = f"{joint.lower:.8g} {joint.upper:.8g}"
            elif joint.joint_type == "prismatic":
                joint_attrs.update({"type": "slide", "axis": format_vec(joint.axis)})
                if joint.lower is not None and joint.upper is not None:
                    joint_attrs["limited"] = "true"
                    joint_attrs["range"] = f"{joint.lower:.8g} {joint.upper:.8g}"
            elif joint.joint_type == "fixed":
                joint_attrs = {}
            else:
                warnings.warn(f"unsupported joint type {joint.joint_type!r} in {joint.name}, keeping as fixed body")
                joint_attrs = {}

            if joint_attrs:
                if joint.damping is not None:
                    joint_attrs["damping"] = f"{joint.damping:.8g}"
                ET.SubElement(child_body, "joint", **joint_attrs)

            emit_link(child_body, joint.child)

    emit_link(root_body, root_link_name)
    return ET.tostring(mujoco_root, encoding="unicode")


def run_viewer(model: mujoco.MjModel, data: mujoco.MjData, lookat: Tuple[float, float, float]) -> None:
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = np.array(lookat, dtype=float)
        viewer.cam.distance = 3.0
        viewer.cam.elevation = -20
        viewer.cam.azimuth = 140

        try:
            while viewer.is_running():
                step_start = time.perf_counter()
                mujoco.mj_step(model, data)
                viewer.sync()
                elapsed = time.perf_counter() - step_start
                sleep_time = model.opt.timestep - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
        except KeyboardInterrupt:
            return


def main() -> int:
    parser = argparse.ArgumentParser(description="Interactive MuJoCo selector for robot descriptions")
    parser.add_argument("--model", help="package name to load directly, e.g. go1_description")
    parser.add_argument("--x", type=float, default=0.0, help="spawn x position")
    parser.add_argument("--y", type=float, default=0.0, help="spawn y position")
    parser.add_argument("--z", type=float, default=0.42, help="spawn z position")
    parser.add_argument("--yaw", type=float, default=0.0, help="spawn yaw in radians")
    parser.add_argument("--list", action="store_true", help="list available models and exit")
    parser.add_argument("--compile-only", action="store_true", help="build the MuJoCo model and exit")
    args = parser.parse_args()

    models = discover_models()
    if args.list:
        for model in models:
            print(model.display_name)
        return 0

    selected = choose_model(models, args.model)
    print(f"Selected: {selected.display_name}")
    print(f"URDF: {selected.urdf_path}")

    with tempfile.TemporaryDirectory(prefix=f"mujoco_assets_{selected.package_name}_") as asset_dir_name:
        asset_dir = Path(asset_dir_name)
        xml_string = build_mjcf(selected, (args.x, args.y, args.z), args.yaw, asset_dir)
        mj_model = mujoco.MjModel.from_xml_string(xml_string)
        print(f"Compiled MuJoCo model: bodies={mj_model.nbody}, joints={mj_model.njnt}, geoms={mj_model.ngeom}")

        if args.compile_only:
            return 0

        mj_data = mujoco.MjData(mj_model)

        mj_data.qpos[0:3] = np.array([args.x, args.y, args.z], dtype=float)
        mj_data.qpos[3:7] = np.array(rpy_to_quat(0.0, 0.0, args.yaw), dtype=float)
        mujoco.mj_forward(mj_model, mj_data)

        run_viewer(mj_model, mj_data, (args.x, args.y, args.z + 0.1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())