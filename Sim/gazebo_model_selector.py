#!/usr/bin/env python3

from __future__ import annotations

import argparse
import re
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple


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


def discover_models() -> List[RobotModel]:
    models: List[RobotModel] = []
    for package_xml in sorted(DESCRIPTION_DIR.rglob("package.xml")):
        package_dir = package_xml.parent
        urdf_path = package_dir / "urdf" / "robot.urdf"
        if not urdf_path.exists():
            continue

        try:
            package_root = ET.parse(package_xml).getroot()
            package_name = package_root.findtext("name")
        except ET.ParseError:
            continue

        if package_name:
            models.append(RobotModel(package_name.strip(), package_dir, urdf_path))

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


def parse_float(value: str, default: float = 0.0) -> float:
    return float(value) if value else default


def update_mesh_paths(root: ET.Element, package_dir: Path) -> None:
    mesh_root = package_dir / "meshes"
    for mesh in root.findall(".//mesh"):
        filename = mesh.attrib.get("filename")
        if not filename:
            continue
        mesh_name = Path(filename).name
        local_mesh = mesh_root / mesh_name
        if local_mesh.exists():
            mesh.attrib["filename"] = f"file://{local_mesh}"


def prepare_urdf(model: RobotModel) -> Tuple[Path, str]:
    tree = ET.parse(model.urdf_path)
    root = tree.getroot()
    update_mesh_paths(root, model.package_dir)

    tmp = tempfile.NamedTemporaryFile("w", suffix=f"_{model.package_name}.urdf", delete=False)
    try:
        tmp.write(ET.tostring(root, encoding="unicode"))
        tmp.flush()
        return Path(tmp.name), tmp.name
    finally:
        tmp.close()


def convert_urdf_to_sdf(urdf_path: Path) -> str:
    result = subprocess.run(
        ["gz", "sdf", "-p", str(urdf_path)],
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout


def sanitize_sdf_text(sdf_text: str) -> str:
    # Drop plugin blocks first: many ROS/Gazebo plugins are irrelevant for pure spawn
    # and may contain extension tags that break strict XML parsing.
    cleaned = re.sub(r"<plugin\b[^>]*/>", "", sdf_text)
    cleaned = re.sub(r"<plugin\b[^>]*>[\s\S]*?</plugin>", "", cleaned)

    # Fallback: map extension tags like <gz::foo> into XML-safe names.
    def _fix_tag(match: re.Match[str]) -> str:
        slash, tag_name, rest = match.group(1), match.group(2), match.group(3)
        safe_name = tag_name.replace("::", "__")
        return f"<{slash}{safe_name}{rest}>"

    cleaned = re.sub(r"<(/?)([A-Za-z_][A-Za-z0-9_:\-.]*::[A-Za-z0-9_:\-.]*)([^>]*)>", _fix_tag, cleaned)
    return cleaned


def build_world_sdf(model: RobotModel, spawn_xyz: Tuple[float, float, float], spawn_yaw: float) -> str:
    urdf_temp, urdf_temp_name = prepare_urdf(model)
    try:
        sdf_text = convert_urdf_to_sdf(urdf_temp)
    finally:
        urdf_temp.unlink(missing_ok=True)

    sdf_root = ET.fromstring(sanitize_sdf_text(sdf_text))
    model_elem = sdf_root.find("model")
    if model_elem is None:
        raise RuntimeError("failed to convert URDF to SDF model")

    model_elem.attrib["name"] = model.package_name
    pose = model_elem.find("pose")
    pose_text = f"{spawn_xyz[0]:.8g} {spawn_xyz[1]:.8g} {spawn_xyz[2]:.8g} 0 0 {spawn_yaw:.8g}"
    if pose is None:
        pose = ET.SubElement(model_elem, "pose")
    pose.text = pose_text

    world_root = ET.Element("sdf", version=sdf_root.attrib.get("version", "1.11"))
    world = ET.SubElement(world_root, "world", name="default")
    ET.SubElement(world, "gravity").text = "0 0 -9.81"
    ET.SubElement(world, "magnetic_field").text = "6e-06 2.3e-05 -4.2e-05"
    ET.SubElement(world, "atmosphere", type="adiabatic")
    light = ET.SubElement(world, "light", name="sun", type="directional")
    ET.SubElement(light, "pose").text = "0 0 10 0 0 0"
    ET.SubElement(light, "direction").text = "-0.5 0.1 -1"
    ET.SubElement(light, "diffuse").text = "0.8 0.8 0.8 1"
    ET.SubElement(light, "specular").text = "0.2 0.2 0.2 1"

    ground = ET.SubElement(world, "model", name="ground_plane")
    ET.SubElement(ground, "static").text = "true"
    ground_link = ET.SubElement(ground, "link", name="ground_link")
    ground_collision = ET.SubElement(ground_link, "collision", name="collision")
    ET.SubElement(ground_collision, "pose").text = "0 0 0 0 0 0"
    ground_geom = ET.SubElement(ground_collision, "geometry")
    plane = ET.SubElement(ground_geom, "plane")
    ET.SubElement(plane, "normal").text = "0 0 1"
    ET.SubElement(plane, "size").text = "100 100"
    ground_visual = ET.SubElement(ground_link, "visual", name="visual")
    ET.SubElement(ground_visual, "pose").text = "0 0 0 0 0 0"
    ground_visual_geom = ET.SubElement(ground_visual, "geometry")
    visual_plane = ET.SubElement(ground_visual_geom, "plane")
    ET.SubElement(visual_plane, "normal").text = "0 0 1"
    ET.SubElement(visual_plane, "size").text = "100 100"
    ET.SubElement(ground_visual, "material")
    world.append(model_elem)

    return ET.tostring(world_root, encoding="unicode")


def write_temp_world(world_text: str, package_name: str) -> Path:
    tmp = tempfile.NamedTemporaryFile("w", suffix=f"_{package_name}.sdf", delete=False)
    try:
        tmp.write(world_text)
        tmp.flush()
        return Path(tmp.name)
    finally:
        tmp.close()


def validate_world(world_path: Path) -> None:
    subprocess.run(["gz", "sdf", "-k", str(world_path)], check=True)


def launch_gz(world_path: Path) -> int:
    process = subprocess.Popen(["gz", "sim", "-r", str(world_path)])
    try:
        return process.wait()
    except KeyboardInterrupt:
        process.terminate()
        try:
            return process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            return process.wait()


def main() -> int:
    parser = argparse.ArgumentParser(description="Interactive Gazebo Sim selector for robot descriptions")
    parser.add_argument("--model", help="package name to load directly, e.g. go1_description")
    parser.add_argument("--x", type=float, default=0.0, help="spawn x position")
    parser.add_argument("--y", type=float, default=0.0, help="spawn y position")
    parser.add_argument("--z", type=float, default=0.42, help="spawn z position")
    parser.add_argument("--yaw", type=float, default=0.0, help="spawn yaw in radians")
    parser.add_argument("--list", action="store_true", help="list available models and exit")
    parser.add_argument("--check-only", action="store_true", help="build and validate the world, then exit")
    args = parser.parse_args()

    models = discover_models()
    if args.list:
        for model in models:
            print(model.display_name)
        return 0

    selected = choose_model(models, args.model)
    print(f"Selected: {selected.display_name}")
    print(f"URDF: {selected.urdf_path}")

    world_text = build_world_sdf(selected, (args.x, args.y, args.z), args.yaw)
    world_path = write_temp_world(world_text, selected.package_name)
    print(f"World: {world_path}")

    try:
        validate_world(world_path)
        print("Validated Gazebo world successfully")
        if args.check_only:
            return 0
        return launch_gz(world_path)
    finally:
        world_path.unlink(missing_ok=True)


if __name__ == "__main__":
    raise SystemExit(main())