# /// script
# requires-python = ">=3.10"
# dependencies = [
#     "robot-descriptions",
#     "viser",
#     "yourdfpy",
#     "trimesh",
#     "xacrodoc",
# ]
# ///
"""Browse a composed Cortado robot in viser."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import time
from types import SimpleNamespace
from urllib.parse import unquote, urlparse
import xml.etree.ElementTree as ET

import numpy as np
import viser
import viser.transforms
import yourdfpy
from viser.extras import ViserUrdf

SCRIPT_DIR = Path(__file__).resolve().parent.parent
ROBOT_XACRO_PATHS = {
    "cortado": SCRIPT_DIR / "robots" / "cortado.urdf.xacro",
    "fr3_robotiq_2f_85": SCRIPT_DIR / "robots" / "fr3_robotiq_2f_85.urdf.xacro",
}
ROBOT_XACRO_ARGS = {
    "cortado": {
        "robot_name": "cortado",
        "wrist_camera": "true",
        "flip_gripper": "true",
        "camera_mast": "2",
    },
    "fr3_robotiq_2f_85": {
        "robot_name": "fr3_robotiq",
        "wrist_camera": "true",
        "flip_gripper": "true",
    },
}
ROBOT_DESCRIPTIONS_CACHE_ROOT = Path(
    os.environ.get("ROBOT_DESCRIPTIONS_CACHE", "~/.cache/robot_descriptions")
).expanduser()

_ROBOTIQ_2F85_REPO_URL = "https://github.com/nickswalker/robotiq-2f-85.git"
_ROBOTIQ_2F85_COMMIT = "aa93c26c09bf5c78d1e6508bbce47657d809d16a"
_ROBOTIQ_2F85_PACKAGE = "robotiq_2f_85_gripper_visualization"
_ROBOTIQ_2F85_MODULE = "robotiq_2f85_v4_description"
_ZED_DESCRIPTION_REPO_URL = "https://github.com/stereolabs/zed-ros2-description.git"
_ZED_DESCRIPTION_COMMIT = "1aa88b319897311d2f33882a4abbc40da8d75ace"
_ZED_DESCRIPTION_PACKAGE = "zed_description"
_ZED_DESCRIPTION_MODULE = "zed_description"


def _get_manual_robotiq_2f85_description() -> SimpleNamespace:
    from robot_descriptions._cache import clone_to_directory

    repo_dir = ROBOT_DESCRIPTIONS_CACHE_ROOT / "manual" / _ROBOTIQ_2F85_MODULE
    clone_to_directory(
        _ROBOTIQ_2F85_REPO_URL,
        str(repo_dir),
        commit=_ROBOTIQ_2F85_COMMIT,
    )
    package_path = repo_dir / _ROBOTIQ_2F85_PACKAGE
    return SimpleNamespace(
        REPOSITORY_PATH=str(repo_dir),
        PACKAGE_PATH=str(package_path),
        XACRO_PATH=str(package_path / "urdf" / "robotiq_arg2f_85.xacro"),
    )


def _get_manual_zed_description() -> SimpleNamespace:
    from robot_descriptions._cache import clone_to_directory

    repo_dir = ROBOT_DESCRIPTIONS_CACHE_ROOT / "manual" / _ZED_DESCRIPTION_MODULE
    clone_to_directory(
        _ZED_DESCRIPTION_REPO_URL,
        str(repo_dir),
        commit=_ZED_DESCRIPTION_COMMIT,
    )
    package_path = repo_dir.resolve()
    return SimpleNamespace(
        REPOSITORY_PATH=str(repo_dir),
        PACKAGE_PATH=str(package_path),
    )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--robot",
        choices=sorted(ROBOT_XACRO_PATHS),
        default="cortado",
        help="Robot xacro to expand before launching the viser preview.",
    )
    return parser.parse_args()


def _get_package_roots() -> dict[str, Path]:
    import xacrodoc
    from robot_descriptions import fr3_description

    robotiq_description = _get_manual_robotiq_2f85_description()
    zed_description = _get_manual_zed_description()
    package_roots = {
        "cortado_description": SCRIPT_DIR,
        "franka_description": Path(fr3_description.REPOSITORY_PATH).resolve(),
        _ROBOTIQ_2F85_PACKAGE: Path(robotiq_description.PACKAGE_PATH).resolve(),
        _ZED_DESCRIPTION_PACKAGE: Path(zed_description.PACKAGE_PATH).resolve(),
    }
    xacrodoc.packages.update_package_cache(package_roots)
    return package_roots


def _bake_xacro_path(robot: str, xacro_path: Path, suffix: str) -> Path:
    from robot_descriptions import _xacro

    description_module = SimpleNamespace(
        __name__=f"cortado_description_{robot}_{suffix}",
        XACRO_PATH=str(xacro_path.resolve()),
        XACRO_ARGS=ROBOT_XACRO_ARGS[robot],
        PACKAGE_PATH=str(SCRIPT_DIR.resolve()),
    )
    return Path(_xacro.get_urdf_path(description_module)).resolve()


def _get_composed_robot_paths(robot: str) -> tuple[Path, Path, dict[str, Path]]:
    package_roots = _get_package_roots()
    urdf_xacro_path = ROBOT_XACRO_PATHS[robot]
    srdf_xacro_path = urdf_xacro_path.with_name(
        urdf_xacro_path.name.replace(".urdf.xacro", ".srdf.xacro")
    )
    urdf_path = _bake_xacro_path(robot, urdf_xacro_path, "urdf")
    srdf_path = _bake_xacro_path(robot, srdf_xacro_path, "srdf")
    return urdf_path, srdf_path, package_roots


def package_filename_handler(fname: str, package_roots: dict[str, Path]) -> str:
    """Resolve package:// URIs relative to this project directory."""
    if fname.startswith("file://"):
        parsed = urlparse(fname)
        return unquote(parsed.path)
    fname = unquote(fname)
    if fname.startswith("package://cortado_description/"):
        return str(SCRIPT_DIR / fname.removeprefix("package://cortado_description/"))
    if fname.startswith("package://"):
        package, relpath = fname.removeprefix("package://").split("/", 1)
        if package in package_roots:
            return str(package_roots[package] / relpath)
    return fname


def _local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def _parse_srdf_spheres(srdf_path: Path) -> list[tuple[str, np.ndarray, float]]:
    root = ET.parse(srdf_path).getroot()
    spheres = []
    for link_element in root.iter():
        if _local_name(link_element.tag) != "link_sphere_approximation":
            continue
        link_name = link_element.attrib["link"]
        for sphere_element in link_element:
            if _local_name(sphere_element.tag) != "sphere":
                continue
            center = np.fromstring(sphere_element.attrib["center"], sep=" ", dtype=float)
            radius = float(sphere_element.attrib["radius"])
            if center.shape != (3,):
                raise ValueError(
                    f"Invalid sphere center for {link_name}: "
                    f"{sphere_element.attrib['center']!r}"
                )
            spheres.append((link_name, center, radius))
    return spheres


def _render_srdf_spheres(
    server: viser.ViserServer,
    root_node_name: str,
    urdf_model: yourdfpy.URDF,
    base_frame: str,
    srdf_spheres: list[tuple[str, np.ndarray, float]],
) -> tuple[int, set[str]]:
    missing_links = set()
    rendered = 0
    for index, (link_name, center, radius) in enumerate(srdf_spheres):
        if link_name not in urdf_model.link_map:
            missing_links.add(link_name)
            continue
        T = urdf_model.get_transform(link_name, base_frame)
        position = T[:3, :3] @ center + T[:3, 3]
        safe_link_name = link_name.replace("/", "_")
        server.scene.add_icosphere(
            f"{root_node_name}/srdf_spheres/{safe_link_name}/{index}",
            radius=radius,
            color=(102, 153, 255),
            opacity=0.35,
            position=tuple(position.tolist()),
        )
        rendered += 1
    return rendered, missing_links


def main() -> None:
    args = _parse_args()
    server = viser.ViserServer()
    urdf_path, srdf_path, package_roots = _get_composed_robot_paths(args.robot)

    urdf_model = yourdfpy.URDF.load(
        str(urdf_path),
        filename_handler=lambda fname: package_filename_handler(fname, package_roots),
        force_mesh=True,
        build_collision_scene_graph=True,
        load_collision_meshes=True,
        force_collision_mesh=True,
    )

    base_frame = urdf_model.scene.graph.base_frame
    has_collision = urdf_model.collision_scene is not None
    root_node_name = f"/{args.robot}"

    server.scene.add_frame(root_node_name, show_axes=False)
    viser_urdf = ViserUrdf(
        server,
        urdf_or_path=urdf_model,
        root_node_name=root_node_name,
        load_meshes=True,
        load_collision_meshes=has_collision,
    )
    viser_urdf.show_visual = True
    if has_collision:
        viser_urdf.show_collision = False

    # ViserUrdf leaves its joint frames at identity until update_cfg is called.
    # Push a zero configuration so fixed-joint transforms are applied.
    viser_urdf.update_cfg(np.zeros(len(viser_urdf.get_actuated_joint_names())))

    # Place a visible frame marker on every URDF link.
    for link_name in urdf_model.link_map:
        T = urdf_model.get_transform(link_name, base_frame)
        frame_name = f"{root_node_name}/frames/{link_name}"
        frame = server.scene.add_frame(frame_name, axes_length=0.05, axes_radius=0.002)
        frame.wxyz = viser.transforms.SO3.from_matrix(T[:3, :3]).wxyz
        frame.position = T[:3, 3]

    srdf_spheres = _parse_srdf_spheres(srdf_path)
    rendered_spheres, missing_sphere_links = _render_srdf_spheres(
        server,
        root_node_name,
        urdf_model,
        base_frame,
        srdf_spheres,
    )

    print(f"Expanded {args.robot} URDF to {urdf_path}")
    print(f"Expanded {args.robot} SRDF to {srdf_path}")
    print(f"Rendered {rendered_spheres} SRDF collision spheres.")
    if missing_sphere_links:
        missing = ", ".join(sorted(missing_sphere_links))
        print(f"Skipped SRDF spheres for missing URDF links: {missing}")
    print("Viser running — open http://localhost:8080 in your browser.")

    while True:
        time.sleep(1.0)


if __name__ == "__main__":
    main()
