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
"""Browse the composed Cortado robot in viser."""

from __future__ import annotations

import os
from pathlib import Path
import time
from types import SimpleNamespace
from urllib.parse import unquote, urlparse

import numpy as np
import viser
import viser.transforms
import yourdfpy
from viser.extras import ViserUrdf

SCRIPT_DIR = Path(__file__).resolve().parent.parent
ROBOT_XACRO_PATH = SCRIPT_DIR / "robots" / "cortado.urdf.xacro"
ROBOT_DESCRIPTIONS_CACHE_ROOT = Path(
    os.environ.get("ROBOT_DESCRIPTIONS_CACHE", "~/.cache/robot_descriptions")
).expanduser()

_ROBOTIQ_2F85_REPO_URL = "https://github.com/nickswalker/robotiq-2f-85.git"
_ROBOTIQ_2F85_COMMIT = "2d41b97e990e4020bf7441b8178badfdcf576c65"
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


def _get_composed_urdf_path() -> tuple[Path, dict[str, Path]]:
    import xacrodoc
    from robot_descriptions import _xacro
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

    description_module = SimpleNamespace(
        __name__="cortado_description",
        XACRO_PATH=str(ROBOT_XACRO_PATH.resolve()),
        XACRO_ARGS={
            "robot_name": "cortado",
            "wrist_camera": "true",
            "flip_gripper": "true",
            "camera_mast": "2",
        },
        PACKAGE_PATH=str(SCRIPT_DIR.resolve()),
    )
    return Path(_xacro.get_urdf_path(description_module)).resolve(), package_roots


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


def main() -> None:
    server = viser.ViserServer()
    urdf_path, package_roots = _get_composed_urdf_path()

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

    server.scene.add_frame("/cortado", show_axes=False)
    viser_urdf = ViserUrdf(
        server,
        urdf_or_path=urdf_model,
        root_node_name="/cortado",
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
        frame_name = f"/cortado/frames/{link_name}"
        frame = server.scene.add_frame(frame_name, axes_length=0.05, axes_radius=0.002)
        frame.wxyz = viser.transforms.SO3.from_matrix(T[:3, :3]).wxyz
        frame.position = T[:3, 3]

    print("Viser running — open http://localhost:8080 in your browser.")

    while True:
        time.sleep(1.0)


if __name__ == "__main__":
    main()
