#!/usr/bin/env python3
"""
High-performance Isaac Sim 5.1 headless launcher / benchmark template.

Design goals:
- Max throughput for multi-robot experiments.
- Support "headless + cameras" (offscreen rendering) OR "headless + no rendering".
- Only uses settings that are explicitly documented for Isaac Sim 5.1 / Omniverse Kit.

Run using Isaac Sim python wrapper:
  ./python.sh fast_isaac_sim.py --headless --render-headless --usd-path <scene.usd>

Notes:
- If you do NOT need RGB/depth cameras (e.g., using PhysX raycast LiDAR only),
  run with --headless (no --render-headless) and optionally --render-every large.
"""

from __future__ import annotations

import argparse
import os
import shlex
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Callable, Iterable

import carb
from isaacsim.simulation_app import SimulationApp


# ---------------------------
# CLI
# ---------------------------

def _positive_int(v: str) -> int:
    i = int(v)
    if i < 0:
        raise argparse.ArgumentTypeError("must be >= 0")
    return i


def _positive_nonzero_int(v: str) -> int:
    i = int(v)
    if i < 1:
        raise argparse.ArgumentTypeError("must be >= 1")
    return i


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser("Isaac Sim 5.1 high-performance headless launcher")

    # Core
    p.add_argument("--headless", action="store_true", help="Run without GUI window.")
    p.add_argument("--usd-path", type=str, default="", help="Optional USD to open at startup.")
    p.add_argument("--no-ground-plane", action="store_true", help="Do not add default ground plane.")

    # Physics rate controls
    p.add_argument("--physics-step", type=float, default=1.0 / 60.0, help="Physics dt (seconds).")
    p.add_argument(
        "--min-frame-rate",
        type=float,
        default=10.0,
        help=(
            "Minimum simulation frame rate (Hz). Lower lets sim fall behind without time-dilation."
        ),
    )

    # Rendering controls (for sensors)
    p.add_argument(
        "--render-headless",
        action="store_true",
        help="Enable offscreen rendering in headless mode (required for RTX cameras).",
    )
    p.add_argument(
        "--render-every",
        type=_positive_nonzero_int,
        default=1,
        help="Render once every N physics steps (1 = render every step).",
    )
    p.add_argument(
        "--renderer",
        type=str,
        default="RaytracedLighting",
        choices=["RaytracedLighting", "PathTracing"],
        help="Renderer pipeline to use when rendering is enabled.",
    )
    p.add_argument(
        "--aa-mode",
        type=int,
        default=0,
        help=(
            "RTX AA mode (int). For pure performance use 0. "
            "Common: 0=None, 1=TAA, 3=DLSS. (Matches RTX setting /rtx/post/aa/op)"
        ),
    )
    p.add_argument(
        "--dlss-exec-mode",
        type=int,
        default=0,
        choices=[0, 1, 2, 3],
        help="DLSS execMode: 0=Performance, 1=Balanced, 2=Quality, 3=Auto.",
    )
    p.add_argument(
        "--enable-dlssg",
        action="store_true",
        help="Enable DLSS Frame Generation (/rtx-transient/dlssg/enabled). OFF by default.",
    )
    p.add_argument(
        "--optimize-render",
        action="store_true",
        help="Disable expensive RTX features (reflections/AO/GI) for higher FPS when rendering.",
    )
    p.add_argument(
        "--disable-texture-streaming",
        action="store_true",
        help="Disable texture streaming (can improve perf but increases VRAM usage).",
    )

    # Headless viewport updates
    p.add_argument(
        "--disable-viewport-updates",
        action="store_true",
        default=True,
        help="Disable viewport updates in headless mode (recommended).",
    )
    p.add_argument(
        "--keep-viewport-updates",
        action="store_true",
        help="Force keep viewport updates enabled even in headless mode.",
    )

    # CPU threads (documented 3 knobs)
    p.add_argument(
        "--limit-cpu-threads",
        type=int,
        default=32,
        help="SimulationApp limit_cpu_threads (Python workflows default to 32).",
    )
    p.add_argument(
        "--set-thread-knobs",
        action="store_true",
        help=(
            "Also set the 3 documented thread settings via carb args: "
            "carb.tasking.threadCount, omni.tbb.maxThreadCount, physics.numThreads."
        ),
    )

    # GPU physics
    p.add_argument("--disable-gpu-dynamics", action="store_true", help="Disable GPU dynamics.")
    p.add_argument(
        "--physics-gpu",
        type=int,
        default=0,
        help="GPU index for physics (GPU dynamics uses 1 GPU regardless of multi-GPU).",
    )

    # Multi-GPU rendering
    p.add_argument("--multi-gpu", action="store_true", help="Enable multi-GPU rendering.")
    p.add_argument(
        "--max-gpu-count",
        type=int,
        default=1,
        help="Max GPUs to use when --multi-gpu is set (match to rendered camera count).",
    )

    # Run-loop rate limiting (real-time pacing)
    p.add_argument(
        "--target-sim-hz",
        type=float,
        default=0.0,
        help=(
            "If > 0: enable /app/runLoops/main rate limiting to this Hz "
            "and set /persistent/simulation/minFrameRate accordingly. "
            "If 0: disable rate limiting (run as fast as possible)."
        ),
    )

    # Experience selection
    p.add_argument("--experience", type=str, default="", help="Full path to a .kit to load.")
    p.add_argument(
        "--use-full-experience",
        action="store_true",
        help="Prefer isaacsim.exp.full.kit over base python kit.",
    )
    p.add_argument(
        "--use-fabric-experience",
        action="store_true",
        help="Prefer isaacsim.exp.full.fabric.kit (experimental).",
    )

    # ROS 2
    p.add_argument(
        "--enable-ros2",
        action="store_true",
        default=True,
        help="Enable isaacsim.ros2.bridge extension.",
    )
    p.add_argument(
        "--disable-ros2",
        action="store_true",
        help="Do not enable ROS2 bridge (saves overhead if you don’t need it).",
    )
    p.add_argument(
        "--ros-namespace",
        type=str,
        default="",
        help=(
            "Optional namespace for ROS 2 topics and frame IDs. "
            "By default TF topics stay global unless --namespace-tf-topics is set. "
            "Empty (default) leaves scene topics/frame IDs unchanged."
        ),
    )
    p.add_argument(
        "--namespace-tf-topics",
        action="store_true",
        help=(
            "When --ros-namespace is set, also namespace TF topics by setting TF node namespaces. "
            "Default keeps TF on global /tf and /tf_static."
        ),
    )
    p.add_argument(
        "--disable-physx-laserscan",
        action="store_true",
        help=(
            "Keep authored RTX LaserScan graph as-is. By default, launcher attempts to replace "
            "front /scan RTX helper with a cheaper PhysX LaserScan pipeline when present."
        ),
    )
    p.add_argument(
        "--disable-ros-render-guard",
        action="store_true",
        help=(
            "Disable fail-fast guard that detects playback-tick ROS graphs with non-advancing timeline "
            "(for example headless runs with no effective rendering)."
        ),
    )
    p.add_argument(
        "--skip-ros-ready-check",
        action="store_true",
        help="Skip startup readiness wait for first sample on --ros-ready-topic.",
    )
    p.add_argument(
        "--ros-ready-topic",
        type=str,
        default="/clock",
        help="ROS topic used for startup readiness gating.",
    )
    p.add_argument(
        "--ros-ready-timeout",
        type=float,
        default=90.0,
        help="Seconds to wait for first message on --ros-ready-topic before failing.",
    )

    # Future profile controls (launcher-driven)
    p.add_argument(
        "--robot-profile",
        type=str,
        choices=["scene-default", "sensors-off", "single-front-camera"],
        default="scene-default",
        help=(
            "Launcher-driven robot feature profile. scene-default keeps authored scene behavior; "
            "other profiles are baseline patterns for later launches."
        ),
    )
    p.add_argument(
        "--profile-camera-width",
        type=_positive_nonzero_int,
        default=320,
        help="Camera width used by profile overrides when applicable.",
    )
    p.add_argument(
        "--profile-camera-height",
        type=_positive_nonzero_int,
        default=240,
        help="Camera height used by profile overrides when applicable.",
    )

    # Benchmark loop
    p.add_argument("--max-steps", type=int, default=2000, help="Steps to run, -1 for unlimited.")
    p.add_argument("--fps-report-interval", type=int, default=240, help="Print FPS every N steps.")

    return p.parse_args()


# ---------------------------
# Helpers
# ---------------------------

def _kit_from_exp_path(exp_path: str, name: str) -> str:
    if not exp_path:
        return ""
    p = os.path.join(exp_path, name)
    return p if os.path.isfile(p) else ""


def _as_cli_setting(path: str, value) -> str:
    # carb/kit CLI style: --/path/to/setting=value
    if isinstance(value, bool):
        v = "true" if value else "false"
    else:
        v = str(value)
    if not path.startswith("/"):
        path = "/" + path
    return f"--{path}={v}"


def _is_tf_bridge_type(node_type: str) -> bool:
    return node_type in {
        "isaacsim.ros2.bridge.ROS2PublishTransformTree",
        "isaacsim.ros2.bridge.ROS2PublishRawTransformTree",
    }


def _is_clock_bridge_type(node_type: str) -> bool:
    return node_type == "isaacsim.ros2.bridge.ROS2PublishClock"


def _iter_ros2_bridge_nodes(stage) -> Iterable[tuple]:
    for prim in stage.Traverse():
        node_type_attr = prim.GetAttribute("node:type")
        if not node_type_attr or not node_type_attr.HasValue():
            continue
        node_type = node_type_attr.Get()
        if isinstance(node_type, str) and node_type.startswith("isaacsim.ros2.bridge."):
            yield prim, node_type


def _normalize_frame_id(frame_id: str, namespace: str) -> str:
    if not namespace:
        return frame_id
    clean = frame_id.strip().lstrip("/")
    if not clean:
        return frame_id
    prefix = f"{namespace}/"
    if clean.startswith(prefix):
        return clean
    return prefix + clean


def _denormalize_frame_id(frame_id: str, namespace: str) -> str:
    clean = frame_id.strip().lstrip("/")
    if not clean or not namespace:
        return clean
    prefix = f"{namespace}/"
    if clean.startswith(prefix):
        return clean[len(prefix):]
    return clean


def _is_transform_tree_bridge_type(node_type: str) -> bool:
    return node_type == "isaacsim.ros2.bridge.ROS2PublishTransformTree"


def _iter_attr_with_connected_sources(stage, attr):
    seen_paths: set[str] = set()
    stack = [attr]
    while stack:
        current = stack.pop()
        if not current or not current.IsValid():
            continue
        current_path = current.GetPath().pathString
        if current_path in seen_paths:
            continue
        seen_paths.add(current_path)
        yield current
        if not current.HasAuthoredConnections():
            continue
        for source_path in current.GetConnections():
            source_attr = stage.GetAttributeAtPath(source_path)
            if source_attr and source_attr.IsValid():
                stack.append(source_attr)


def _resolve_connected_string_value(stage, attr) -> str:
    for candidate in _iter_attr_with_connected_sources(stage, attr):
        try:
            value = candidate.Get()
        except Exception:
            continue
        if value is None:
            continue
        text = value if isinstance(value, str) else str(value)
        if text:
            return text
    return ""


def _set_connected_string_value(stage, attr, desired_value: str) -> int:
    updates = 0
    for candidate in _iter_attr_with_connected_sources(stage, attr):
        type_name = str(candidate.GetTypeName())
        if type_name not in {"string", "token"}:
            continue
        try:
            current = candidate.Get()
        except Exception:
            current = None
        current_text = "" if current is None else str(current)
        if current_text != desired_value:
            candidate.Set(desired_value)
            updates += 1
    return updates


def _is_frame_attr_name(low_name: str) -> bool:
    if "frame" not in low_name:
        return False
    if "frameskip" in low_name:
        return False
    return (
        "frameid" in low_name
        or "frame_id" in low_name
        or "framenamemap" in low_name
        or low_name.endswith("frame")
    )


def _should_keep_tf_anchor_unprefixed(node_type: str, attr_name: str, has_transform_tree: bool) -> bool:
    if not has_transform_tree:
        return False
    if node_type == "isaacsim.ros2.bridge.ROS2PublishRawTransformTree" and attr_name == "inputs:childFrameId":
        return True
    if node_type == "isaacsim.ros2.bridge.ROS2PublishOdometry" and attr_name == "inputs:chassisFrameId":
        return True
    return False


def _normalize_ros2_namespace(stage, namespace: str, namespace_tf_topics: bool = False) -> dict[str, int]:
    bridge_nodes = list(_iter_ros2_bridge_nodes(stage))
    has_transform_tree = any(_is_transform_tree_bridge_type(node_type) for _, node_type in bridge_nodes)

    stats = {
        "bridge_nodes": 0,
        "namespace_updates": 0,
        "topic_updates": 0,
        "frame_updates": 0,
        "tf_tree_nodes": 0,
    }

    for prim, node_type in bridge_nodes:
        stats["bridge_nodes"] += 1
        is_tf = _is_tf_bridge_type(node_type)
        is_clock = _is_clock_bridge_type(node_type)
        if _is_transform_tree_bridge_type(node_type):
            stats["tf_tree_nodes"] += 1

        node_ns_attr = prim.GetAttribute("inputs:nodeNamespace")
        if node_ns_attr:
            desired_ns = ""
            if namespace and (not is_clock) and (not is_tf or namespace_tf_topics):
                desired_ns = namespace
            stats["namespace_updates"] += _set_connected_string_value(stage, node_ns_attr, desired_ns)

        for attr in prim.GetAttributes():
            name = attr.GetName()
            if not name.startswith("inputs:"):
                continue
            low_name = name.lower()

            if "topicname" in low_name:
                current_topic = _resolve_connected_string_value(stage, attr)
                desired_topic = None
                if is_clock:
                    desired_topic = "/clock"
                elif is_tf:
                    if namespace and namespace_tf_topics:
                        if current_topic:
                            desired_topic = current_topic.lstrip("/")
                            ns_prefix = f"{namespace}/"
                            if desired_topic.startswith(ns_prefix):
                                desired_topic = desired_topic[len(ns_prefix):]
                    elif current_topic:
                        tf_topic = current_topic.lstrip("/")
                        desired_topic = "/tf_static" if tf_topic == "tf_static" else "/tf"
                elif namespace and current_topic:
                    if current_topic in {"tf", "tf_static", "clock", "/tf", "/tf_static", "/clock"}:
                        desired_topic = "/" + current_topic.lstrip("/")
                    else:
                        topic_rel = current_topic.lstrip("/")
                        ns_prefix = f"{namespace}/"
                        if topic_rel.startswith(ns_prefix):
                            topic_rel = topic_rel[len(ns_prefix):]
                        desired_topic = topic_rel
                if desired_topic is not None:
                    stats["topic_updates"] += _set_connected_string_value(stage, attr, desired_topic)
                continue

            if not namespace:
                continue
            if not _is_frame_attr_name(low_name):
                continue

            current_frame = _resolve_connected_string_value(stage, attr)
            if not current_frame and node_type == "isaacsim.ros2.bridge.ROS2PublishOdometry":
                if name == "inputs:chassisFrameId":
                    current_frame = "base_link"
                elif name == "inputs:odomFrameId":
                    current_frame = "odom"
            if not current_frame:
                continue
            if _should_keep_tf_anchor_unprefixed(node_type, name, has_transform_tree):
                desired_frame = _denormalize_frame_id(current_frame, namespace)
            else:
                desired_frame = _normalize_frame_id(current_frame, namespace)
            stats["frame_updates"] += _set_connected_string_value(stage, attr, desired_frame)

    return stats


def _find_tf_publishers(stage) -> list[tuple[str, str]]:
    tf_publishers: list[tuple[str, str]] = []
    for prim, node_type in _iter_ros2_bridge_nodes(stage):
        if _is_tf_bridge_type(node_type):
            tf_publishers.append((prim.GetPath().pathString, node_type))
    return tf_publishers


def _should_render_step(step: int, want_render: bool, render_every: int) -> bool:
    if not want_render:
        return False
    if render_every <= 1:
        return True
    return (step % render_every) == 0


def _compute_physx_scan_rate_hint_hz(args: argparse.Namespace, want_render: bool) -> float:
    if args.target_sim_hz > 0.0:
        base_hz = float(args.target_sim_hz)
    elif args.physics_step > 0.0:
        base_hz = 1.0 / float(args.physics_step)
    else:
        base_hz = 0.0
    if base_hz <= 0.0:
        return 0.0
    if want_render and args.render_every > 1:
        return base_hz / float(args.render_every)
    return base_hz


def _find_playback_tick_driven_ros_graphs(stage) -> list[str]:
    playback_graphs: set[str] = set()
    ros_graphs: set[str] = set()

    for prim in stage.Traverse():
        node_type_attr = prim.GetAttribute("node:type")
        if not node_type_attr or not node_type_attr.HasValue():
            continue
        node_type = node_type_attr.Get()
        if not isinstance(node_type, str):
            continue
        graph_path = prim.GetPath().GetParentPath().pathString
        if node_type == "omni.graph.action.OnPlaybackTick":
            playback_graphs.add(graph_path)
        if node_type.startswith("isaacsim.ros2.bridge."):
            ros_graphs.add(graph_path)

    return sorted(playback_graphs.intersection(ros_graphs))


def _apply_robot_profile(stage, profile: str, camera_width: int, camera_height: int) -> dict[str, int]:
    stats = {
        "camera_nodes_seen": 0,
        "camera_enable_updates": 0,
        "camera_size_updates": 0,
        "lidar_enable_updates": 0,
    }
    if profile == "scene-default":
        return stats

    def _set_uint_attr(attr_path: str, value: int) -> bool:
        attr = stage.GetAttributeAtPath(attr_path)
        if not attr:
            return False
        try:
            current = attr.Get()
        except Exception:
            current = None
        if current == value:
            return False
        attr.Set(value)
        return True

    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        if path.endswith("/left_camera_render_product") or path.endswith("/right_camera_render_product"):
            stats["camera_nodes_seen"] += 1
            enable_attr = path + ".inputs:enabled"
            if profile == "sensors-off":
                if _set_bool_attr(stage, enable_attr, False):
                    stats["camera_enable_updates"] += 1
            elif profile == "single-front-camera":
                desired = "/front_hawk/left_camera_render_product" in path
                if _set_bool_attr(stage, enable_attr, desired):
                    stats["camera_enable_updates"] += 1
                if desired:
                    if _set_uint_attr(path + ".inputs:width", camera_width):
                        stats["camera_size_updates"] += 1
                    if _set_uint_attr(path + ".inputs:height", camera_height):
                        stats["camera_size_updates"] += 1

        if "/ros_lidars/" in path:
            node_name = path.rsplit("/", 1)[-1]
            if node_name in {
                "front_2d_lidar_render_product",
                "back_2d_lidar_render_product",
                "front_3d_lidar_render_product",
                "publish_front_2d_lidar_scan",
                "publish_back_2d_lidar_scan",
                "publish_front_3d_lidar_scan",
            }:
                if _set_bool_attr(stage, path + ".inputs:enabled", False):
                    stats["lidar_enable_updates"] += 1

    return stats


def _override_hospital_camera_resolution(stage, width: int = 300, height: int = 200) -> dict[str, int]:
    stats = {
        "camera_nodes_seen": 0,
        "camera_size_updates": 0,
    }
    if width < 1 or height < 1:
        return stats

    def _set_uint_attr(attr_path: str, value: int) -> bool:
        attr = stage.GetAttributeAtPath(attr_path)
        if not attr:
            return False
        try:
            current = attr.Get()
        except Exception:
            current = None
        if current == value:
            return False
        attr.Set(value)
        return True

    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        if not (path.endswith("/left_camera_render_product") or path.endswith("/right_camera_render_product")):
            continue
        stats["camera_nodes_seen"] += 1
        if _set_uint_attr(path + ".inputs:width", width):
            stats["camera_size_updates"] += 1
        if _set_uint_attr(path + ".inputs:height", height):
            stats["camera_size_updates"] += 1

    return stats


def _wait_for_ros_topic_sample(
    *,
    topic: str,
    timeout_s: float,
    simulation_app,
    step_once: Callable[[], None],
) -> tuple[bool, str]:
    topic = topic.strip() or "/clock"
    if not topic.startswith("/"):
        topic = "/" + topic

    ros_env = os.environ.copy()
    ros_env.pop("PYTHONHOME", None)
    ros_env.pop("PYTHONPATH", None)

    ros2_bin = shutil.which("ros2", path=ros_env.get("PATH"))
    if not ros2_bin:
        return False, "ROS readiness check unavailable: ros2 CLI not found in PATH"

    ros_distro = (os.environ.get("ROS_DISTRO") or "jazzy").strip()
    ros_setup = Path(f"/opt/ros/{ros_distro}/setup.bash")
    source_prefix = ""
    if ros_setup.is_file():
        source_prefix = f"source {shlex.quote(str(ros_setup))} >/dev/null 2>&1 && "

    echo_cmd = source_prefix + f"ros2 topic echo {shlex.quote(topic)} --once"
    type_cmd = source_prefix + f"ros2 topic type {shlex.quote(topic)}"

    def _launch_echo():
        return subprocess.Popen(
            ["bash", "-lc", echo_cmd],
            env=ros_env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
        )

    probe_proc = None
    last_nonzero = ""
    next_launch_time = 0.0
    try:
        deadline = time.perf_counter() + timeout_s
        while simulation_app.is_running() and time.perf_counter() < deadline:
            if probe_proc is None and time.perf_counter() >= next_launch_time:
                probe_proc = _launch_echo()

            step_once()
            if probe_proc is None:
                continue

            rc = probe_proc.poll()
            if rc is None:
                continue

            if rc == 0:
                topic_type = "unknown"
                try:
                    type_result = subprocess.run(
                        ["bash", "-lc", type_cmd],
                        env=ros_env,
                        capture_output=True,
                        text=True,
                        timeout=5.0,
                        check=False,
                    )
                    if type_result.returncode == 0:
                        maybe_type = type_result.stdout.strip().splitlines()
                        if maybe_type and maybe_type[0]:
                            topic_type = maybe_type[0]
                except Exception:
                    pass
                return True, topic_type

            stderr_text = ""
            try:
                _stdout, stderr_text = probe_proc.communicate(timeout=0.2)
            except Exception:
                pass
            stderr_text = (stderr_text or "").strip()
            if stderr_text:
                last_nonzero = stderr_text.splitlines()[-1]
            else:
                last_nonzero = f"ros2 topic echo exited with code {rc}"
            probe_proc = None
            next_launch_time = time.perf_counter() + 0.5

        if last_nonzero:
            return False, last_nonzero
        return False, "timeout waiting for first sample"
    finally:
        if probe_proc is not None and probe_proc.poll() is None:
            try:
                probe_proc.terminate()
            except Exception:
                pass
            try:
                probe_proc.wait(timeout=2.0)
            except Exception:
                try:
                    probe_proc.kill()
                except Exception:
                    pass


def _set_bool_attr(stage, attr_path: str, value: bool) -> bool:
    attr = stage.GetAttributeAtPath(attr_path)
    if not attr or not attr.IsValid():
        return False
    updates = 0
    for candidate in _iter_attr_with_connected_sources(stage, attr):
        type_name = str(candidate.GetTypeName())
        if type_name not in {"bool", "uchar"}:
            continue
        try:
            current = candidate.Get()
        except Exception:
            current = None
        current_bool = None if current is None else bool(current)
        if current_bool != bool(value):
            candidate.Set(bool(value))
            updates += 1
    return updates > 0


def _set_prim_active(stage, prim_path: str, active: bool) -> bool:
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return False
    desired = bool(active)
    if prim.IsActive() == desired:
        return False
    try:
        prim.SetActive(desired)
    except Exception:
        return False
    return True


def _set_float_attr(stage, attr_path: str, value: float, tol: float = 1e-6) -> bool:
    attr = stage.GetAttributeAtPath(attr_path)
    if not attr or not attr.IsValid():
        return False
    try:
        current = attr.Get()
    except Exception:
        current = None
    try:
        if current is not None and abs(float(current) - float(value)) <= tol:
            return False
    except Exception:
        pass
    attr.Set(float(value))
    return True


def _resolve_rel_target_path(stage, rel_path: str) -> str:
    rel = stage.GetRelationshipAtPath(rel_path)
    if rel and rel.IsValid():
        try:
            targets = rel.GetTargets()
        except Exception:
            targets = []
        if targets:
            return targets[0].pathString

    # Fallback in case the input is authored as a non-relationship path-like attribute.
    attr = stage.GetAttributeAtPath(rel_path)
    if attr and attr.IsValid():
        try:
            value = attr.Get()
        except Exception:
            value = None
        if value is None:
            return ""
        if isinstance(value, (list, tuple)) and value:
            first = value[0]
            return getattr(first, "pathString", str(first))
        return getattr(value, "pathString", str(value))
    return ""


def _set_rel_target_path(stage, rel_path: str, target_path: str) -> bool:
    if not target_path:
        return False
    rel = stage.GetRelationshipAtPath(rel_path)
    if rel and rel.IsValid():
        try:
            current = rel.GetTargets()
        except Exception:
            current = []
        if len(current) == 1 and current[0].pathString == target_path:
            return False
        try:
            from pxr import Sdf

            rel.SetTargets([Sdf.Path(target_path)])
            return True
        except Exception:
            try:
                rel.SetTargets([target_path])
                return True
            except Exception:
                return False

    attr = stage.GetAttributeAtPath(rel_path)
    if not attr or not attr.IsValid():
        return False
    current = None
    try:
        current = attr.Get()
    except Exception:
        current = None
    current_path = getattr(current, "pathString", str(current)) if current is not None else ""
    if current_path == target_path:
        return False
    try:
        attr.Set(target_path)
        return True
    except Exception:
        return False


def _vec3_tuple(value) -> tuple[float, float, float] | None:
    if value is None:
        return None
    try:
        return (float(value[0]), float(value[1]), float(value[2]))
    except Exception:
        pass
    for names in (("x", "y", "z"), ("real", "imaginary", "k")):
        try:
            return (
                float(getattr(value, names[0])),
                float(getattr(value, names[1])),
                float(getattr(value, names[2])),
            )
        except Exception:
            continue
    return None


def _set_vec3_attr(stage, attr_path: str, xyz: tuple[float, float, float]) -> bool:
    attr = stage.GetAttributeAtPath(attr_path)
    if not attr or not attr.IsValid():
        return False
    try:
        current = attr.Get()
    except Exception:
        current = None
    current_xyz = _vec3_tuple(current)
    if current_xyz and all(abs(a - b) < 1e-6 for a, b in zip(current_xyz, xyz)):
        return False
    type_name = str(attr.GetTypeName())
    try:
        from pxr import Gf

        if type_name == "double3":
            attr.Set(Gf.Vec3d(*xyz))
        elif type_name == "float3":
            attr.Set(Gf.Vec3f(*xyz))
        else:
            attr.Set(xyz)
    except Exception:
        attr.Set(xyz)
    return True


def _sync_lidar_pose_from_mount(stage, lidar_path: str, mount_target_path: str, z_lift_m: float) -> int:
    """Copy local pose from mount target to lidar and add upward z lift."""
    if not lidar_path or not mount_target_path:
        # If mount target is missing, still try a simple local z lift.
        mount_target_path = ""

    updates = 0
    src_translate_attr = stage.GetAttributeAtPath(f"{mount_target_path}.xformOp:translate")
    dst_translate_attr = stage.GetAttributeAtPath(f"{lidar_path}.xformOp:translate")
    desired_xyz: tuple[float, float, float] | None = None

    if src_translate_attr and src_translate_attr.IsValid():
        try:
            src_translate = src_translate_attr.Get()
        except Exception:
            src_translate = None
        src_xyz = _vec3_tuple(src_translate)
        if src_xyz is not None:
            desired_xyz = (src_xyz[0], src_xyz[1], src_xyz[2] + float(z_lift_m))

    # Fallback: if mount local pose is unavailable, raise current lidar local z in-place.
    if desired_xyz is None and dst_translate_attr and dst_translate_attr.IsValid():
        try:
            dst_translate = dst_translate_attr.Get()
        except Exception:
            dst_translate = None
        dst_xyz = _vec3_tuple(dst_translate)
        if dst_xyz is not None:
            desired_xyz = (dst_xyz[0], dst_xyz[1], dst_xyz[2] + float(z_lift_m))

    if desired_xyz is not None:
        if _set_vec3_attr(stage, f"{lidar_path}.xformOp:translate", desired_xyz):
            updates += 1

    src_orient_attr = stage.GetAttributeAtPath(f"{mount_target_path}.xformOp:orient")
    dst_orient_attr = stage.GetAttributeAtPath(f"{lidar_path}.xformOp:orient")
    if src_orient_attr and src_orient_attr.IsValid() and dst_orient_attr and dst_orient_attr.IsValid():
        try:
            src_orient = src_orient_attr.Get()
        except Exception:
            src_orient = None
        try:
            dst_orient = dst_orient_attr.Get()
        except Exception:
            dst_orient = None
        if src_orient is not None and dst_orient != src_orient:
            try:
                dst_orient_attr.Set(src_orient)
                updates += 1
            except Exception:
                pass

    return updates


def _apply_hospital_rtx_front_lidar_only(stage, keep_namespace: str = "carter1") -> dict[str, object]:
    import omni.graph.core as og

    stats: dict[str, object] = {
        "applied": False,
        "graphs_seen": 0,
        "graphs_updated": 0,
        "rtx_targets_seen": 0,
        "rtx_targets_missing": 0,
        "rtx_publish_nodes_typed": 0,
        "rtx_publish_nodes_untyped": 0,
        "rtx_publish_types": [],
        "rtx_enable_updates": 0,
        "rtx_config_updates": 0,
        "rtx_connections": 0,
        "physx_nodes_disabled": 0,
        "physx_lidars_disabled": 0,
        "keep_namespace": keep_namespace,
        "enabled_graphs": [],
        "front_pub_enabled": None,
        "front_pub_topic": "",
        "front_pub_type": "",
        "front_source_path": "",
        "front_source_link_updates": 0,
        "front_pub_render_product": "",
        "front_render_product_output": "",
        "rtx_prim_active_updates": 0,
        "rtx_helpers_forced_off": 0,
    }

    def _attr_exists(attr_path: str) -> bool:
        attr = stage.GetAttributeAtPath(attr_path)
        return bool(attr and attr.IsValid())

    def _looks_like_rtx_source_prim(prim) -> bool:
        if not prim or not prim.IsValid():
            return False
        type_name = str(prim.GetTypeName())
        if type_name in {"Camera", "OmniLidar"}:
            return True
        if "lidar" in type_name.lower() or "camera" in type_name.lower():
            return True
        try:
            applied = prim.GetAppliedSchemas()
        except Exception:
            applied = []
        for schema_name in applied:
            low = str(schema_name).lower()
            if "lidar" in low or "camera" in low:
                return True
        return False

    def _find_front_rtx_source(robot_root: str) -> str:
        direct_candidates = [
            f"{robot_root}/chassis_link/sensors/front_RPLidar/RPLIDAR_S2E",
            f"{robot_root}/chassis_link/sensors/front_RPLidar",
        ]
        for candidate in direct_candidates:
            prim = stage.GetPrimAtPath(candidate)
            if _looks_like_rtx_source_prim(prim):
                return candidate

        sensor_root = stage.GetPrimAtPath(f"{robot_root}/chassis_link/sensors/front_RPLidar")
        if not sensor_root or not sensor_root.IsValid():
            return ""
        stack = list(sensor_root.GetChildren())
        while stack:
            child = stack.pop(0)
            if _looks_like_rtx_source_prim(child):
                return child.GetPath().pathString
            stack.extend(child.GetChildren())
        return ""

    def _set_int_attr(attr_path: str, value: int) -> bool:
        attr = stage.GetAttributeAtPath(attr_path)
        if not attr or not attr.IsValid():
            return False
        try:
            current = attr.Get()
        except Exception:
            current = None
        try:
            if current is not None and int(current) == int(value):
                return False
        except Exception:
            pass
        try:
            attr.Set(int(value))
            return True
        except Exception:
            return False

    graph_paths: list[str] = []
    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        if prim.GetName() == "ros_lidars" and path.startswith("/World/Nova_Carter_ROS_"):
            graph_paths.append(path)
    graph_paths = sorted(set(graph_paths))
    stats["graphs_seen"] = len(graph_paths)
    allowed_rtx_helper_paths: set[str] = set()

    for graph_path in graph_paths:
        graph_prim = stage.GetPrimAtPath(graph_path)
        if not graph_prim or not graph_prim.IsValid():
            continue
        robot_root = graph_prim.GetPath().GetParentPath().pathString

        node_ns = ""
        ns_attr = stage.GetAttributeAtPath(f"{graph_path}/node_namespace.inputs:value")
        if ns_attr and ns_attr.IsValid():
            node_ns = _resolve_connected_string_value(stage, ns_attr).strip()

        if not node_ns:
            if robot_root.endswith("_1"):
                node_ns = "carter1"
            elif robot_root.endswith("_2"):
                node_ns = "carter2"
            elif robot_root.endswith("_3"):
                node_ns = "carter3"

        keep_front = node_ns == keep_namespace
        if keep_front:
            stats["enabled_graphs"].append(graph_path)
            allowed_rtx_helper_paths.add(f"{graph_path}/publish_front_2d_lidar_scan")

        pub_prim = stage.GetPrimAtPath(f"{graph_path}/publish_front_2d_lidar_scan")
        if pub_prim and pub_prim.IsValid():
            node_type_attr = pub_prim.GetAttribute("node:type")
            if node_type_attr and node_type_attr.HasValue():
                stats["rtx_publish_nodes_typed"] += 1
                node_type_value = str(node_type_attr.Get())
                if node_type_value not in stats["rtx_publish_types"]:
                    stats["rtx_publish_types"].append(node_type_value)
            else:
                stats["rtx_publish_nodes_untyped"] += 1

        desired_enabled_nodes = {
            "publish_front_2d_lidar_scan": keep_front,
            "front_2d_lidar_render_product": keep_front,
            "publish_back_2d_lidar_scan": False,
            "back_2d_lidar_render_product": False,
            "publish_front_3d_lidar_scan": False,
            "front_3d_lidar_render_product": False,
        }

        for node_name, node_enabled in desired_enabled_nodes.items():
            attr_path = f"{graph_path}/{node_name}.inputs:enabled"
            attr = stage.GetAttributeAtPath(attr_path)
            if attr and attr.IsValid():
                stats["rtx_targets_seen"] += 1
            else:
                stats["rtx_targets_missing"] += 1
            if _set_bool_attr(stage, attr_path, node_enabled):
                stats["rtx_enable_updates"] += 1
            if _set_prim_active(stage, f"{graph_path}/{node_name}", node_enabled):
                stats["rtx_prim_active_updates"] += 1

        if keep_front:
            front_source = _find_front_rtx_source(robot_root)
            if front_source:
                stats["front_source_path"] = front_source
                if _set_rel_target_path(
                    stage,
                    f"{graph_path}/front_2d_lidar_render_product.inputs:cameraPrim",
                    front_source,
                ):
                    stats["rtx_config_updates"] += 1
                    stats["front_source_link_updates"] += 1
                _set_bool_attr(stage, f"{front_source}.enabled", True)
            if _set_int_attr(f"{graph_path}/front_2d_lidar_render_product.inputs:width", 1):
                stats["rtx_config_updates"] += 1
            if _set_int_attr(f"{graph_path}/front_2d_lidar_render_product.inputs:height", 1):
                stats["rtx_config_updates"] += 1
            render_path_input_attr = stage.GetAttributeAtPath(
                f"{graph_path}/front_2d_lidar_render_product.inputs:renderProductPath"
            )
            if render_path_input_attr and render_path_input_attr.IsValid():
                desired_rp_path = f"/Render/OmniverseKit/HydraTextures/{node_ns or 'carter1'}_front_2d_lidar"
                stats["rtx_config_updates"] += _set_connected_string_value(
                    stage, render_path_input_attr, desired_rp_path
                )

            topic_attr = stage.GetAttributeAtPath(f"{graph_path}/publish_front_2d_lidar_scan.inputs:topicName")
            if topic_attr and topic_attr.IsValid():
                stats["rtx_config_updates"] += _set_connected_string_value(stage, topic_attr, "front_2d_lidar/scan")
            type_attr = stage.GetAttributeAtPath(f"{graph_path}/publish_front_2d_lidar_scan.inputs:type")
            if type_attr and type_attr.IsValid():
                stats["rtx_config_updates"] += _set_connected_string_value(stage, type_attr, "laser_scan")
            frame_attr = stage.GetAttributeAtPath(f"{graph_path}/publish_front_2d_lidar_scan.inputs:frameId")
            if frame_attr and frame_attr.IsValid():
                stats["rtx_config_updates"] += _set_connected_string_value(stage, frame_attr, "front_2d_lidar")
            pub_exec_source = f"{graph_path}/on_playback_tick.outputs:tick"
            if _attr_exists(f"{graph_path}/front_2d_lidar_render_product.outputs:execOut"):
                pub_exec_source = f"{graph_path}/front_2d_lidar_render_product.outputs:execOut"
            connects = []
            for src_attr, dst_attr in (
                (pub_exec_source, f"{graph_path}/publish_front_2d_lidar_scan.inputs:execIn"),
                (f"{graph_path}/ros2_context.outputs:context", f"{graph_path}/publish_front_2d_lidar_scan.inputs:context"),
                (f"{graph_path}/ros2_qos_profile.outputs:qosProfile", f"{graph_path}/publish_front_2d_lidar_scan.inputs:qosProfile"),
                (f"{graph_path}/node_namespace.inputs:value", f"{graph_path}/publish_front_2d_lidar_scan.inputs:nodeNamespace"),
                (
                    f"{graph_path}/front_2d_lidar_render_product.outputs:renderProductPath",
                    f"{graph_path}/publish_front_2d_lidar_scan.inputs:renderProductPath",
                ),
            ):
                if _attr_exists(src_attr) and _attr_exists(dst_attr):
                    connects.append((src_attr, dst_attr))
            if _attr_exists(f"{graph_path}/on_playback_tick.outputs:tick") and _attr_exists(
                f"{graph_path}/front_2d_lidar_render_product.inputs:execIn"
            ):
                connects.append(
                    (
                        f"{graph_path}/on_playback_tick.outputs:tick",
                        f"{graph_path}/front_2d_lidar_render_product.inputs:execIn",
                    )
                )
            if connects:
                og.Controller.edit(graph_path, {og.Controller.Keys.CONNECT: connects})
                stats["rtx_connections"] += len(connects)
            enabled_attr = stage.GetAttributeAtPath(f"{graph_path}/publish_front_2d_lidar_scan.inputs:enabled")
            if enabled_attr and enabled_attr.IsValid():
                try:
                    stats["front_pub_enabled"] = bool(enabled_attr.Get())
                except Exception:
                    stats["front_pub_enabled"] = None
            if topic_attr and topic_attr.IsValid():
                stats["front_pub_topic"] = _resolve_connected_string_value(stage, topic_attr)
            if type_attr and type_attr.IsValid():
                stats["front_pub_type"] = _resolve_connected_string_value(stage, type_attr)
            render_path_attr = stage.GetAttributeAtPath(
                f"{graph_path}/publish_front_2d_lidar_scan.inputs:renderProductPath"
            )
            if render_path_attr and render_path_attr.IsValid():
                stats["front_pub_render_product"] = _resolve_connected_string_value(stage, render_path_attr)
            render_output_attr = stage.GetAttributeAtPath(
                f"{graph_path}/front_2d_lidar_render_product.outputs:renderProductPath"
            )
            if render_output_attr and render_output_attr.IsValid():
                stats["front_render_product_output"] = _resolve_connected_string_value(stage, render_output_attr)

        for child in graph_prim.GetChildren():
            child_name = child.GetName().lower()
            node_type_attr = child.GetAttribute("node:type")
            node_type = node_type_attr.Get() if node_type_attr and node_type_attr.HasValue() else ""
            is_physx_read = node_type == "isaacsim.sensors.physx.IsaacReadLidarBeams"
            is_physx_pub = ("physx" in child_name) and (node_type == "isaacsim.ros2.bridge.ROS2PublishLaserScan")
            if is_physx_read or is_physx_pub:
                if _set_prim_active(stage, f"{graph_path}/{child.GetName()}", False):
                    stats["physx_nodes_disabled"] += 1
                if _set_bool_attr(stage, f"{graph_path}/{child.GetName()}.inputs:enabled", False):
                    stats["physx_nodes_disabled"] += 1

        stats["graphs_updated"] += 1

    # Guardrail: if any RTX lidar helper node exists outside the kept front path, force it inactive.
    for prim, node_type in _iter_ros2_bridge_nodes(stage):
        if node_type != "isaacsim.ros2.bridge.ROS2RtxLidarHelper":
            continue
        path = prim.GetPath().pathString
        should_keep = path in allowed_rtx_helper_paths
        if _set_prim_active(stage, path, should_keep):
            if not should_keep:
                stats["rtx_helpers_forced_off"] += 1
            else:
                stats["rtx_prim_active_updates"] += 1
        if not should_keep:
            _set_bool_attr(stage, path + ".inputs:enabled", False)

    for prim in stage.Traverse():
        if str(prim.GetTypeName()) != "Lidar":
            continue
        path = prim.GetPath().pathString
        if not path.startswith("/World/Nova_Carter_ROS_"):
            continue
        # Keep authored RTX lidar sensors intact; only disable PhysX-style lidar prims.
        is_physx_style = ("PhysX_LaserScan" in path) or path.endswith("/Lidar")
        if not is_physx_style:
            continue
        if _set_bool_attr(stage, f"{path}.enabled", False):
            stats["physx_lidars_disabled"] += 1

    stats["applied"] = stats["graphs_updated"] > 0
    return stats


def _configure_physx_laserscan_multi(stage, scan_rate_hint_hz: float = 0.0) -> dict[str, object]:
    stats: dict[str, object] = {
        "applied": False,
        "reason": "",
        "graphs_seen": 0,
        "graphs_configured": 0,
        "physx_lidars_found": 0,
        "mount_lidars_created": 0,
        "mount_pose_updates": 0,
        "lidar_param_updates": 0,
        "root_lidars_disabled": 0,
        "extra_publishers_disabled": 0,
        "created_nodes": 0,
        "set_values": 0,
        "connections": 0,
        "rtx_nodes_disabled": 0,
        "scan_rate_hint_hz": max(float(scan_rate_hint_hz), 0.0),
        "source_bindings": [],
    }

    graph_paths: list[str] = []
    for prim in stage.Traverse():
        if prim.GetName() == "ros_lidars":
            graph_paths.append(prim.GetPath().pathString)
    graph_paths = sorted(set(graph_paths))
    stats["graphs_seen"] = len(graph_paths)
    if not graph_paths:
        stats["reason"] = "no ros_lidars graphs found"
        return stats

    physx_lidar_paths: list[str] = []
    for prim in stage.Traverse():
        if str(prim.GetTypeName()) != "Lidar":
            continue
        path = prim.GetPath().pathString
        if "/ros_lidars/" in path:
            continue
        physx_lidar_paths.append(path)
    physx_lidar_paths = sorted(set(physx_lidar_paths))
    stats["physx_lidars_found"] = len(physx_lidar_paths)

    def _attr_exists(attr_path: str) -> bool:
        attr = stage.GetAttributeAtPath(attr_path)
        return bool(attr and attr.IsValid())

    def _resolve_input(graph_path: str, node_name: str, input_name: str, fallback: str) -> str:
        attr = stage.GetAttributeAtPath(f"{graph_path}/{node_name}.inputs:{input_name}")
        if not attr or not attr.IsValid():
            return fallback
        value = _resolve_connected_string_value(stage, attr)
        return value or fallback

    def _is_root_level_lidar(path: str, robot_root: str) -> bool:
        if not path.startswith(robot_root + "/"):
            return False
        rel = path[len(robot_root) + 1 :]
        return "/" not in rel

    import omni.graph.core as og
    import omni.kit.commands
    import usdrt.Sdf

    for graph_path in graph_paths:
        graph_prim = stage.GetPrimAtPath(graph_path)
        if not graph_prim or not graph_prim.IsValid():
            continue

        robot_root = graph_prim.GetPath().GetParentPath().pathString

        exec_source = ""
        for candidate in (
            f"{graph_path}/on_playback_tick.outputs:tick",
            f"{graph_path}/isaac_run_one_simualtion_frame.outputs:step",
            f"{graph_path}/on_playback_tick.outputs:execOut",
        ):
            if _attr_exists(candidate):
                exec_source = candidate
                break
        if not exec_source:
            continue

        timestamp_source = ""
        for candidate in (
            f"{graph_path}/isaac_read_simulation_time.outputs:simulationTime",
            f"{graph_path}/on_playback_tick.outputs:time",
        ):
            if _attr_exists(candidate):
                timestamp_source = candidate
                break

        front_topic = _resolve_input(graph_path, "publish_front_2d_lidar_scan", "topicName", "scan")
        front_frame = _resolve_input(graph_path, "publish_front_2d_lidar_scan", "frameId", "base_link")
        robot_ns = _resolve_input(graph_path, "node_namespace", "value", "")
        front_mount_target = _resolve_rel_target_path(
            stage, f"{graph_path}/front_2d_lidar_render_product.inputs:cameraPrim"
        )
        front_mount_parent = front_mount_target.rsplit("/", 1)[0] if "/" in front_mount_target else ""
        preferred_lidar_path = (
            f"{front_mount_parent}/PhysX_LaserScan" if front_mount_parent else ""
        )

        lidar_path = ""
        if preferred_lidar_path and stage.GetPrimAtPath(preferred_lidar_path).IsValid():
            lidar_path = preferred_lidar_path
        elif preferred_lidar_path:
            created, _lidar_prim = omni.kit.commands.execute(
                "RangeSensorCreateLidar",
                path=preferred_lidar_path,
                parent=None,
                min_range=0.05,
                max_range=20.0,
                draw_points=False,
                draw_lines=False,
                horizontal_fov=360.0,
                vertical_fov=1.0,
                horizontal_resolution=1.0,
                vertical_resolution=1.0,
                rotation_rate=0.0,
                high_lod=False,
                yaw_offset=0.0,
                enable_semantics=False,
            )
            if created and stage.GetPrimAtPath(preferred_lidar_path).IsValid():
                lidar_path = preferred_lidar_path
                stats["mount_lidars_created"] += 1

        if not lidar_path and front_mount_parent:
            mount_candidates = sorted(
                p
                for p in physx_lidar_paths
                if p.startswith(front_mount_parent + "/")
            )
            if mount_candidates:
                lidar_path = mount_candidates[0]

        # Guardrail: never publish from root-level lidar prims.
        if lidar_path and _is_root_level_lidar(lidar_path, robot_root):
            lidar_path = ""

        if not lidar_path:
            continue

        # Raise scan origin above lidar body to avoid self-hit rings while keeping mount pose alignment.
        stats["mount_pose_updates"] += _sync_lidar_pose_from_mount(
            stage,
            lidar_path,
            front_mount_parent,
            z_lift_m=0.08,
        )

        # Force low-cost 2D lidar profile in instant full-scan mode for stable RViz behavior while turning.
        if _set_float_attr(stage, f"{lidar_path}.horizontalResolution", 1.0):
            stats["lidar_param_updates"] += 1
        if _set_float_attr(stage, f"{lidar_path}.verticalFov", 1.0):
            stats["lidar_param_updates"] += 1
        if _set_float_attr(stage, f"{lidar_path}.verticalResolution", 1.0):
            stats["lidar_param_updates"] += 1
        if _set_float_attr(stage, f"{lidar_path}.rotationRate", 0.0):
            stats["lidar_param_updates"] += 1

        # Keep mount-local source enabled for publishing.
        _set_bool_attr(stage, f"{lidar_path}.enabled", True)

        # Disable root-level robot lidars to avoid power drain / accidental misuse.
        for root_lidar in (
            p for p in physx_lidar_paths if _is_root_level_lidar(p, robot_root)
        ):
            if _set_bool_attr(stage, f"{root_lidar}.enabled", False):
                stats["root_lidars_disabled"] += 1

        read_node = "front_2d_lidar_physx_read"
        pub_node = "publish_front_2d_lidar_scan_physx"
        read_node_path = f"{graph_path}/{read_node}"
        pub_node_path = f"{graph_path}/{pub_node}"

        create_nodes = []
        if not stage.GetPrimAtPath(read_node_path).IsValid():
            create_nodes.append((read_node, "isaacsim.sensors.physx.IsaacReadLidarBeams"))
        if not stage.GetPrimAtPath(pub_node_path).IsValid():
            create_nodes.append((pub_node, "isaacsim.ros2.bridge.ROS2PublishLaserScan"))
        if create_nodes:
            og.Controller.edit(graph_path, {og.Controller.Keys.CREATE_NODES: create_nodes})
            stats["created_nodes"] += len(create_nodes)

        values = [
            (f"{read_node_path}.inputs:lidarPrim", [usdrt.Sdf.Path(lidar_path)]),
            (f"{pub_node_path}.inputs:topicName", front_topic),
            (f"{pub_node_path}.inputs:frameId", front_frame),
            (f"{pub_node_path}.inputs:queueSize", 1),
        ]
        if stats["scan_rate_hint_hz"] > 0.0:
            values.append((f"{pub_node_path}.inputs:rotationRate", float(stats["scan_rate_hint_hz"])))
        og.Controller.edit(graph_path, {og.Controller.Keys.SET_VALUES: values})
        stats["set_values"] += len(values)
        # Optional input on some builds; best-effort only.
        _set_bool_attr(stage, f"{pub_node_path}.inputs:fullScan", True)

        connects = [
            (exec_source, f"{read_node_path}.inputs:execIn"),
            (f"{read_node_path}.outputs:execOut", f"{pub_node_path}.inputs:execIn"),
            (f"{read_node_path}.outputs:azimuthRange", f"{pub_node_path}.inputs:azimuthRange"),
            (f"{read_node_path}.outputs:depthRange", f"{pub_node_path}.inputs:depthRange"),
            (f"{read_node_path}.outputs:horizontalFov", f"{pub_node_path}.inputs:horizontalFov"),
            (f"{read_node_path}.outputs:horizontalResolution", f"{pub_node_path}.inputs:horizontalResolution"),
            (f"{read_node_path}.outputs:intensitiesData", f"{pub_node_path}.inputs:intensitiesData"),
            (f"{read_node_path}.outputs:linearDepthData", f"{pub_node_path}.inputs:linearDepthData"),
            (f"{read_node_path}.outputs:numCols", f"{pub_node_path}.inputs:numCols"),
            (f"{read_node_path}.outputs:numRows", f"{pub_node_path}.inputs:numRows"),
        ]
        if timestamp_source:
            connects.append((timestamp_source, f"{pub_node_path}.inputs:timeStamp"))
        if _attr_exists(f"{graph_path}/ros2_context.outputs:context"):
            connects.append((f"{graph_path}/ros2_context.outputs:context", f"{pub_node_path}.inputs:context"))
        if _attr_exists(f"{graph_path}/ros2_qos_profile.outputs:qosProfile"):
            connects.append((f"{graph_path}/ros2_qos_profile.outputs:qosProfile", f"{pub_node_path}.inputs:qosProfile"))
        if _attr_exists(f"{graph_path}/node_namespace.inputs:value"):
            connects.append((f"{graph_path}/node_namespace.inputs:value", f"{pub_node_path}.inputs:nodeNamespace"))
        og.Controller.edit(graph_path, {og.Controller.Keys.CONNECT: connects})
        stats["connections"] += len(connects)

        _set_bool_attr(stage, f"{pub_node_path}.inputs:enabled", True)
        _set_bool_attr(stage, f"{read_node_path}.inputs:enabled", True)

        # Disable any extra ROS2 LaserScan publishers in this graph except our front PhysX publisher.
        for child in graph_prim.GetChildren():
            child_name = child.GetName()
            if child_name == pub_node:
                continue
            node_type_attr = child.GetAttribute("node:type")
            node_type = node_type_attr.Get() if node_type_attr and node_type_attr.HasValue() else ""
            if node_type == "isaacsim.ros2.bridge.ROS2PublishLaserScan":
                if _set_bool_attr(stage, f"{graph_path}/{child_name}.inputs:enabled", False):
                    stats["extra_publishers_disabled"] += 1

        # Keep RTX lidar graph path disabled when PhysX publish path is active.
        for node_name in (
            "publish_front_2d_lidar_scan",
            "publish_back_2d_lidar_scan",
            "publish_front_3d_lidar_scan",
            "front_2d_lidar_render_product",
            "back_2d_lidar_render_product",
            "front_3d_lidar_render_product",
        ):
            if _set_bool_attr(stage, f"{graph_path}/{node_name}.inputs:enabled", False):
                stats["rtx_nodes_disabled"] += 1

        stats["source_bindings"].append(
            {
                "graph_path": graph_path,
                "namespace": robot_ns,
                "lidar_path": lidar_path,
                "mode": "instant_fullscan",
                "publish_rate_hint_hz": float(stats["scan_rate_hint_hz"]),
            }
        )
        stats["graphs_configured"] += 1

    stats["applied"] = stats["graphs_configured"] > 0
    if not stats["applied"] and not stats["reason"]:
        stats["reason"] = "no front-mount PhysX lidar source resolved for ros_lidars graphs"
    return stats


def _configure_physx_laserscan(stage) -> dict[str, object]:
    stats: dict[str, object] = {
        "applied": False,
        "reason": "",
        "rtx_nodes_disabled": 0,
        "created_lidar": False,
        "created_nodes": 0,
        "set_values": 0,
        "connections": 0,
    }

    graph_path = "/World/Nova_Carter_ROS/ros_lidars"
    graph_prim = stage.GetPrimAtPath(graph_path)
    if not graph_prim or not graph_prim.IsValid():
        stats["reason"] = "ros_lidars graph not found"
        return stats

    front_rtx_pub = stage.GetPrimAtPath(graph_path + "/publish_front_2d_lidar_scan")
    if not front_rtx_pub or not front_rtx_pub.IsValid():
        stats["reason"] = "front RTX scan publisher not found"
        return stats

    node_type_attr = front_rtx_pub.GetAttribute("node:type")
    node_type = node_type_attr.Get() if node_type_attr and node_type_attr.HasValue() else ""
    if node_type != "isaacsim.ros2.bridge.ROS2RtxLidarHelper":
        stats["reason"] = f"front scan publisher is not RTX helper ({node_type or 'unknown'})"
        return stats

    # Respect scene-authored lidar-off variants: do not auto-create PhysX scan if front scan is disabled.
    front_scan_enabled_attr = front_rtx_pub.GetAttribute("inputs:enabled")
    if front_scan_enabled_attr and front_scan_enabled_attr.HasValue():
        if not bool(front_scan_enabled_attr.Get()):
            stats["reason"] = "front scan publisher is disabled in scene"
            return stats

    front_rtx_rp = stage.GetPrimAtPath(graph_path + "/front_2d_lidar_render_product")
    if front_rtx_rp and front_rtx_rp.IsValid():
        front_rp_enabled_attr = front_rtx_rp.GetAttribute("inputs:enabled")
        if front_rp_enabled_attr and front_rp_enabled_attr.HasValue():
            if not bool(front_rp_enabled_attr.Get()):
                stats["reason"] = "front lidar render product is disabled in scene"
                return stats

    # Disable authored RTX scan render path (front scan only).
    if _set_bool_attr(stage, graph_path + "/front_2d_lidar_render_product.inputs:enabled", False):
        stats["rtx_nodes_disabled"] += 1
    if _set_bool_attr(stage, graph_path + "/publish_front_2d_lidar_scan.inputs:enabled", False):
        stats["rtx_nodes_disabled"] += 1

    # Create a dedicated low-cost PhysX lidar sensor attached to the same front lidar mount.
    lidar_path = "/World/Nova_Carter_ROS/chassis_link/sensors/front_RPLidar/PhysX_LaserScan"
    if not stage.GetPrimAtPath(lidar_path).IsValid():
        import omni.kit.commands

        created, _lidar_prim = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path=lidar_path,
            parent=None,
            min_range=0.05,
            max_range=20.0,
            draw_points=False,
            draw_lines=False,
            horizontal_fov=360.0,
            vertical_fov=1.0,
            horizontal_resolution=1.0,
            vertical_resolution=1.0,
            rotation_rate=10.0,
            high_lod=False,
            yaw_offset=0.0,
            enable_semantics=False,
        )
        if not created:
            stats["reason"] = "failed to create PhysX lidar prim"
            return stats
        stats["created_lidar"] = True

    # Build graph nodes for PhysX -> ROS2 LaserScan.
    import omni.graph.core as og
    import usdrt.Sdf

    read_node = "front_2d_lidar_physx_read"
    pub_node = "publish_front_2d_lidar_scan_physx"
    read_node_path = f"{graph_path}/{read_node}"
    pub_node_path = f"{graph_path}/{pub_node}"
    create_nodes = []
    if not stage.GetPrimAtPath(read_node_path).IsValid():
        create_nodes.append((read_node, "isaacsim.sensors.physx.IsaacReadLidarBeams"))
    if not stage.GetPrimAtPath(pub_node_path).IsValid():
        create_nodes.append((pub_node, "isaacsim.ros2.bridge.ROS2PublishLaserScan"))

    if create_nodes:
        og.Controller.edit(graph_path, {og.Controller.Keys.CREATE_NODES: create_nodes})
        stats["created_nodes"] = len(create_nodes)

    # Configure values (safe to reapply).
    og.Controller.edit(
        graph_path,
        {
            og.Controller.Keys.SET_VALUES: [
                (f"{read_node_path}.inputs:lidarPrim", [usdrt.Sdf.Path(lidar_path)]),
                (f"{pub_node_path}.inputs:topicName", "scan"),
                (f"{pub_node_path}.inputs:frameId", "front_2d_lidar"),
                (f"{pub_node_path}.inputs:queueSize", 1),
            ]
        },
    )
    stats["set_values"] = 4
    _set_bool_attr(stage, f"{pub_node_path}.inputs:fullScan", False)

    # Connect execution + data + ROS context/qos.
    connects = [
        (f"{graph_path}/isaac_run_one_simualtion_frame.outputs:step", f"{read_node_path}.inputs:execIn"),
        (f"{read_node_path}.outputs:execOut", f"{pub_node_path}.inputs:execIn"),
        (f"{read_node_path}.outputs:azimuthRange", f"{pub_node_path}.inputs:azimuthRange"),
        (f"{read_node_path}.outputs:depthRange", f"{pub_node_path}.inputs:depthRange"),
        (f"{read_node_path}.outputs:horizontalFov", f"{pub_node_path}.inputs:horizontalFov"),
        (f"{read_node_path}.outputs:horizontalResolution", f"{pub_node_path}.inputs:horizontalResolution"),
        (f"{read_node_path}.outputs:intensitiesData", f"{pub_node_path}.inputs:intensitiesData"),
        (f"{read_node_path}.outputs:linearDepthData", f"{pub_node_path}.inputs:linearDepthData"),
        (f"{read_node_path}.outputs:numCols", f"{pub_node_path}.inputs:numCols"),
        (f"{read_node_path}.outputs:numRows", f"{pub_node_path}.inputs:numRows"),
        (f"{read_node_path}.outputs:rotationRate", f"{pub_node_path}.inputs:rotationRate"),
        (f"{graph_path}/on_playback_tick.outputs:time", f"{pub_node_path}.inputs:timeStamp"),
        (f"{graph_path}/ros2_context.outputs:context", f"{pub_node_path}.inputs:context"),
        (f"{graph_path}/ros2_qos_profile.outputs:qosProfile", f"{pub_node_path}.inputs:qosProfile"),
        (f"{graph_path}/node_namespace.inputs:value", f"{pub_node_path}.inputs:nodeNamespace"),
    ]
    og.Controller.edit(graph_path, {og.Controller.Keys.CONNECT: connects})
    stats["connections"] = len(connects)

    stats["applied"] = True
    return stats


# ---------------------------
# Main
# ---------------------------

def main() -> None:
    args = parse_args()

    # Normalize conflicting flags
    if args.keep_viewport_updates:
        args.disable_viewport_updates = False
    if args.disable_ros2:
        args.enable_ros2 = False
    args.ros_namespace = args.ros_namespace.strip().strip("/")

    effective_min_frame_rate = args.target_sim_hz if args.target_sim_hz > 0 else args.min_frame_rate

    # Configure ROS2 environment (only if requested)
    if args.enable_ros2:
        os.environ.setdefault("ROS_DISTRO", "jazzy")
        os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")

        # Try to prefer Isaac Sim's internal ROS libs if ISAAC_PATH is set
        isaac_root = None
        for env_key in ("ISAAC_PATH", "ISAACSIM_PATH"):
            if os.environ.get(env_key):
                isaac_root = Path(os.environ[env_key])
                break

        if isaac_root:
            jazzy_root = isaac_root / "exts" / "isaacsim.ros2.bridge" / "jazzy"
            jazzy_lib = jazzy_root / "lib"
            jazzy_rclpy = jazzy_root / "rclpy"

            if jazzy_lib.is_dir():
                ld_path = os.environ.get("LD_LIBRARY_PATH", "")
                parts = ld_path.split(":") if ld_path else []
                if str(jazzy_lib) not in parts:
                    os.environ["LD_LIBRARY_PATH"] = (ld_path + ":" if ld_path else "") + str(jazzy_lib)

            if jazzy_rclpy.is_dir() and str(jazzy_rclpy) not in sys.path:
                sys.path.insert(0, str(jazzy_rclpy))

    # Validate USD early (so we don't crash Kit startup with a bad path)
    open_usd = ""
    if args.usd_path:
        if not os.path.isfile(args.usd_path):
            print(f"[fast_isaac_sim] WARNING: USD not found: {args.usd_path}")
        else:
            open_usd = args.usd_path

    # Hospital family scenes: default to DLSS Performance unless user explicitly set AA/DLSS flags.
    usd_name = os.path.basename(open_usd or args.usd_path or "").lower()
    is_hospital_family_requested = "hospital_experiment" in usd_name
    aa_mode_explicit = any(
        token == "--aa-mode" or token.startswith("--aa-mode=")
        for token in sys.argv[1:]
    )
    dlss_mode_explicit = any(
        token == "--dlss-exec-mode" or token.startswith("--dlss-exec-mode=")
        for token in sys.argv[1:]
    )
    if is_hospital_family_requested:
        if not aa_mode_explicit:
            args.aa_mode = 3
        if not dlss_mode_explicit:
            args.dlss_exec_mode = 0
        print(
            "[fast_isaac_sim] Hospital render profile:"
            f" aa_mode={args.aa_mode}, dlss_exec_mode={args.dlss_exec_mode}"
            " (defaulting to DLSS Performance unless explicitly overridden)."
        )

    # Build experience selection
    experience = args.experience
    exp_path = os.environ.get("EXP_PATH", "")

    if not experience:
        # Default python workflow kit (lightest) unless user requests full/fabric
        # This follows SimulationApp docs default preference order.
        base_python = _kit_from_exp_path(exp_path, "isaacsim.exp.base.python.kit")
        omni_python = _kit_from_exp_path(exp_path, "omni.isaac.sim.python.kit")
        full = _kit_from_exp_path(exp_path, "isaacsim.exp.full.kit")
        full_fabric = _kit_from_exp_path(exp_path, "isaacsim.exp.full.fabric.kit")

        if args.use_fabric_experience and full_fabric:
            experience = full_fabric
        elif args.use_full_experience and full:
            experience = full
        else:
            experience = base_python or omni_python or full or full_fabric or ""

    # Build extra args (applied at Kit startup)
    extra_args: list[str] = []

    # CPU thread knobs (documented by Isaac Sim 5.1 performance handbook)
    if args.set_thread_knobs:
        extra_args += [
            _as_cli_setting("/plugins/carb.tasking.plugin/threadCount", int(args.limit_cpu_threads)),
            _as_cli_setting("/plugins/omni.tbb.globalcontrol/maxThreadCount", int(args.limit_cpu_threads)),
            _as_cli_setting("/persistent/physics/numThreads", int(args.limit_cpu_threads)),
        ]

    # Texture streaming
    if args.disable_texture_streaming:
        extra_args.append(_as_cli_setting("/rtx-transient/resourcemanager/texturestreaming/enabled", False))

    # DLSSG (frame generation) toggle (default OFF)
    extra_args.append(_as_cli_setting("/rtx-transient/dlssg/enabled", bool(args.enable_dlssg)))

    # If user wants a target sim rate, set the rate-limit knobs at startup too
    # (also set again at runtime for safety).
    if args.target_sim_hz > 0:
        hz = int(round(args.target_sim_hz))
        extra_args += [
            _as_cli_setting("/app/runLoops/main/rateLimitEnabled", True),
            _as_cli_setting("/app/runLoops/main/rateLimitFrequency", hz),
            _as_cli_setting("/persistent/simulation/minFrameRate", hz),
        ]
    else:
        # Run as fast as possible
        extra_args += [
            _as_cli_setting("/app/runLoops/main/rateLimitEnabled", False),
            _as_cli_setting("/app/runLoops/present/rateLimitEnabled", False),
            _as_cli_setting("/app/runLoops/rendering_0/rateLimitEnabled", False),
        ]

    # Enable ROS2 bridge at startup if requested (so node types exist when stage loads)
    if args.enable_ros2:
        extra_args += ["--enable", "isaacsim.ros2.bridge"]

    # SimulationApp config (documented keys)
    want_render = (not args.headless) or args.render_headless
    app_config = {
        "headless": bool(args.headless),
        "limit_cpu_threads": int(args.limit_cpu_threads),
        "open_usd": open_usd,  # empty means "don't auto-open"
        "physics_gpu": int(args.physics_gpu),
        "renderer": args.renderer if want_render else args.renderer,  # renderer still selected, but we can skip rendering in step()
        "anti_aliasing": int(args.aa_mode),
        "multi_gpu": bool(args.multi_gpu),
        "max_gpu_count": int(args.max_gpu_count) if args.multi_gpu else 1,
        "extra_args": extra_args,
        # Reduce default viewport allocation cost (even headless can create viewports)
        "width": 640,
        "height": 480,
        "window_width": 640,
        "window_height": 480,
    }

    if args.headless and args.disable_viewport_updates:
        app_config["disable_viewport_updates"] = True

    print("[fast_isaac_sim] Launch config summary:")
    print(f"  experience={experience or '(default)'}")
    print(f"  headless={args.headless}, render_headless={args.render_headless}, render_every={args.render_every}")
    print(
        "  "
        f"physics_dt={args.physics_step:.6f}s, "
        f"min_frame_rate={args.min_frame_rate}Hz, "
        f"target_sim_hz={args.target_sim_hz}, "
        f"effective_min_frame_rate={effective_min_frame_rate}Hz"
    )
    print(f"  gpu_dynamics={'OFF' if args.disable_gpu_dynamics else 'ON'}, physics_gpu={args.physics_gpu}")
    print(f"  cpu_threads={args.limit_cpu_threads}, set_thread_knobs={args.set_thread_knobs}")
    print(f"  multi_gpu={args.multi_gpu}, max_gpu_count={args.max_gpu_count}")
    print(f"  aa_mode={args.aa_mode}, dlss_exec_mode={args.dlss_exec_mode}, dlssg={args.enable_dlssg}")
    print(f"  disable_texture_streaming={args.disable_texture_streaming}")
    print(f"  optimize_render={args.optimize_render}")
    print(f"  ros2_bridge={args.enable_ros2}")
    print(f"  ros_namespace={args.ros_namespace or '(disabled)'}")
    print(f"  namespace_tf_topics={args.namespace_tf_topics}")
    print(f"  ros_render_guard={not args.disable_ros_render_guard}")
    physx_scan_rate_hint_hz = _compute_physx_scan_rate_hint_hz(args, want_render)
    print(f"  physx_scan_rate_hint_hz={physx_scan_rate_hint_hz:.2f}")
    print(
        "  "
        f"ros_ready_check={not args.skip_ros_ready_check}, "
        f"ready_topic={args.ros_ready_topic}, "
        f"ready_timeout={args.ros_ready_timeout:.1f}s"
    )
    print(
        "  "
        f"robot_profile={args.robot_profile}, "
        f"profile_camera={args.profile_camera_width}x{args.profile_camera_height}"
    )
    if args.enable_ros2:
        print(f"  ros_distro={os.environ.get('ROS_DISTRO', '(unset)')}")
        print(f"  rmw_implementation={os.environ.get('RMW_IMPLEMENTATION', '(unset)')}")
        print(f"  ros_domain_id={os.environ.get('ROS_DOMAIN_ID', '(default=0)')}")
        print(f"  ros_localhost_only={os.environ.get('ROS_LOCALHOST_ONLY', '(unset)')}")
        if args.ros_namespace:
            tf_topic = f"/{args.ros_namespace}/tf" if args.namespace_tf_topics else "/tf"
            print(f"  expected_tf_topic={tf_topic}")
    print(f"  auto_physx_laserscan={not args.disable_physx_laserscan}")
    print(f"  usd={open_usd or '(empty stage)'}")

    simulation_app = SimulationApp(app_config, experience=experience)

    # Import Isaac Sim modules only AFTER SimulationApp
    from isaacsim.core.api import World
    import omni.kit.app
    import omni.timeline
    import omni.usd

    usd_ctx = omni.usd.get_context()

    # Explicitly open requested USD to guarantee the stage is loaded before normalization.
    if open_usd:
        current_stage_url = usd_ctx.get_stage_url() or ""
        current_matches = False
        try:
            current_matches = Path(current_stage_url).resolve() == Path(open_usd).resolve()
        except Exception:
            current_matches = current_stage_url == open_usd

        if not current_matches:
            if not usd_ctx.open_stage(open_usd):
                print(f"[fast_isaac_sim] WARNING: Failed to open stage via omni.usd: {open_usd}")
            else:
                for _ in range(180):
                    simulation_app.update()

    stage = usd_ctx.get_stage()
    stage_url = usd_ctx.get_stage_url() or "(in-memory stage)"
    is_hospital_scene = "hospital_experiment.usda" in ((args.usd_path or "") + " " + str(stage_url))
    if stage:
        print(f"[fast_isaac_sim] Startup phase: stage loaded ({stage_url})")
    else:
        print("[fast_isaac_sim] WARNING: Startup phase: stage not available after load")

    # Ensure ROS2 bridge enabled (best-effort; safe if already enabled)
    if args.enable_ros2:
        try:
            ext_manager = omni.kit.app.get_app().get_extension_manager()
            ext_manager.set_extension_enabled("isaacsim.ros2.bridge", True)
        except Exception as exc:
            print(f"[fast_isaac_sim] WARNING: Could not enable isaacsim.ros2.bridge: {exc}")

    # Apply runtime carb settings (documented)
    try:
        settings = carb.settings.get_settings()

        # DLSS mode: documented as /rtx/post/dlss/execMode (0..3)
        settings.set_int("/rtx/post/dlss/execMode", int(args.dlss_exec_mode))

        # AA mode: /rtx/post/aa/op (common: 0 none, 1 TAA, 3 DLSS)
        settings.set_int("/rtx/post/aa/op", int(args.aa_mode))

        # Keep Kit rate-limits in sync with requested target for compatibility with existing workflows.
        if args.target_sim_hz > 0:
            hz = int(round(args.target_sim_hz))
            settings.set_bool("/app/runLoops/main/rateLimitEnabled", True)
            settings.set_int("/app/runLoops/main/rateLimitFrequency", hz)
            settings.set_int("/persistent/simulation/minFrameRate", hz)
        else:
            settings.set_bool("/app/runLoops/main/rateLimitEnabled", False)
            settings.set_bool("/app/runLoops/present/rateLimitEnabled", False)
            settings.set_bool("/app/runLoops/rendering_0/rateLimitEnabled", False)
            settings.set_int("/persistent/simulation/minFrameRate", int(round(args.min_frame_rate)))

        # Optional render feature disables (documented to improve perf)
        if want_render and args.optimize_render:
            settings.set_bool("/rtx/reflections/enabled", False)
            settings.set_bool("/rtx/ambientOcclusion/enabled", False)
            settings.set_bool("/rtx/directLighting/sampledLighting/enabled", False)
            settings.set_bool("/rtx/indirectDiffuse/enabled", False)

    except Exception as exc:
        print(f"[fast_isaac_sim] WARNING: carb settings update failed: {exc}")

    # Create world (backend defaults to numpy; torch is fine too if you use tensor APIs)
    world = World(stage_units_in_meters=1.0, backend="numpy")

    # Documented performance knobs (5.1 handbook)
    try:
        world.set_physics_step_size(args.physics_step)
    except Exception:
        # Fallback to older API style if present
        try:
            world.set_simulation_dt(physics_dt=args.physics_step, rendering_dt=args.physics_step)
        except Exception as exc:
            print(f"[fast_isaac_sim] WARNING: Could not set physics dt: {exc}")

    try:
        world.set_min_simulation_frame_rate(float(effective_min_frame_rate))
    except Exception as exc:
        print(f"[fast_isaac_sim] WARNING: Could not set min sim frame rate on World: {exc}")

    try:
        world.set_gpu_dynamics_enabled(not args.disable_gpu_dynamics)
    except Exception:
        # Fallback
        try:
            physics_context = world.get_physics_context()
            physics_context.enable_gpu_dynamics(flag=not args.disable_gpu_dynamics)
        except Exception as exc:
            print(f"[fast_isaac_sim] WARNING: Could not set GPU dynamics: {exc}")

    if not args.no_ground_plane:
        world.scene.add_default_ground_plane()

    if stage and is_hospital_scene:
        # Cloud profile: run all robot cameras at 640x480.
        hospital_cam_stats = _override_hospital_camera_resolution(stage, width=640, height=480)
        print(
            "[fast_isaac_sim] Hospital camera override (pre-reset):"
            f" camera_nodes_seen={hospital_cam_stats['camera_nodes_seen']},"
            f" camera_size_updates={hospital_cam_stats['camera_size_updates']},"
            " size=640x480"
        )

    if stage and args.robot_profile != "scene-default":
        profile_stats = _apply_robot_profile(
            stage,
            args.robot_profile,
            camera_width=args.profile_camera_width,
            camera_height=args.profile_camera_height,
        )
        print(
            "[fast_isaac_sim] Robot profile applied (pre-reset):"
            f" profile={args.robot_profile},"
            f" camera_nodes_seen={profile_stats['camera_nodes_seen']},"
            f" camera_enable_updates={profile_stats['camera_enable_updates']},"
            f" camera_size_updates={profile_stats['camera_size_updates']},"
            f" lidar_enable_updates={profile_stats['lidar_enable_updates']}"
        )

    if is_hospital_scene:
        # Keep authored RTX lidar graph for hospital scene and skip auto PhysX swap.
        args.disable_physx_laserscan = True
        print("[fast_isaac_sim] Hospital lidar profile: authored RTX 2D lidar (front/back) enabled in scene")

    playback_tick_ros_graphs: list[str] = []
    if args.enable_ros2:
        if stage:
            if not args.disable_physx_laserscan:
                try:
                    physx_multi_stats = _configure_physx_laserscan_multi(
                        stage,
                        scan_rate_hint_hz=physx_scan_rate_hint_hz,
                    )
                    if physx_multi_stats.get("applied"):
                        print(
                            "[fast_isaac_sim] PhysX LaserScan configured (multi-robot):"
                            f" graphs_seen={physx_multi_stats['graphs_seen']},"
                            f" graphs_configured={physx_multi_stats['graphs_configured']},"
                            f" physx_lidars_found={physx_multi_stats['physx_lidars_found']},"
                            f" mount_lidars_created={physx_multi_stats['mount_lidars_created']},"
                            f" mount_pose_updates={physx_multi_stats['mount_pose_updates']},"
                            f" lidar_param_updates={physx_multi_stats['lidar_param_updates']},"
                            f" root_lidars_disabled={physx_multi_stats['root_lidars_disabled']},"
                            f" extra_publishers_disabled={physx_multi_stats['extra_publishers_disabled']},"
                            f" created_nodes={physx_multi_stats['created_nodes']},"
                            f" rtx_nodes_disabled={physx_multi_stats['rtx_nodes_disabled']},"
                            f" connections={physx_multi_stats['connections']},"
                            f" scan_rate_hint_hz={physx_multi_stats['scan_rate_hint_hz']:.2f}"
                        )
                        for binding in physx_multi_stats.get("source_bindings", []):
                            if isinstance(binding, dict):
                                ns_label = str(binding.get("namespace", "")).strip() or "(scene-default)"
                                print(
                                    "[fast_isaac_sim]   PhysX source binding:"
                                    f" namespace={ns_label},"
                                    f" source={binding.get('lidar_path', '')},"
                                    f" graph={binding.get('graph_path', '')},"
                                    f" mode={binding.get('mode', 'instant_fullscan')},"
                                    f" publish_rate_hint_hz={float(binding.get('publish_rate_hint_hz', 0.0)):.2f}"
                                )
                            else:
                                print(f"[fast_isaac_sim]   PhysX source binding: {binding}")
                    else:
                        physx_stats = _configure_physx_laserscan(stage)
                        if physx_stats.get("applied"):
                            print(
                                "[fast_isaac_sim] PhysX LaserScan configured:"
                                f" rtx_nodes_disabled={physx_stats['rtx_nodes_disabled']},"
                                f" created_lidar={physx_stats['created_lidar']},"
                                f" created_nodes={physx_stats['created_nodes']},"
                                f" connections={physx_stats['connections']}"
                            )
                        else:
                            reason = physx_multi_stats.get("reason") or physx_stats.get("reason", "unknown reason")
                            print(f"[fast_isaac_sim] PhysX LaserScan auto-swap skipped: {reason}")
                except Exception as exc:
                    print(f"[fast_isaac_sim] WARNING: PhysX LaserScan auto-swap failed: {exc}")

            if args.ros_namespace:
                ns_stats = _normalize_ros2_namespace(
                    stage,
                    args.ros_namespace,
                    namespace_tf_topics=args.namespace_tf_topics,
                )
                print(
                    "[fast_isaac_sim] ROS2 namespace normalization:"
                    f" bridge_nodes={ns_stats['bridge_nodes']},"
                    f" namespace_updates={ns_stats['namespace_updates']},"
                    f" topic_updates={ns_stats['topic_updates']},"
                    f" frame_updates={ns_stats['frame_updates']},"
                    f" tf_tree_nodes={ns_stats['tf_tree_nodes']}"
                )
                if ns_stats["tf_tree_nodes"] > 0:
                    print(
                        "[fast_isaac_sim] NOTE: ROS2PublishTransformTree frames are generated from USD prim names. "
                        "Frame-id prefixing applies to ROS message frame-id fields and raw TF nodes, but not to "
                        "all frames emitted from transform-tree target prims."
                    )
            else:
                print("[fast_isaac_sim] ROS2 namespace normalization disabled (using scene defaults).")

            tf_publishers = _find_tf_publishers(stage)
            if tf_publishers:
                print(f"[fast_isaac_sim] TF publishers detected: {len(tf_publishers)}")
            else:
                print(
                    "[fast_isaac_sim] WARNING: No TF publisher nodes detected in the stage. "
                    "Expected ROS2 transform tree publishers for /tf or /tf_static."
                )

            playback_tick_ros_graphs = _find_playback_tick_driven_ros_graphs(stage)
            if playback_tick_ros_graphs:
                print(
                    "[fast_isaac_sim] ROS playback-tick graphs detected:"
                    f" count={len(playback_tick_ros_graphs)}"
                )
        else:
            print("[fast_isaac_sim] WARNING: No USD stage available for ROS2 namespace normalization.")

    world.reset()
    print("[fast_isaac_sim] Startup phase: world reset complete")

    post_stage = usd_ctx.get_stage()
    if post_stage and args.robot_profile != "scene-default":
        post_profile_stats = _apply_robot_profile(
            post_stage,
            args.robot_profile,
            camera_width=args.profile_camera_width,
            camera_height=args.profile_camera_height,
        )
        print(
            "[fast_isaac_sim] Robot profile applied (post-reset):"
            f" profile={args.robot_profile},"
            f" camera_enable_updates={post_profile_stats['camera_enable_updates']},"
            f" camera_size_updates={post_profile_stats['camera_size_updates']},"
            f" lidar_enable_updates={post_profile_stats['lidar_enable_updates']}"
        )

    if post_stage and is_hospital_scene:
        hospital_cam_post_stats = _override_hospital_camera_resolution(post_stage, width=640, height=480)
        print(
            "[fast_isaac_sim] Hospital camera override (post-reset):"
            f" camera_nodes_seen={hospital_cam_post_stats['camera_nodes_seen']},"
            f" camera_size_updates={hospital_cam_post_stats['camera_size_updates']},"
            " size=640x480"
        )

    # Some ROS2 nodes can appear only after world reset; apply namespace normalization again.
    if args.enable_ros2 and args.ros_namespace:
        if post_stage:
            post_stats = _normalize_ros2_namespace(
                post_stage,
                args.ros_namespace,
                namespace_tf_topics=args.namespace_tf_topics,
            )
            print(
                "[fast_isaac_sim] ROS2 namespace post-reset:"
                f" bridge_nodes={post_stats['bridge_nodes']},"
                f" namespace_updates={post_stats['namespace_updates']},"
                f" topic_updates={post_stats['topic_updates']},"
                f" frame_updates={post_stats['frame_updates']},"
                f" tf_tree_nodes={post_stats['tf_tree_nodes']}"
            )
        else:
            print("[fast_isaac_sim] WARNING: No USD stage available for post-reset namespace normalization.")

    if args.enable_ros2 and post_stage:
        playback_tick_ros_graphs = _find_playback_tick_driven_ros_graphs(post_stage)

    if args.enable_ros2 and playback_tick_ros_graphs and (not args.disable_ros_render_guard):
        if not want_render:
            preview = ", ".join(playback_tick_ros_graphs[:3])
            remainder = "" if len(playback_tick_ros_graphs) <= 3 else ", ..."
            print(
                "[fast_isaac_sim] ERROR: ROS render guard triggered. Playback-tick ROS graphs were detected, "
                "but rendering is disabled for all steps. In this scene, ROS publishers can remain silent until "
                "timeline time advances on rendered steps."
            )
            print(f"[fast_isaac_sim]   graphs={preview}{remainder}")
            print(
                "[fast_isaac_sim]   fix: relaunch with --render-headless "
                "(and set --render-every as needed, for example --render-every 2)."
            )
            simulation_app.close()
            raise SystemExit(2)

        timeline = omni.timeline.get_timeline_interface()
        probe_steps = max(6, min(args.render_every + 2, 12))
        probe_rendered = 0
        t_before = float(timeline.get_current_time())
        for probe_step in range(probe_steps):
            do_render = _should_render_step(probe_step, want_render, args.render_every)
            world.step(render=do_render)
            if do_render:
                probe_rendered += 1
        t_after = float(timeline.get_current_time())
        if t_after <= t_before + 1e-9:
            print(
                "[fast_isaac_sim] ERROR: ROS render guard triggered. Timeline time did not advance during "
                f"startup probe (before={t_before:.6f}, after={t_after:.6f}) with playback-tick ROS graphs present."
            )
            print(
                "[fast_isaac_sim]   fix: use effective rendered stepping "
                "(--render-headless and a practical --render-every value)."
            )
            simulation_app.close()
            raise SystemExit(2)
        print(
            "[fast_isaac_sim] ROS render guard passed:"
            f" timeline_before={t_before:.6f}, timeline_after={t_after:.6f},"
            f" probe_steps={probe_steps}, probe_rendered_steps={probe_rendered}"
        )

    pace_period = (1.0 / args.target_sim_hz) if args.target_sim_hz > 0 else 0.0
    next_step_deadline = time.perf_counter()

    def _step_with_policy(step_index: int) -> bool:
        nonlocal next_step_deadline
        if pace_period > 0.0:
            now = time.perf_counter()
            sleep_for = next_step_deadline - now
            if sleep_for > 0.0:
                time.sleep(sleep_for)

        do_render = _should_render_step(step_index, want_render, args.render_every)
        world.step(render=do_render)

        if pace_period > 0.0:
            next_step_deadline += pace_period
            now = time.perf_counter()
            if next_step_deadline < (now - pace_period):
                next_step_deadline = now
        return do_render

    print("[fast_isaac_sim] Startup phase: loop started")
    if args.enable_ros2 and (not args.skip_ros_ready_check):
        ready_topic = args.ros_ready_topic.strip() or "/clock"
        if not ready_topic.startswith("/"):
            ready_topic = "/" + ready_topic
        ready_step = 0
        ready_rendered = 0
        ready_t0 = time.perf_counter()
        next_step_deadline = ready_t0

        def _ready_step_once() -> None:
            nonlocal ready_step, ready_rendered
            do_render = _step_with_policy(ready_step)
            ready_step += 1
            if do_render:
                ready_rendered += 1

        ready_ok, ready_detail = _wait_for_ros_topic_sample(
            topic=ready_topic,
            timeout_s=float(args.ros_ready_timeout),
            simulation_app=simulation_app,
            step_once=_ready_step_once,
        )
        ready_wall = max(time.perf_counter() - ready_t0, 0.0)
        if not ready_ok:
            print(
                "[fast_isaac_sim] ERROR: ROS startup readiness failed:"
                f" topic={ready_topic}, detail={ready_detail},"
                f" timeout={args.ros_ready_timeout:.1f}s, steps={ready_step}, rendered_steps={ready_rendered}"
            )
            print(
                "[fast_isaac_sim]   hint: verify ROS_DOMAIN_ID/RMW matches your ROS terminal. "
                "If this scene uses playback-tick ROS graphs, ensure rendered steps are active."
            )
            simulation_app.close()
            raise SystemExit(3)
        print(
            "[fast_isaac_sim] Startup phase: ready topic received:"
            f" topic={ready_topic}, type={ready_detail},"
            f" steps={ready_step}, rendered_steps={ready_rendered}, wall={ready_wall:.2f}s"
        )
    elif args.enable_ros2:
        print("[fast_isaac_sim] Startup phase: ROS readiness check skipped by flag")

    # Main loop (benchmark/reporting phase)
    step = 0
    rendered_total = 0
    window_rendered = 0
    t0 = time.perf_counter()
    next_step_deadline = t0

    print("[fast_isaac_sim] Starting simulation loop...")
    while simulation_app.is_running():
        if args.max_steps > 0 and step >= args.max_steps:
            break

        do_render = _step_with_policy(step)
        step += 1
        if do_render:
            rendered_total += 1
            window_rendered += 1

        if args.fps_report_interval > 0 and (step % args.fps_report_interval) == 0:
            dt = max(time.perf_counter() - t0, 1e-9)
            fps = step / dt
            interval_ratio = (window_rendered / float(args.fps_report_interval)) * 100.0
            print(
                "[fast_isaac_sim] "
                f"steps={step}  wall={dt:.2f}s  approx_step_fps={fps:.1f}  "
                f"rendered_steps={window_rendered}/{args.fps_report_interval} ({interval_ratio:.1f}%)"
            )
            window_rendered = 0

    wall = max(time.perf_counter() - t0, 1e-9)
    avg = step / wall
    total_ratio = (rendered_total / float(step)) * 100.0 if step > 0 else 0.0
    print("[fast_isaac_sim] Finished")
    print(f"  steps={step}")
    print(f"  wall={wall:.2f}s")
    print(f"  avg_step_fps={avg:.1f}")
    print(f"  rendered_steps={rendered_total} ({total_ratio:.1f}%)")

    simulation_app.close()


if __name__ == "__main__":
    main()
