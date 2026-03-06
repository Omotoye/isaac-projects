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
import sys
import time
from pathlib import Path
from typing import Iterable

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
            "Optional namespace for non-TF/non-clock ROS 2 topics and frame IDs. "
            "Empty (default) leaves scene topics/frame IDs unchanged."
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


def _normalize_ros2_namespace(stage, namespace: str) -> dict[str, int]:
    stats = {
        "bridge_nodes": 0,
        "namespace_updates": 0,
        "topic_updates": 0,
        "frame_updates": 0,
    }

    for prim, node_type in _iter_ros2_bridge_nodes(stage):
        stats["bridge_nodes"] += 1
        is_tf = _is_tf_bridge_type(node_type)
        is_clock = _is_clock_bridge_type(node_type)

        node_ns_attr = prim.GetAttribute("inputs:nodeNamespace")
        if node_ns_attr:
            desired_ns = "" if (is_tf or is_clock or not namespace) else namespace
            current_ns = node_ns_attr.Get()
            current_ns = "" if current_ns is None else str(current_ns)
            if current_ns != desired_ns:
                node_ns_attr.Set(desired_ns)
                stats["namespace_updates"] += 1

        for attr in prim.GetAttributes():
            name = attr.GetName()
            if not name.startswith("inputs:"):
                continue
            low_name = name.lower()
            current = attr.Get()

            if "topicname" in low_name:
                desired_topic = current
                if is_clock:
                    desired_topic = "/clock"
                elif current is not None:
                    current_topic = str(current)
                    if is_tf:
                        if current_topic == "tf":
                            desired_topic = "/tf"
                        elif current_topic == "tf_static":
                            desired_topic = "/tf_static"
                    elif namespace and current_topic:
                        if current_topic in {"tf", "tf_static", "clock", "/tf", "/tf_static", "/clock"}:
                            desired_topic = "/" + current_topic.lstrip("/")
                        else:
                            topic_rel = current_topic.lstrip("/")
                            ns_prefix = f"{namespace}/"
                            if topic_rel.startswith(ns_prefix):
                                topic_rel = topic_rel[len(ns_prefix):]
                            desired_topic = topic_rel
                if desired_topic != current:
                    attr.Set(desired_topic)
                    stats["topic_updates"] += 1
                continue

            if current is None:
                continue
            if "frame" in low_name and ("id" in low_name or low_name.endswith("frame")):
                current_frame = str(current)
                if not current_frame:
                    continue
                desired_frame = _normalize_frame_id(current_frame, namespace)
                if desired_frame != current_frame:
                    attr.Set(desired_frame)
                    stats["frame_updates"] += 1

    return stats


def _find_tf_publishers(stage) -> list[tuple[str, str]]:
    tf_publishers: list[tuple[str, str]] = []
    for prim, node_type in _iter_ros2_bridge_nodes(stage):
        if _is_tf_bridge_type(node_type):
            tf_publishers.append((prim.GetPath().pathString, node_type))
    return tf_publishers


def _set_bool_attr(stage, attr_path: str, value: bool) -> bool:
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
                (f"{pub_node_path}.inputs:queueSize", 5),
            ]
        },
    )
    stats["set_values"] = 4

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
    print(f"  auto_physx_laserscan={not args.disable_physx_laserscan}")
    print(f"  usd={open_usd or '(empty stage)'}")

    simulation_app = SimulationApp(app_config, experience=experience)

    # Import Isaac Sim modules only AFTER SimulationApp
    from isaacsim.core.api import World
    import omni.kit.app
    import omni.usd

    # Explicitly open requested USD to guarantee the stage is loaded before normalization.
    if open_usd:
        usd_ctx = omni.usd.get_context()
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

        # If user provided target sim Hz, ensure main loop is rate-limited (ROS tutorial shows this)
        if args.target_sim_hz > 0:
            hz = int(round(args.target_sim_hz))
            settings.set_bool("/app/runLoops/main/rateLimitEnabled", True)
            settings.set_int("/app/runLoops/main/rateLimitFrequency", hz)
            settings.set_int("/persistent/simulation/minFrameRate", hz)
        else:
            # Disable limits to run as fast as possible
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

    if args.enable_ros2:
        stage = omni.usd.get_context().get_stage()
        if stage:
            if not args.disable_physx_laserscan:
                try:
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
                        reason = physx_stats.get("reason", "unknown reason")
                        print(f"[fast_isaac_sim] PhysX LaserScan auto-swap skipped: {reason}")
                except Exception as exc:
                    print(f"[fast_isaac_sim] WARNING: PhysX LaserScan auto-swap failed: {exc}")

            if args.ros_namespace:
                ns_stats = _normalize_ros2_namespace(stage, args.ros_namespace)
                print(
                    "[fast_isaac_sim] ROS2 namespace normalization:"
                    f" bridge_nodes={ns_stats['bridge_nodes']},"
                    f" namespace_updates={ns_stats['namespace_updates']},"
                    f" topic_updates={ns_stats['topic_updates']},"
                    f" frame_updates={ns_stats['frame_updates']}"
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
        else:
            print("[fast_isaac_sim] WARNING: No USD stage available for ROS2 namespace normalization.")

    world.reset()

    # Main loop
    step = 0
    t0 = time.perf_counter()

    print("[fast_isaac_sim] Starting simulation loop...")
    while simulation_app.is_running():
        if args.max_steps > 0 and step >= args.max_steps:
            break

        # Render policy:
        # - If no rendering is needed, keep render=False.
        # - If rendering is needed, render only every N steps if requested.
        do_render = False
        if want_render:
            if args.render_every <= 1:
                do_render = True
            else:
                do_render = (step % args.render_every) == 0

        world.step(render=do_render)
        step += 1

        if args.fps_report_interval > 0 and (step % args.fps_report_interval) == 0:
            dt = max(time.perf_counter() - t0, 1e-9)
            fps = step / dt
            print(f"[fast_isaac_sim] steps={step}  wall={dt:.2f}s  approx_step_fps={fps:.1f}  render={'Y' if do_render else 'N'}")

    wall = max(time.perf_counter() - t0, 1e-9)
    avg = step / wall
    print("[fast_isaac_sim] Finished")
    print(f"  steps={step}")
    print(f"  wall={wall:.2f}s")
    print(f"  avg_step_fps={avg:.1f}")

    simulation_app.close()


if __name__ == "__main__":
    main()
