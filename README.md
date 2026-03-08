# Isaac Projects

ROS2-first Isaac Sim 5.1 project for fast warehouse simulation, benchmarking, and teleop-focused scene variants.

> [!IMPORTANT]
> This repo is tuned for **Isaac Sim 5.1**. Launch through Isaac Sim `python.sh` when running `fast_isaac_sim.py`.

> [!NOTE]
> `test_factory.usda` references Isaac assets from the online 5.1 asset root (`omniverse-content-production`). First run may be slower while assets are cached.

## Features

- ROS2 enabled by default with clock and TF visibility.
- Headless-first launcher with high-throughput runtime controls.
- Startup reliability guard for playback-tick ROS graphs in headless mode.
- Data-based ROS readiness gate (default: wait for first `/clock` sample, timeout `90s`).
- Prebuilt scene variants for:
  - baseline ROS2 control,
  - minimal perception,
  - 720p camera-only teleop.
- Automatic low-cost PhysX LaserScan swap for supported scan paths.

## Project Structure

- `fast_isaac_sim.py` - high-performance Isaac Sim launcher.
- `test_factory.usda` - base warehouse scene.
- `test_factory_baseline_ros2.usda` - baseline control profile (sensors mostly off).
- `test_factory_step3_minimal.usda` - one camera + one scan, no pointcloud.
- `test_factory_camera_only_720p.usda` - one 720p camera, no scan/pointcloud.
- `test_factory_camera_only_480p.usda` - one 480p camera, no scan/pointcloud.
- `test_factory_camera_only_360p.usda` - one 360p camera, no scan/pointcloud.
- `test_factory_camera_only_360p_namespaced.usda` - 360p camera profile intended for launcher-side ROS namespace/frame prefixing.
- `hospital_experiment.usda` - hospital experiment scene.
- `hospital_experiment_exp1a.usda` - hospital exp1a (all robots front scan on, only robot1 camera at 300x200).
- `hospital_experiment_exp1b.usda` - hospital exp1b (no scans, only robot1 camera at 300x200).
- `.gitignore` - Git-safe defaults for Python/ROS/Isaac local artifacts.
- `LICENSE` - MIT license.
- `CONTRIBUTING.md` - lightweight contribution guide.

## Requirements

- Isaac Sim 5.1
- ROS2 environment sourced in terminal where you run ROS2 checks
- NVIDIA GPU + driver stack compatible with Isaac Sim

## Quick Start

```bash
git clone https://github.com/Omotoye/isaac-projects.git isaac-projects
cd ~/isaac-sim
./python.sh ~/isaac-projects/fast_isaac_sim.py \
  --headless \
  --usd-path ~/isaac-projects/test_factory_baseline_ros2.usda \
  --no-ground-plane \
  --physics-step 0.0166667 \
  --target-sim-hz 60 \
  --max-steps -1
```

> [!TIP]
> For camera publishing in headless mode, add `--render-headless`. For ~30 FPS camera at 60 Hz physics, use `--render-every 2`.

## Launch Profiles

### Baseline ROS2 Control

```bash
cd ~/isaac-sim
./python.sh ~/isaac-projects/fast_isaac_sim.py \
  --headless \
  --usd-path ~/isaac-projects/test_factory_baseline_ros2.usda \
  --no-ground-plane \
  --physics-step 0.0166667 \
  --target-sim-hz 60 \
  --max-steps -1
```

### Step 3 Minimal Perception

- Front-left camera at `320x240`
- Front scan enabled (`/scan`)
- Pointcloud disabled

```bash
cd ~/isaac-sim
./python.sh ~/isaac-projects/fast_isaac_sim.py \
  --headless \
  --render-headless \
  --render-every 2 \
  --optimize-render \
  --usd-path ~/isaac-projects/test_factory_step3_minimal.usda \
  --no-ground-plane \
  --physics-step 0.0166667 \
  --target-sim-hz 60 \
  --max-steps -1
```

### Camera-Only Teleop (720p)

- Front-left camera at `1280x720`
- No scan
- No pointcloud
- TF and clock remain active

```bash
cd ~/isaac-sim
./python.sh ~/isaac-projects/fast_isaac_sim.py \
  --headless \
  --render-headless \
  --render-every 2 \
  --optimize-render \
  --usd-path ~/isaac-projects/test_factory_camera_only_720p.usda \
  --no-ground-plane \
  --physics-step 0.0166667 \
  --target-sim-hz 60 \
  --max-steps -1
```

### Camera-Only Teleop (480p)

- Front-left camera at `640x480`
- No scan
- No pointcloud
- TF and clock remain active

```bash
cd ~/isaac-sim
./python.sh ~/isaac-projects/fast_isaac_sim.py \
  --headless \
  --render-headless \
  --render-every 2 \
  --optimize-render \
  --usd-path ~/isaac-projects/test_factory_camera_only_480p.usda \
  --no-ground-plane \
  --physics-step 0.0166667 \
  --target-sim-hz 60 \
  --max-steps -1
```

### Camera-Only Teleop (360p)

- Front-left camera at `640x360`
- No scan
- No pointcloud
- TF and clock remain active

```bash
cd ~/isaac-sim
./python.sh ~/isaac-projects/fast_isaac_sim.py \
  --headless \
  --render-headless \
  --render-every 2 \
  --optimize-render \
  --usd-path ~/isaac-projects/test_factory_camera_only_360p.usda \
  --no-ground-plane \
  --physics-step 0.0166667 \
  --target-sim-hz 60 \
  --max-steps -1
```

### Camera-Only Teleop (360p, Namespaced)

- Front-left camera at `640x360`
- No scan
- No pointcloud
- Namespaces topics and frame IDs using `nova_carter`

> [!NOTE]
> Isaac `ROS2PublishTransformTree` derives many TF frame names from USD prims. The launcher keeps the TF anchor (`base_link`) unprefixed while prefixing the odom parent (`nova_carter/odom`) to keep one connected tree.

```bash
cd ~/isaac-sim
./python.sh ~/isaac-projects/fast_isaac_sim.py \
  --headless \
  --render-headless \
  --render-every 2 \
  --optimize-render \
  --usd-path ~/isaac-projects/test_factory_camera_only_360p_namespaced.usda \
  --ros-namespace nova_carter \
  --namespace-tf-topics \
  --no-ground-plane \
  --physics-step 0.0166667 \
  --target-sim-hz 60 \
  --max-steps -1
```

### Hospital Experiment

- Canonical ROS-working headless command for this scene
- Keeps scene behavior as authored (3 robots, one camera per robot)
- Uses authored namespaces: `/carter1`, `/carter2`, `/carter3`
- Do not pass `--ros-namespace` for this scene

> [!TIP]
> Hospital scenes use DLSS Performance (`--aa-mode 3 --dlss-exec-mode 0`) by default in the launcher unless you explicitly override these flags.

```bash
cd ~/isaac-sim
export ROS_DOMAIN_ID=5
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
./python.sh ~/isaac-projects/fast_isaac_sim.py \
  --headless \
  --render-headless \
  --render-every 2 \
  --aa-mode 3 \
  --dlss-exec-mode 0 \
  --usd-path ~/isaac-projects/hospital_experiment.usda \
  --no-ground-plane \
  --physics-step 0.0166667 \
  --target-sim-hz 60 \
  --max-steps -1
```

> [!IMPORTANT]
> In this hospital scene, ROS graphs are playback-tick driven. If rendered stepping is effectively off, topics may be discoverable but silent. The launcher now fails fast in that case.

> [!NOTE]
> First run can take around `60-90s` before first ROS samples appear due graph/shader warmup. Readiness is data-based (first message received), not `ros2 topic list` based.

> [!TIP]
> The launcher readiness probe uses `ros2 topic echo --once` in a sanitized ROS shell and auto-sources `/opt/ros/$ROS_DISTRO/setup.bash` when available.

> [!TIP]
> Hospital cloud profile uses authored RTX sensors: all 3 robots publish front/back 2D lidar scans and front camera at `640x480`.

```bash
# second terminal
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=5
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 topic list | rg '^/(carter1|carter2|carter3|clock)'
ros2 topic echo /clock --once
ros2 topic echo /carter1/chassis/odom --once
ros2 topic echo /carter1/tf --once
ros2 topic echo /carter2/chassis/odom --once
ros2 topic echo /carter3/chassis/odom --once
ros2 topic list | rg '^/carter(1|2|3)/(front|back)_2d_lidar/scan'
ros2 topic hz /carter1/front_2d_lidar/scan
ros2 topic echo /carter1/front_2d_lidar/scan --once | rg 'frame_id|scan_time'
```

### Hospital Exp 1A (Cloud/Local Profile)

- 3 robots
- Front 2D LaserScan enabled on all robots
- Only `carter1` camera enabled at `300x200`

Local PC:

```bash
cd ~/isaac-sim
./python.sh ~/isaac-projects/fast_isaac_sim.py \
  --headless \
  --render-headless \
  --render-every 2 \
  --aa-mode 3 \
  --dlss-exec-mode 0 \
  --usd-path ~/isaac-projects/hospital_experiment_exp1a.usda \
  --no-ground-plane \
  --physics-step 0.0166667 \
  --target-sim-hz 60 \
  --max-steps -1
```

Cloud:

```bash
cd /isaac-sim
./python.sh ~/isaac-projects/fast_isaac_sim.py \
  --headless \
  --render-headless \
  --render-every 2 \
  --aa-mode 3 \
  --dlss-exec-mode 0 \
  --usd-path ~/isaac-projects/hospital_experiment_exp1a.usda \
  --no-ground-plane \
  --physics-step 0.0166667 \
  --target-sim-hz 60 \
  --max-steps -1
```

### Hospital Exp 1B (Cloud/Local Profile)

- 3 robots
- No LaserScan publishers
- Only `carter1` camera enabled at `300x200`

Local PC:

```bash
cd ~/isaac-sim
./python.sh ~/isaac-projects/fast_isaac_sim.py \
  --headless \
  --render-headless \
  --render-every 2 \
  --aa-mode 3 \
  --dlss-exec-mode 0 \
  --usd-path ~/isaac-projects/hospital_experiment_exp1b.usda \
  --no-ground-plane \
  --physics-step 0.0166667 \
  --target-sim-hz 60 \
  --max-steps -1
```

Cloud:

```bash
cd /isaac-sim
./python.sh ~/isaac-projects/fast_isaac_sim.py \
  --headless \
  --render-headless \
  --render-every 2 \
  --aa-mode 3 \
  --dlss-exec-mode 0 \
  --usd-path ~/isaac-projects/hospital_experiment_exp1b.usda \
  --no-ground-plane \
  --physics-step 0.0166667 \
  --target-sim-hz 60 \
  --max-steps -1
```

### Future Robot Feature Profiles (Hybrid)

- Keep `hospital_experiment.usda` as source scene.
- Apply launcher profiles first for sensor toggles and camera overrides.
- Use new USD variants only if topology changes are required.

Example profile runs:

```bash
# Disable camera/lidar publishers where profile hooks exist
./python.sh ~/isaac-projects/fast_isaac_sim.py \
  --headless \
  --render-headless \
  --render-every 2 \
  --aa-mode 3 \
  --dlss-exec-mode 0 \
  --usd-path ~/isaac-projects/hospital_experiment.usda \
  --robot-profile sensors-off \
  --target-sim-hz 60 \
  --max-steps -1
```

```bash
# Single front camera override (launcher-side)
./python.sh ~/isaac-projects/fast_isaac_sim.py \
  --headless \
  --render-headless \
  --render-every 2 \
  --aa-mode 3 \
  --dlss-exec-mode 0 \
  --usd-path ~/isaac-projects/hospital_experiment.usda \
  --robot-profile single-front-camera \
  --profile-camera-width 320 \
  --profile-camera-height 240 \
  --target-sim-hz 60 \
  --max-steps -1
```

## ROS2 Validation

Run from a ROS2-sourced terminal while simulation is active.

```bash
ros2 topic list | sort
ros2 topic echo /clock --once
ros2 topic echo /tf --once
ros2 topic echo /tf_static --once
ros2 topic hz /front_left/image_raw
ros2 topic hz /scan
ros2 topic list | rg -i '(lidar|pointcloud|pcl|lidar_points)'
```

## Key Launcher Flags

- `--disable-ros2`: disable ROS2 bridge.
- `--ros-namespace <name>`: optional namespace rewrite (default off).
- `--namespace-tf-topics`: include TF topics in namespace when `--ros-namespace` is set.
- `--disable-ros-render-guard`: disable fail-fast check for silent playback-tick ROS runs.
- `--skip-ros-ready-check`: skip startup wait for first sample on `--ros-ready-topic`.
- `--ros-ready-topic <topic>`: readiness topic (default `/clock`).
- `--ros-ready-timeout <sec>`: readiness timeout in seconds (default `90`).
- `--render-headless`: required for RTX camera publishing in headless mode.
- `--render-every N`: render every `N` physics steps (`N >= 1`).
- `--target-sim-hz <hz>`: real-time pacing target.
- `--disable-physx-laserscan`: keep authored RTX scan path (skip auto PhysX swap).
- `--robot-profile <scene-default|sensors-off|single-front-camera>`: launcher-driven feature profile.
- `--profile-camera-width`, `--profile-camera-height`: profile camera resolution overrides.

> [!WARNING]
> This repo includes large USD content (for example `test_factory.usda`). If history grows, consider Git LFS for large binary assets.

## Cloud Deployment Notes

> [!IMPORTANT]
> Use a GPU-backed cloud instance with current NVIDIA drivers and outbound internet access for online Isaac assets.

> [!TIP]
> Clone this repo on cloud and run the same commands. Local `../isaacsim_assets` is not required for references already switched to online URLs.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for branch, check, and PR conventions.

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE).
