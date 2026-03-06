# Isaac Projects

ROS2-first Isaac Sim 5.1 project for fast warehouse simulation, benchmarking, and teleop-focused scene variants.

> [!IMPORTANT]
> This repo is tuned for **Isaac Sim 5.1**. Launch through Isaac Sim `python.sh` when running `fast_isaac_sim.py`.

> [!NOTE]
> `test_factory.usda` references Isaac assets from the online 5.1 asset root (`omniverse-content-production`). First run may be slower while assets are cached.

## Features

- ROS2 enabled by default with clock and TF visibility.
- Headless-first launcher with high-throughput runtime controls.
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
- `--render-headless`: required for RTX camera publishing in headless mode.
- `--render-every N`: render every `N` physics steps (`N >= 1`).
- `--target-sim-hz <hz>`: real-time pacing target.
- `--disable-physx-laserscan`: keep authored RTX scan path (skip auto PhysX swap).

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
