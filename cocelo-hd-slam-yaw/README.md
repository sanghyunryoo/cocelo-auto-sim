# autonomy_light

This workspace runs Livox + Super-LIO in real time and publishes the angle and
range of the nearest qualified front wall/vertical obstacle. There is no
elevation map, height-map bridge, D435 dependency, DDS height-map transport,
saved-map relocation, or pre-mapping workflow.

```
Livox CustomMsg + IMU -> Super-LIO -> /lio/cloud_world (map frame)
                                      |
                                      v
                      front_wall_angle_estimator
                         (transform to base_link)
                                      |
                                      v
             /autonomy_light/front_wall_angle (WallAngle)
```

Build and run:

```bash
./build.sh --clean
./launch.sh --real
```

`launch.sh` always prints a compact terminal status table (2 Hz by default)
with the current-scan (`RAW`), LIO-aligned local submap (`SUBMAP`), final
fused wall angle/range, and Super-LIO position/velocity:

```bash
# Choose another display rate when needed.
./launch.sh --real --vis-rate 5
```

At startup the default configuration holds `FINAL` output until the robot is
aligned to the first front wall within ±1°. The status table shows the rolling
mean signed wall angle and frame count in the red `INIT` state. This
gate is configurable in `config/autonomy_light.yaml` and can be bypassed with
`require_initial_alignment: false`.

When `launch.sh` owns Super-LIO, it runs one temporary LIO instance solely for
this INIT measurement. On acceptance that temporary process and its map are
discarded, then Super-LIO starts again with a fresh operational map/yaw from
the aligned robot pose. During INIT the display contains only alignment status;
the normal RAW/SUBMAP/FINAL and LIO table begins after the restart.

For a simulation that already publishes Livox CustomMsg and IMU:

```bash
./launch.sh --sim --sim-topic-prefix /f4
```

The `livox:` block in `config/autonomy_light.yaml` is the sole source of
physical LiDAR settings: `model` (`mid360` or `mid360s`), `interface`,
computer `host_ip`, sensor `lidar_ip`, UDP ports, frame, and driver options.
The launcher generates the Livox driver JSON at runtime, so vendor files under
`third_party/` are never edited. It verifies that `host_ip` is configured on
the selected `interface`, but deliberately never changes the computer network
configuration itself. The same model, endpoints, and CustomMsg/IMU topics are
also injected into Super-LIO's generated runtime parameters; Driver2 alone
opens the LiDAR IP socket, while Super-LIO consumes its ROS topics.

For a MID360S installation, change only the `livox:` values in that YAML, for
example `model: "mid360s"`, its NIC name/IP, and the LiDAR's configured IP;
then run the same `./launch.sh --real` command.

Build a source-free Debian runtime package with:

```bash
scripts/package_deb.sh
sudo apt install ./dist/cocelo-hd-slam-yaw_*.deb
autonomy-slam --real
```

The target needs the same ROS 2 distribution preinstalled; the editable
installed LiDAR configuration is
`/etc/cocelo/cocelo-hd-slam-yaw/autonomy_light.yaml`. The installed Korean
user guide is `/usr/share/doc/cocelo-hd-slam-yaw/user_guide_ko.pdf`.

The wall estimator uses RANSAC plus total-least-squares fitting on the
de-skewed Super-LIO cloud. Every geometrically valid wall measurement is
published, including 3–4° relative angles; the message includes its per-scan
geometric uncertainty for downstream monitoring. See
[`docs/wall_angle_interface.md`](docs/wall_angle_interface.md) for the frame,
sign, confidence, and calibration requirements. The ROI and size gates are in
[`config/autonomy_light.yaml`](config/autonomy_light.yaml).
For a 0.20–0.35 m obstacle, its vertical face must be visible; a horizontal
top surface alone does not define a yaw angle.

Inspect the result:

```bash
ros2 topic echo /autonomy_light/front_wall_angle
```

## Nav2 goal navigation

Nav2 starts by default after Super-LIO publishes its first `/lio/odom`. It uses
the continuous `map -> odom -> base_link` TF chain and builds rolling global
and local costmaps directly from `/lio/cloud_world`; AMCL and a separately
saved 2-D map are deliberately not used.

In RViz2 choose the `2D Goal Pose` tool and click/drag a goal in the `map`
frame. The launcher configuration publishes that pose on `/goal_pose`, and
`nav2_goal_pose_bridge` forwards it to the standard `/navigate_to_pose` action.

The same goal can be sent from a terminal without manually assembling an
action message:

```bash
# X[m] Y[m] YAW[deg]
./scripts/nav_goal.sh 2.0 1.0 90
```

The terminal command exits successfully only when Nav2 reports `SUCCEEDED`.
Use `./launch.sh --no-nav2` for SLAM-only operation. The source-tree launcher
can install a Jazzy Nav2 runtime without root access under `.deps`:

```bash
./scripts/setup_nav2.sh --ros-distro jazzy
```

Nav2 emits raw controller commands on `/nav2/cmd_vel_raw` and acceleration-
limited `geometry_msgs/Twist` on `/nav2/cmd_vel`. The simulator subscribes to
the latter directly. A hardware deployment should route the same final topic
through its motor safety/mux layer.

To view the exact wall-estimation ROI in RViz2, set its Fixed Frame to
`base_link`, add a `PointCloud2` display, and select
`/autonomy_light/front_wall_roi`.

`./launch.sh` also publishes the cumulative Super-LIO map as
`/lio/global_map` (`sensor_msgs/msg/PointCloud2`, frame `map`) at about 1 Hz.
Set RViz2 Fixed Frame to `map`, add a `PointCloud2` display, and select that
topic. This live topic is enabled by default; `lio.map.save_map` remains the
separate optional PCD-on-shutdown setting.
# cocelo-hd-slam-yaw
# cocelo-hd-slam-yaw
