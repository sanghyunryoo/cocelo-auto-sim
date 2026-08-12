# Front-wall angle interface

`front_wall_angle_estimator` consumes Super-LIO's de-skewed `/lio/cloud_world`
`sensor_msgs/msg/PointCloud2`, transforms each cloud to `base_link` at the cloud
timestamp, and publishes `autonomy_light/msg/WallAngle` on
`/autonomy_light/front_wall_angle`.

It also publishes the exact ROI-filtered cloud on
`/autonomy_light/front_wall_roi` as `sensor_msgs/msg/PointCloud2`. Its frame
is `base_link`; it contains every point that passes the configured x/y/z ROI,
before the estimator's internal point cap.

In RViz2, set **Fixed Frame** to `base_link`, add a **PointCloud2** display,
and choose `/autonomy_light/front_wall_roi` as its topic.

`relative_angle_deg` is the signed yaw from the robot's forward axis (`base_link`
`+X`) to the vertical-plane normal pointing from the robot toward the obstacle.
Zero therefore means the robot faces the wall square-on; positive means the wall
normal is to the right. `distance_m` is the perpendicular range to that plane.

## Separate SLAM fallback angle

`/autonomy_light/slam_angle` publishes `std_msgs/msg/Float64` in **degrees**.
It does not change `/autonomy_light/front_wall_angle` or any RAW/SUBMAP/FINAL
logic. It is solely a fallback from the timestamped `map -> base_link` SLAM
yaw, reduced to the signed remainder around the nearest 90° heading:
`slam_angle = remainder(slam_yaw, 90°)`. Thus `+93°` publishes `+3°` and
`+88°` publishes `-2°`. The result lies in `[-45°, +45°]` and is independent of
wall detection.

The estimator accepts only front-facing, vertically extended planes. It uses
RANSAC followed by total-least-squares line fitting in the horizontal plane.
`angle_stddev_deg` is the per-scan geometric 1-sigma estimate and is always
published with a geometrically valid measurement; it never suppresses a 3°,
4°, or other relative angle.

By default the final measurement also uses a short rolling local submap. Each
prior ROI cloud is retained in Super-LIO's map frame, then reprojected into the
current `base_link` frame using the timestamped LIO transform. This lets LIO's
pose/yaw stabilize the same local surface without defining one global wall or
assuming that a wall remains valid after a turn. The current scan is never put
into the submap until after its estimate is made. `raw_*` reports the
current-scan fit, `submap_*` reports the prior-scan fit, and the original
`relative_angle_*` fields report the final result. `estimate_source` identifies
whether it came from RAW, SUBMAP, a compatible fusion, or `RAW_CONFLICT`.

## Startup alignment gate

The supplied configuration enables a startup `INIT` gate. Before `FINAL` is
released, the estimator collects the independent RAW/SUBMAP fusion candidates
in a short window and calculates their circular mean wall angle. The robot must
be presented square-on to the initial front wall: the mean must be within
`initialization_tolerance_deg` (±1° by default) of the 0° target after at
least `initialization_min_frames` valid frames. While this is not true,
`detected=false`, `detection_reason=REASON_INITIALIZING`, and the final numeric
fields are NaN; RAW and SUBMAP diagnostics continue to be published. The red
`INIT` row in the status table states the rolling mean wall angle, 0° target,
tolerance, and frame count so the operator can align the robot. Once the condition is met,
it is latched for that process lifetime and FINAL output begins. Set
`require_initial_alignment: false` to bypass this gate, or tune the tolerance,
minimum frame count, and `initialization_window_sec` in the YAML configuration.

`launch.sh` uses this gate in a two-stage startup. It first starts a temporary
Super-LIO instance and the wall estimator only to obtain the INIT measurement.
The corresponding map is thrown away after acceptance; both temporary
processes stop, and Super-LIO starts again with a fresh operational map/yaw
while the robot is already aligned. Therefore the normal RAW/SUBMAP/FINAL
monitor and the operational SLAM begin only after INIT succeeds. This is why
`--no-drivers` is intentionally rejected by the launcher: it cannot safely
restart an externally owned Super-LIO process.

Raw and submap candidates are fused only when both their angle and perpendicular
range are within the configured fusion gates. When they disagree, `RAW_CONFLICT`
uses the current scan as its candidate so two physical walls are never averaged.
It does **not** reset FINAL. Every valid candidate then passes through a
unit-normal exponential average (0.45 s by default) and a bounded FINAL output
rate (8 deg/s by default). This prevents a one-scan RAW/SUBMAP disagreement
from creating a 1–2 degree FINAL jump, while a sustained robot turn or real wall
hand-over still converges smoothly. The filter resets after 0.50 s with no valid
newer measurement. Set `angle_filter_max_rate_deg_per_sec: 0.0` to disable only
the rate bound; `angle_filter_time_constant_sec: 0.0` disables time smoothing.
Outside the startup INIT gate, `detected=false` and NaN final numeric fields mean
that neither estimate met the configured geometric requirements in that scan.

`detection_reason` identifies the final outcome: valid detection, invalid cloud,
missing timestamped TF, PointCloud2 read error, insufficient ROI points, or no
plane satisfying the wall gates. `raw_detection_reason` explains the direct
current-scan result independently of a possible submap recovery.
`roi_point_count`, `best_candidate_inlier_count`, and
`required_inlier_count` accompany every message so a low-support rejection can
be diagnosed without replaying a bag. `submap_point_count` shows how much prior
ROI data reached the current local view.

When multiple qualified planes are present in the ROI, the estimator reports
the one with the greatest inlier support (then the nearest one on a tie). This
prevents a sparse, incidental plane fragment from replacing a well-observed
front wall on the next scan.

This is a per-scan geometric confidence, not a guarantee against extrinsic
calibration error, LiDAR range bias, motion distortion outside Super-LIO's
correction, or a non-planar obstacle. Validate the calibrated `base_link` ↔
LiDAR transform against a surveyed wall before treating the reported uncertainty
as a system-accuracy requirement.

The default `min_vertical_span_m: 0.12` intentionally permits a 0.20–0.35 m
vertical face, while excluding a horizontal floor. The implementation removes
the fitted height slope along the candidate wall before measuring this span, so
it also rejects a floor seen while the robot pitches. Tune the ROI, inlier
count, and minimum horizontal span in `config/autonomy_light.yaml` for the
installed LiDAR and expected obstacle width. The supplied MID360 front-wall
configuration uses `min_inliers: 150`: it was selected from the live wall
stream to reject a sparse 0.8 m secondary plane while retaining roughly 28 Hz
of stable wall detections. Reduce it only after confirming the target wall has
enough support at the maximum required range.

A purely horizontal top surface has no observable yaw direction, so it is
intentionally not reported as a wall. A 0.20–0.35 m obstacle is detected when
its front or side **vertical face** is visible with enough width and LiDAR
returns.

The default front ROI starts at `0.05 m` in `base_link`, and Super-LIO's
near-range (`lio.sensor.blind`) filter is also `0.05 m`. Both values must stay
at or below the minimum obstacle range that must be measured; a LiDAR cannot
report surfaces inside its hardware minimum range.
