# A front-facing vertical plane in base_link coordinates.
#
# relative_angle_rad is the signed yaw from base_link +X (forward) to the
# plane normal that points from the robot toward the obstacle. A wall directly
# in front of the robot is therefore 0 rad; positive is to the robot's right.
# When detected is false, the final floating-point fields are NaN.
# detection_reason and the point-count diagnostics explain why no final angle
# was published; the raw/submap diagnostic fields below can still be present.

uint8 REASON_DETECTED=0
uint8 REASON_INVALID_CLOUD=1
uint8 REASON_TF_UNAVAILABLE=2
uint8 REASON_CLOUD_READ_ERROR=3
uint8 REASON_INSUFFICIENT_ROI_POINTS=4
uint8 REASON_NO_QUALIFIED_PLANE=5
uint8 REASON_INITIALIZING=6

# Which measurement produced the final relative_angle_* fields.
# RAW_CONFLICT means the current scan and local submap disagree, so the
# current scan was deliberately kept to avoid delaying a newly encountered wall.
uint8 SOURCE_NONE=0
uint8 SOURCE_RAW=1
uint8 SOURCE_SUBMAP=2
uint8 SOURCE_FUSED=3
uint8 SOURCE_RAW_CONFLICT=4

std_msgs/Header header
bool detected
uint8 detection_reason
float64 relative_angle_rad
float64 relative_angle_deg
float64 distance_m
float64 angle_stddev_deg
uint32 inlier_count
float64 horizontal_span_m
float64 vertical_span_m
uint32 roi_point_count
uint32 best_candidate_inlier_count
uint32 required_inlier_count

# Raw means the fit from this one PointCloud2. Submap means the independent
# fit from prior ROI points reprojected with the timestamped map -> base_link
# transform. These fields make the fusion observable; unavailable floats are
# NaN. The final fields above are the result used by consumers.
uint8 estimate_source
uint8 raw_detection_reason
bool raw_detected
float64 raw_relative_angle_deg
float64 raw_distance_m
float64 raw_angle_stddev_deg
uint32 raw_inlier_count
bool submap_detected
float64 submap_relative_angle_deg
float64 submap_distance_m
float64 submap_angle_stddev_deg
uint32 submap_inlier_count
uint32 submap_point_count

# Startup alignment gate. Until initialization_complete is true, the final
# fields are deliberately withheld even if raw/submap diagnostics are valid.
bool initialization_complete
float64 initialization_angle_error_deg
uint32 initialization_sample_count
uint32 initialization_required_sample_count
float64 initialization_tolerance_deg
