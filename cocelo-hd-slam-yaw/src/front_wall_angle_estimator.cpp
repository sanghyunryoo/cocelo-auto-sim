#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "autonomy_light/msg/wall_angle.hpp"

namespace autonomy_light {
namespace {

constexpr double kPi = 3.14159265358979323846;

struct Point {
  double x;
  double y;
  double z;
};

struct LineModel {
  double nx{0.0};
  double ny{0.0};
  double offset{0.0};
  std::vector<std::size_t> inliers;
};

struct Detection {
  double angle_rad{0.0};
  double distance_m{0.0};
  double angle_stddev_deg{std::numeric_limits<double>::infinity()};
  double horizontal_span_m{0.0};
  double vertical_span_m{0.0};
  std::size_t inlier_count{0};
};

struct DetectionSearch {
  std::optional<Detection> detection;
  std::size_t best_candidate_inlier_count{0};
};

struct SubmapScan {
  rclcpp::Time stamp;
  std::vector<Point> points_in_source_frame;
};

struct InitializationSample {
  rclcpp::Time stamp;
  double angle_rad{0.0};
};

double degrees(const double radians) { return radians * 180.0 / kPi; }

bool hasField(const sensor_msgs::msg::PointCloud2 &cloud,
              const std::string &name) {
  return std::any_of(cloud.fields.begin(), cloud.fields.end(),
                     [&name](const sensor_msgs::msg::PointField &field) {
                       return field.name == name;
                     });
}

}  // namespace

class FrontWallAngleEstimator final : public rclcpp::Node {
 public:
  FrontWallAngleEstimator()
      : Node("front_wall_angle_estimator"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) {
    input_topic_ = declare_parameter<std::string>("input_topic", "/lio/cloud_world");
    output_topic_ = declare_parameter<std::string>(
        "output_topic", "/autonomy_light/front_wall_angle");
    slam_angle_topic_ = declare_parameter<std::string>(
        "slam_angle_topic", "/autonomy_light/slam_angle");
    roi_cloud_topic_ = declare_parameter<std::string>(
        "roi_cloud_topic", "/autonomy_light/front_wall_roi");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    transform_timeout_sec_ = declare_parameter<double>("transform_timeout_sec", 0.05);
    min_forward_distance_m_ =
        declare_parameter<double>("min_forward_distance_m", 0.05);
    max_forward_distance_m_ =
        declare_parameter<double>("max_forward_distance_m", 4.00);
    max_lateral_distance_m_ =
        declare_parameter<double>("max_lateral_distance_m", 0.50);
    min_z_m_ = declare_parameter<double>("min_z_m", -1.00);
    max_z_m_ = declare_parameter<double>("max_z_m", 1.50);
    max_cloud_points_ = declare_parameter<int>("max_cloud_points", 5000);
    ransac_iterations_ = declare_parameter<int>("ransac_iterations", 400);
    ransac_distance_threshold_m_ =
        declare_parameter<double>("ransac_distance_threshold_m", 0.025);
    min_inliers_ = declare_parameter<int>("min_inliers", 250);
    min_horizontal_span_m_ =
        declare_parameter<double>("min_horizontal_span_m", 0.70);
    min_vertical_span_m_ = declare_parameter<double>("min_vertical_span_m", 0.12);
    max_abs_relative_angle_rad_ = degreesToRadians(
        declare_parameter<double>("max_abs_relative_angle_deg", 70.0));
    angle_filter_time_constant_sec_ =
        declare_parameter<double>("angle_filter_time_constant_sec", 0.45);
    angle_filter_reset_sec_ =
        declare_parameter<double>("angle_filter_reset_sec", 0.50);
    angle_filter_max_rate_rad_per_sec_ = degreesToRadians(
        declare_parameter<double>("angle_filter_max_rate_deg_per_sec", 8.0));
    enable_submap_fusion_ =
        declare_parameter<bool>("enable_submap_fusion", true);
    submap_history_sec_ = declare_parameter<double>("submap_history_sec", 0.75);
    submap_points_per_scan_ =
        declare_parameter<int>("submap_points_per_scan", 600);
    submap_max_points_ = declare_parameter<int>("submap_max_points", 5000);
    submap_ransac_iterations_ =
        declare_parameter<int>("submap_ransac_iterations", 160);
    fusion_max_angle_difference_rad_ = degreesToRadians(
        declare_parameter<double>("fusion_max_angle_difference_deg", 3.0));
    fusion_max_distance_difference_m_ =
        declare_parameter<double>("fusion_max_distance_difference_m", 0.20);
    require_initial_alignment_ =
        declare_parameter<bool>("require_initial_alignment", true);
    initialization_tolerance_rad_ = degreesToRadians(
        declare_parameter<double>("initialization_tolerance_deg", 1.0));
    initialization_min_frames_ =
        declare_parameter<int>("initialization_min_frames", 10);
    initialization_window_sec_ =
        declare_parameter<double>("initialization_window_sec", 1.0);
    initialization_complete_ = !require_initial_alignment_;

    if (input_topic_.empty() || output_topic_.empty() || slam_angle_topic_.empty() ||
        roi_cloud_topic_.empty() || base_frame_.empty()) {
      throw std::invalid_argument(
          "input_topic, output_topic, slam_angle_topic, roi_cloud_topic, and base_frame must not be empty");
    }
    if (transform_timeout_sec_ < 0.0 || min_forward_distance_m_ <= 0.0 ||
        max_forward_distance_m_ <= min_forward_distance_m_ ||
        max_lateral_distance_m_ <= 0.0 || max_z_m_ <= min_z_m_ ||
        max_cloud_points_ < 3 || ransac_iterations_ < 1 ||
        ransac_distance_threshold_m_ <= 0.0 || min_inliers_ < 3 ||
        min_horizontal_span_m_ <= 0.0 || min_vertical_span_m_ <= 0.0 ||
        max_abs_relative_angle_rad_ <= 0.0 ||
        max_abs_relative_angle_rad_ >= kPi / 2.0 ||
        angle_filter_time_constant_sec_ < 0.0 || angle_filter_reset_sec_ <= 0.0 ||
        angle_filter_max_rate_rad_per_sec_ < 0.0 ||
        submap_history_sec_ <= 0.0 || submap_points_per_scan_ < 3 ||
        submap_max_points_ < min_inliers_ || submap_ransac_iterations_ < 1 ||
        fusion_max_angle_difference_rad_ <= 0.0 ||
        fusion_max_angle_difference_rad_ >= kPi / 2.0 ||
        fusion_max_distance_difference_m_ <= 0.0 ||
        initialization_tolerance_rad_ <= 0.0 ||
        initialization_tolerance_rad_ >= kPi / 2.0 ||
        initialization_min_frames_ < 1 || initialization_window_sec_ <= 0.0) {
      throw std::invalid_argument("front-wall angle estimator parameters are invalid");
    }

    publisher_ = create_publisher<msg::WallAngle>(output_topic_,
                                                   rclcpp::QoS(10).reliable());
    slam_angle_publisher_ = create_publisher<std_msgs::msg::Float64>(
        slam_angle_topic_, rclcpp::QoS(10).reliable());
    roi_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        roi_cloud_topic_, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&FrontWallAngleEstimator::onCloud, this, std::placeholders::_1));

    RCLCPP_INFO(
        get_logger(),
        "Front-wall angle estimator: input=%s, output=%s, SLAM fallback angle=%s "
        "(deg, yaw modulo 90 deg), ROI cloud=%s, "
        "ROI x=[%.2f, %.2f] m, |y|<=%.2f m, z=[%.2f, %.2f] m, "
        "FINAL stabilizer: tau=%.2f s, max-rate=%.2f deg/s; "
        "submap fusion=%s (history=%.2f s), "
        "startup alignment=%s (+/-%.2f deg, %d frames / %.2f s)",
        input_topic_.c_str(), output_topic_.c_str(), slam_angle_topic_.c_str(),
        roi_cloud_topic_.c_str(),
        min_forward_distance_m_, max_forward_distance_m_, max_lateral_distance_m_,
        min_z_m_, max_z_m_, angle_filter_time_constant_sec_,
        degrees(angle_filter_max_rate_rad_per_sec_),
        enable_submap_fusion_ ? "enabled" : "disabled", submap_history_sec_,
        require_initial_alignment_ ? "required" : "disabled",
        degrees(initialization_tolerance_rad_), initialization_min_frames_,
        initialization_window_sec_);
  }

 private:
  static double degreesToRadians(const double angle_deg) {
    return angle_deg * kPi / 180.0;
  }

  double filterAngle(const double measured_angle_rad,
                     const rclcpp::Time &stamp) {
    if (!last_angle_stamp_ || stamp <= *last_angle_stamp_) {
      filtered_angle_rad_ = measured_angle_rad;
      last_angle_stamp_ = stamp;
      return measured_angle_rad;
    }

    const double elapsed_sec = (stamp - *last_angle_stamp_).seconds();
    if (elapsed_sec > angle_filter_reset_sec_) {
      filtered_angle_rad_ = measured_angle_rad;
      last_angle_stamp_ = stamp;
      return measured_angle_rad;
    }

    double smoothed_angle_rad = measured_angle_rad;
    if (angle_filter_time_constant_sec_ > 0.0) {
      // Average unit normals rather than raw angles, so the filter remains
      // correct across the atan2 wrap boundary as well as near zero degrees.
      const double alpha =
          1.0 - std::exp(-elapsed_sec / angle_filter_time_constant_sec_);
      const double mean_x = (1.0 - alpha) * std::cos(*filtered_angle_rad_) +
                            alpha * std::cos(measured_angle_rad);
      const double mean_y = (1.0 - alpha) * std::sin(*filtered_angle_rad_) +
                            alpha * std::sin(measured_angle_rad);
      smoothed_angle_rad = std::atan2(mean_y, mean_x);
    }

    // A temporal average alone still makes a large single-scan innovation
    // visible in the final output. Bound the published angular speed too.
    // This applies to every source, including RAW_CONFLICT, so a transient
    // raw/submap disagreement cannot reset FINAL by one or two degrees in one
    // scan.
    const double maximum_step_rad =
        angle_filter_max_rate_rad_per_sec_ * elapsed_sec;
    const double requested_step_rad =
        wrappedAngleDifference(smoothed_angle_rad, *filtered_angle_rad_);
    if (angle_filter_max_rate_rad_per_sec_ > 0.0 &&
        std::abs(requested_step_rad) > maximum_step_rad) {
      *filtered_angle_rad_ += std::copysign(maximum_step_rad, requested_step_rad);
    } else {
      filtered_angle_rad_ = smoothed_angle_rad;
    }
    last_angle_stamp_ = stamp;
    return *filtered_angle_rad_;
  }

  static double wrappedAngleDifference(const double first, const double second) {
    return std::atan2(std::sin(first - second), std::cos(first - second));
  }

  static std::optional<double> projectedYaw(const tf2::Vector3 &vector) {
    const double horizontal_norm = std::hypot(vector.x(), vector.y());
    if (horizontal_norm <= 1e-6) {
      return std::nullopt;
    }
    return std::atan2(vector.y(), vector.x());
  }

  void publishSlamAngle(const tf2::Transform &from_base) {
    std_msgs::msg::Float64 result;
    result.data = std::numeric_limits<double>::quiet_NaN();
    const std::optional<double> raw_yaw = projectedYaw(
        from_base.getBasis() * tf2::Vector3(1.0, 0.0, 0.0));
    if (raw_yaw) {
      // ROS yaw is counter-clockwise positive.  std::remainder keeps the
      // signed residual around the nearest 90-degree heading: +93 -> +3 and
      // +88 -> -2.  This is a SLAM-only fallback and never affects FINAL.
      result.data = degrees(std::remainder(*raw_yaw, kPi / 2.0));
    }
    slam_angle_publisher_->publish(result);
  }

  bool isInRoi(const Point &point) const {
    return point.x >= min_forward_distance_m_ &&
           point.x <= max_forward_distance_m_ &&
           std::abs(point.y) <= max_lateral_distance_m_ &&
           point.z >= min_z_m_ && point.z <= max_z_m_;
  }

  static std::vector<Point> uniformlySamplePoints(const std::vector<Point> &points,
                                                   const std::size_t max_points) {
    if (points.size() <= max_points) {
      return points;
    }
    std::vector<Point> sampled;
    sampled.reserve(max_points);
    const double stride = static_cast<double>(points.size()) /
                          static_cast<double>(max_points);
    for (std::size_t index = 0; index < max_points; ++index) {
      sampled.push_back(points[static_cast<std::size_t>(index * stride)]);
    }
    return sampled;
  }

  bool supportsSubmap(const std::string &source_frame) const {
    // A base_link cloud has no fixed world frame in which old scans can be
    // safely accumulated. /lio/cloud_world is map-framed, which is the normal
    // and intended input for this estimator.
    return enable_submap_fusion_ && source_frame != base_frame_;
  }

  void prepareSubmap(const rclcpp::Time &stamp,
                     const std::string &source_frame) {
    if (submap_source_frame_ != source_frame) {
      submap_scans_.clear();
      submap_source_frame_ = source_frame;
      last_submap_stamp_.reset();
    }
    if (last_submap_stamp_ && stamp < *last_submap_stamp_) {
      // A bag loop or a restarted LIO clock must not combine different
      // trajectories in one local submap.
      submap_scans_.clear();
    }
    last_submap_stamp_ = stamp;
    while (!submap_scans_.empty() &&
           (stamp - submap_scans_.front().stamp).seconds() > submap_history_sec_) {
      submap_scans_.pop_front();
    }
  }

  std::vector<Point> collectSubmapPoints(const tf2::Transform &to_base,
                                         const rclcpp::Time &stamp,
                                         const std::string &source_frame) {
    if (!supportsSubmap(source_frame)) {
      submap_scans_.clear();
      submap_source_frame_.clear();
      last_submap_stamp_.reset();
      return {};
    }
    prepareSubmap(stamp, source_frame);

    std::vector<Point> current_base_points;
    for (const SubmapScan &scan : submap_scans_) {
      for (const Point &source_point : scan.points_in_source_frame) {
        const tf2::Vector3 transformed =
            to_base * tf2::Vector3(source_point.x, source_point.y, source_point.z);
        const Point point{transformed.x(), transformed.y(), transformed.z()};
        if (isInRoi(point)) {
          current_base_points.push_back(point);
        }
      }
    }
    return uniformlySamplePoints(current_base_points,
                                 static_cast<std::size_t>(submap_max_points_));
  }

  void appendToSubmap(const tf2::Transform &from_base,
                      const rclcpp::Time &stamp,
                      const std::string &source_frame,
                      const std::vector<Point> &current_base_points) {
    if (!supportsSubmap(source_frame) || current_base_points.empty()) {
      return;
    }
    prepareSubmap(stamp, source_frame);
    const std::vector<Point> sampled = uniformlySamplePoints(
        current_base_points, static_cast<std::size_t>(submap_points_per_scan_));
    SubmapScan scan;
    scan.stamp = stamp;
    scan.points_in_source_frame.reserve(sampled.size());
    for (const Point &point : sampled) {
      const tf2::Vector3 source_point =
          from_base * tf2::Vector3(point.x, point.y, point.z);
      scan.points_in_source_frame.push_back(
          {source_point.x(), source_point.y(), source_point.z()});
    }
    submap_scans_.push_back(std::move(scan));
  }

  void updateInitialization(const double angle_rad, const rclcpp::Time &stamp) {
    if (!require_initial_alignment_ || initialization_complete_) {
      return;
    }
    if (last_initialization_stamp_ && stamp < *last_initialization_stamp_) {
      initialization_samples_.clear();
      initialization_error_rad_.reset();
    }
    last_initialization_stamp_ = stamp;
    initialization_samples_.push_back({stamp, angle_rad});
    while (!initialization_samples_.empty() &&
           (stamp - initialization_samples_.front().stamp).seconds() >
               initialization_window_sec_) {
      initialization_samples_.pop_front();
    }

    double normal_x = 0.0;
    double normal_y = 0.0;
    for (const InitializationSample &sample : initialization_samples_) {
      normal_x += std::cos(sample.angle_rad);
      normal_y += std::sin(sample.angle_rad);
    }
    initialization_error_rad_ = std::atan2(normal_y, normal_x);

    if (initialization_samples_.size() >=
            static_cast<std::size_t>(initialization_min_frames_) &&
        std::abs(*initialization_error_rad_) <= initialization_tolerance_rad_) {
      initialization_complete_ = true;
    }
  }

  void setInitializationDiagnostics(msg::WallAngle &result) const {
    result.initialization_complete = initialization_complete_;
    result.initialization_angle_error_deg = initialization_error_rad_
        ? degrees(-*initialization_error_rad_)
        : std::numeric_limits<double>::quiet_NaN();
    result.initialization_sample_count =
        static_cast<std::uint32_t>(initialization_samples_.size());
    result.initialization_required_sample_count =
        static_cast<std::uint32_t>(initialization_min_frames_);
    result.initialization_tolerance_deg = degrees(initialization_tolerance_rad_);
  }

  static LineModel collectInliers(const std::vector<Point> &points,
                                  const double nx, const double ny,
                                  const double offset,
                                  const double threshold) {
    LineModel model;
    model.nx = nx;
    model.ny = ny;
    model.offset = offset;
    model.inliers.reserve(points.size());
    for (std::size_t index = 0; index < points.size(); ++index) {
      const Point &point = points[index];
      if (std::abs(nx * point.x + ny * point.y - offset) <= threshold) {
        model.inliers.push_back(index);
      }
    }
    return model;
  }

  static std::optional<LineModel> refitLine(const std::vector<Point> &points,
                                             const std::vector<std::size_t> &inliers,
                                             const double threshold) {
    if (inliers.size() < 3) {
      return std::nullopt;
    }

    double center_x = 0.0;
    double center_y = 0.0;
    for (const std::size_t index : inliers) {
      center_x += points[index].x;
      center_y += points[index].y;
    }
    center_x /= static_cast<double>(inliers.size());
    center_y /= static_cast<double>(inliers.size());

    double cov_xx = 0.0;
    double cov_xy = 0.0;
    double cov_yy = 0.0;
    for (const std::size_t index : inliers) {
      const double dx = points[index].x - center_x;
      const double dy = points[index].y - center_y;
      cov_xx += dx * dx;
      cov_xy += dx * dy;
      cov_yy += dy * dy;
    }
    if (cov_xx + cov_yy < 1.0e-12) {
      return std::nullopt;
    }

    const double tangent_yaw = 0.5 * std::atan2(2.0 * cov_xy, cov_xx - cov_yy);
    const double nx = -std::sin(tangent_yaw);
    const double ny = std::cos(tangent_yaw);
    const double offset = nx * center_x + ny * center_y;
    return collectInliers(points, nx, ny, offset, threshold);
  }

  std::optional<Detection> makeDetection(const std::vector<Point> &points,
                                         LineModel model) const {
    auto refined = refitLine(points, model.inliers, ransac_distance_threshold_m_);
    if (!refined || refined->inliers.size() < static_cast<std::size_t>(min_inliers_)) {
      return std::nullopt;
    }
    refined = refitLine(points, refined->inliers, ransac_distance_threshold_m_);
    if (!refined || refined->inliers.size() < static_cast<std::size_t>(min_inliers_)) {
      return std::nullopt;
    }
    model = std::move(*refined);

    double center_x = 0.0;
    double center_y = 0.0;
    for (const std::size_t index : model.inliers) {
      center_x += points[index].x;
      center_y += points[index].y;
    }
    center_x /= static_cast<double>(model.inliers.size());
    center_y /= static_cast<double>(model.inliers.size());

    // The plane normal is signed to point from the robot toward the obstacle.
    if (model.nx * center_x + model.ny * center_y < 0.0) {
      model.nx = -model.nx;
      model.ny = -model.ny;
      model.offset = -model.offset;
    }
    const double angle_rad = std::atan2(model.ny, model.nx);
    if (std::abs(angle_rad) > max_abs_relative_angle_rad_) {
      return std::nullopt;
    }

    const double tangent_x = -model.ny;
    const double tangent_y = model.nx;
    double min_tangent = std::numeric_limits<double>::infinity();
    double max_tangent = -std::numeric_limits<double>::infinity();
    double mean_z = 0.0;
    double residual_squared_sum = 0.0;
    double tangent_squared_sum = 0.0;
    for (const std::size_t index : model.inliers) {
      const Point &point = points[index];
      const double tangent = tangent_x * (point.x - center_x) +
                             tangent_y * (point.y - center_y);
      min_tangent = std::min(min_tangent, tangent);
      max_tangent = std::max(max_tangent, tangent);
      mean_z += point.z;
      const double residual = model.nx * point.x + model.ny * point.y - model.offset;
      residual_squared_sum += residual * residual;
      tangent_squared_sum += tangent * tangent;
    }

    if (tangent_squared_sum < 1.0e-12) {
      return std::nullopt;
    }
    mean_z /= static_cast<double>(model.inliers.size());
    double tangent_z_sum = 0.0;
    for (const std::size_t index : model.inliers) {
      const Point &point = points[index];
      const double tangent = tangent_x * (point.x - center_x) +
                             tangent_y * (point.y - center_y);
      tangent_z_sum += tangent * (point.z - mean_z);
    }
    const double z_along_tangent = tangent_z_sum / tangent_squared_sum;
    double min_vertical = std::numeric_limits<double>::infinity();
    double max_vertical = -std::numeric_limits<double>::infinity();
    for (const std::size_t index : model.inliers) {
      const Point &point = points[index];
      const double tangent = tangent_x * (point.x - center_x) +
                             tangent_y * (point.y - center_y);
      const double vertical = point.z - z_along_tangent * tangent;
      min_vertical = std::min(min_vertical, vertical);
      max_vertical = std::max(max_vertical, vertical);
    }

    const double horizontal_span = max_tangent - min_tangent;
    // A ground return can look line-like in x-y. Removing the best z-vs-line
    // slope before measuring height rejects that horizontal plane even when
    // the robot is pitched, while retaining the vertical extent of a wall.
    const double vertical_span = max_vertical - min_vertical;
    if (horizontal_span < min_horizontal_span_m_ ||
        vertical_span < min_vertical_span_m_) {
      return std::nullopt;
    }

    const auto degrees_of_freedom =
        std::max<std::size_t>(1, model.inliers.size() - 2);
    const double residual_rms =
        std::sqrt(residual_squared_sum / static_cast<double>(degrees_of_freedom));
    const double angle_stddev_deg =
        degrees(std::atan2(residual_rms, std::sqrt(tangent_squared_sum)));
    if (!std::isfinite(angle_stddev_deg)) {
      return std::nullopt;
    }

    return Detection{angle_rad, std::abs(model.offset), angle_stddev_deg,
                     horizontal_span, vertical_span, model.inliers.size()};
  }

  DetectionSearch detect(const std::vector<Point> &points,
                         const int iterations) const {
    DetectionSearch search;
    if (points.size() < static_cast<std::size_t>(min_inliers_)) {
      return search;
    }

    // Keeping the sampling sequence fixed removes an avoidable source of
    // scan-to-scan angle jitter.  The input points change naturally each scan;
    // RANSAC need not be seeded from the timestamp as well.
    std::mt19937_64 random(0x9e3779b97f4a7c15ULL);
    std::uniform_int_distribution<std::size_t> choose(0, points.size() - 1);
    std::optional<Detection> best;
    for (int iteration = 0; iteration < iterations; ++iteration) {
      const std::size_t first = choose(random);
      std::size_t second = choose(random);
      if (first == second) {
        continue;
      }
      const double dx = points[second].x - points[first].x;
      const double dy = points[second].y - points[first].y;
      const double length = std::hypot(dx, dy);
      if (length < 1.0e-4) {
        continue;
      }
      const double nx = -dy / length;
      const double ny = dx / length;
      const double offset = nx * points[first].x + ny * points[first].y;
      LineModel model = collectInliers(points, nx, ny, offset,
                                       ransac_distance_threshold_m_);
      search.best_candidate_inlier_count = std::max(
          search.best_candidate_inlier_count, model.inliers.size());
      if (model.inliers.size() < static_cast<std::size_t>(min_inliers_)) {
        continue;
      }
      const auto candidate = makeDetection(points, std::move(model));
      if (!candidate) {
        continue;
      }

      // A scene can contain several vertical surfaces. Selecting the nearest
      // two-point RANSAC hypothesis makes sparse, incidental plane fragments
      // replace the actual front wall from one scan to the next. Prefer the
      // plane with the strongest geometric support; use range only to break a
      // support tie.
      if (!best || candidate->inlier_count > best->inlier_count ||
          (candidate->inlier_count == best->inlier_count &&
           candidate->distance_m < best->distance_m)) {
        best = candidate;
      }
    }
    search.detection = best;
    return search;
  }

  static double fusionWeight(const Detection &detection) {
    // The RANSAC residual estimate is useful, but it can become unrealistically
    // small for repeated points in a submap. A 0.25 degree floor keeps one
    // exceptionally clean-looking fit from completely overriding the other.
    const double sigma_deg = std::max(0.25, detection.angle_stddev_deg);
    return std::sqrt(static_cast<double>(detection.inlier_count)) / sigma_deg;
  }

  static Detection fuseDetections(const Detection &raw,
                                  const Detection &submap) {
    const double raw_weight = fusionWeight(raw);
    const double submap_weight = fusionWeight(submap);
    const double total_weight = raw_weight + submap_weight;
    Detection fused = raw;
    fused.angle_rad = std::atan2(
        raw_weight * std::sin(raw.angle_rad) +
            submap_weight * std::sin(submap.angle_rad),
        raw_weight * std::cos(raw.angle_rad) +
            submap_weight * std::cos(submap.angle_rad));
    fused.distance_m = (raw_weight * raw.distance_m +
                        submap_weight * submap.distance_m) / total_weight;
    fused.angle_stddev_deg = std::max(0.10, 1.0 / std::sqrt(total_weight));
    return fused;
  }

  static void initializeFusionDiagnostics(msg::WallAngle &result) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    result.estimate_source = msg::WallAngle::SOURCE_NONE;
    result.raw_detection_reason = msg::WallAngle::REASON_NO_QUALIFIED_PLANE;
    result.raw_detected = false;
    result.raw_relative_angle_deg = nan;
    result.raw_distance_m = nan;
    result.raw_angle_stddev_deg = nan;
    result.raw_inlier_count = 0;
    result.submap_detected = false;
    result.submap_relative_angle_deg = nan;
    result.submap_distance_m = nan;
    result.submap_angle_stddev_deg = nan;
    result.submap_inlier_count = 0;
    result.submap_point_count = 0;
    result.initialization_complete = false;
    result.initialization_angle_error_deg = nan;
    result.initialization_sample_count = 0;
    result.initialization_required_sample_count = 0;
    result.initialization_tolerance_deg = nan;
  }

  static void setRawDiagnostics(msg::WallAngle &result,
                                const std::optional<Detection> &detection,
                                const std::uint8_t reason) {
    result.raw_detection_reason = reason;
    if (!detection) {
      return;
    }
    result.raw_detected = true;
    result.raw_relative_angle_deg = degrees(-detection->angle_rad);
    result.raw_distance_m = detection->distance_m;
    result.raw_angle_stddev_deg = detection->angle_stddev_deg;
    result.raw_inlier_count = static_cast<std::uint32_t>(detection->inlier_count);
  }

  static void setSubmapDiagnostics(msg::WallAngle &result,
                                   const std::optional<Detection> &detection,
                                   const std::size_t point_count) {
    result.submap_point_count = static_cast<std::uint32_t>(point_count);
    if (!detection) {
      return;
    }
    result.submap_detected = true;
    result.submap_relative_angle_deg = degrees(-detection->angle_rad);
    result.submap_distance_m = detection->distance_m;
    result.submap_angle_stddev_deg = detection->angle_stddev_deg;
    result.submap_inlier_count =
        static_cast<std::uint32_t>(detection->inlier_count);
  }

  void publishInvalid(const std_msgs::msg::Header &header,
                      const std::uint8_t reason,
                      const std::size_t roi_point_count = 0,
                      const std::size_t best_candidate_inlier_count = 0) {
    msg::WallAngle result;
    result.header = header;
    result.header.frame_id = base_frame_;
    result.detected = false;
    result.detection_reason = reason;
    initializeFusionDiagnostics(result);
    result.raw_detection_reason = reason;
    setInitializationDiagnostics(result);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    result.relative_angle_rad = nan;
    result.relative_angle_deg = nan;
    result.distance_m = nan;
    result.angle_stddev_deg = nan;
    result.inlier_count = 0;
    result.horizontal_span_m = nan;
    result.vertical_span_m = nan;
    result.roi_point_count = static_cast<std::uint32_t>(roi_point_count);
    result.best_candidate_inlier_count =
        static_cast<std::uint32_t>(best_candidate_inlier_count);
    result.required_inlier_count = static_cast<std::uint32_t>(min_inliers_);
    publisher_->publish(result);
  }

  void publishRoiCloud(const std_msgs::msg::Header &header,
                       const std::vector<Point> &points) {
    sensor_msgs::msg::PointCloud2 result;
    result.header = header;
    result.header.frame_id = base_frame_;
    sensor_msgs::PointCloud2Modifier modifier(result);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> x(result, "x");
    sensor_msgs::PointCloud2Iterator<float> y(result, "y");
    sensor_msgs::PointCloud2Iterator<float> z(result, "z");
    for (const Point &point : points) {
      *x = static_cast<float>(point.x);
      *y = static_cast<float>(point.y);
      *z = static_cast<float>(point.z);
      ++x;
      ++y;
      ++z;
    }
    roi_cloud_publisher_->publish(result);
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
    if (cloud->header.frame_id.empty() || !hasField(*cloud, "x") ||
        !hasField(*cloud, "y") || !hasField(*cloud, "z")) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Cloud has no frame_id or x/y/z fields");
      publishInvalid(cloud->header, msg::WallAngle::REASON_INVALID_CLOUD);
      return;
    }

    tf2::Transform to_base;
    to_base.setIdentity();
    try {
      if (cloud->header.frame_id != base_frame_) {
        const auto transform = tf_buffer_.lookupTransform(
            base_frame_, cloud->header.frame_id, cloud->header.stamp,
            tf2::durationFromSec(transform_timeout_sec_));
        const auto &translation = transform.transform.translation;
        const auto &rotation = transform.transform.rotation;
        to_base.setOrigin(
            tf2::Vector3(translation.x, translation.y, translation.z));
        to_base.setRotation(
            tf2::Quaternion(rotation.x, rotation.y, rotation.z, rotation.w));
      }
    } catch (const tf2::TransformException &error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Cannot transform %s to %s: %s",
                           cloud->header.frame_id.c_str(), base_frame_.c_str(),
                           error.what());
      publishInvalid(cloud->header, msg::WallAngle::REASON_TF_UNAVAILABLE);
      return;
    }
    const tf2::Transform from_base = to_base.inverse();

    std::vector<Point> points;
    points.reserve(static_cast<std::size_t>(cloud->width) * cloud->height);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*cloud, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
          continue;
        }
        const tf2::Vector3 transformed =
            to_base * tf2::Vector3(*x, *y, *z);
        const Point point{transformed.x(), transformed.y(), transformed.z()};
        if (!isInRoi(point)) {
          continue;
        }
        points.push_back(point);
      }
    } catch (const std::runtime_error &error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Cannot read PointCloud2 x/y/z fields: %s", error.what());
      publishInvalid(cloud->header, msg::WallAngle::REASON_CLOUD_READ_ERROR);
      return;
    }

    // Publish every point that passed the same base_link ROI used by RANSAC.
    // This happens before estimator-only downsampling so RViz shows the true
    // configured region rather than an implementation detail of the fit.
    publishRoiCloud(cloud->header, points);

    const std::size_t roi_point_count = points.size();
    const rclcpp::Time stamp(cloud->header.stamp);
    const std::vector<Point> raw_points = uniformlySamplePoints(
        points, static_cast<std::size_t>(max_cloud_points_));
    const std::vector<Point> submap_points =
        collectSubmapPoints(to_base, stamp, cloud->header.frame_id);

    DetectionSearch raw_search;
    std::uint8_t raw_reason = msg::WallAngle::REASON_INSUFFICIENT_ROI_POINTS;
    if (raw_points.size() >= static_cast<std::size_t>(min_inliers_)) {
      raw_search = detect(raw_points, ransac_iterations_);
      raw_reason = raw_search.detection ? msg::WallAngle::REASON_DETECTED
                                        : msg::WallAngle::REASON_NO_QUALIFIED_PLANE;
    }

    DetectionSearch submap_search;
    if (submap_points.size() >= static_cast<std::size_t>(min_inliers_)) {
      submap_search = detect(submap_points, submap_ransac_iterations_);
    }

    std::optional<Detection> final_detection;
    std::uint8_t source = msg::WallAngle::SOURCE_NONE;
    if (raw_search.detection && submap_search.detection) {
      const double angle_difference = std::abs(wrappedAngleDifference(
          raw_search.detection->angle_rad, submap_search.detection->angle_rad));
      const double distance_difference = std::abs(raw_search.detection->distance_m -
                                                  submap_search.detection->distance_m);
      if (angle_difference <= fusion_max_angle_difference_rad_ &&
          distance_difference <= fusion_max_distance_difference_m_) {
        final_detection = fuseDetections(*raw_search.detection,
                                         *submap_search.detection);
        source = msg::WallAngle::SOURCE_FUSED;
      } else {
        // A local submap necessarily contains the previous wall for a short
        // time. On a real wall hand-over, let this scan win instead of
        // averaging two different physical surfaces.
        final_detection = raw_search.detection;
        source = msg::WallAngle::SOURCE_RAW_CONFLICT;
      }
    } else if (raw_search.detection) {
      final_detection = raw_search.detection;
      source = msg::WallAngle::SOURCE_RAW;
    } else if (submap_search.detection) {
      final_detection = submap_search.detection;
      source = msg::WallAngle::SOURCE_SUBMAP;
    }

    msg::WallAngle result;
    result.header = cloud->header;
    result.header.frame_id = base_frame_;
    initializeFusionDiagnostics(result);
    setRawDiagnostics(result, raw_search.detection, raw_reason);
    setSubmapDiagnostics(result, submap_search.detection, submap_points.size());
    result.roi_point_count = static_cast<std::uint32_t>(roi_point_count);
    result.best_candidate_inlier_count =
        static_cast<std::uint32_t>(raw_search.best_candidate_inlier_count);
    result.required_inlier_count = static_cast<std::uint32_t>(min_inliers_);

    if (final_detection) {
      result.estimate_source = source;
      updateInitialization(final_detection->angle_rad, stamp);
    }
    setInitializationDiagnostics(result);
    publishSlamAngle(from_base);

    if (final_detection && initialization_complete_) {
      result.detected = true;
      result.detection_reason = msg::WallAngle::REASON_DETECTED;
      // SOURCE_RAW_CONFLICT remains visible to the operator, but must not
      // bypass FINAL stabilization. A true new wall persists in subsequent
      // scans and is followed smoothly; a one-scan conflict cannot jump FINAL.
      const double filtered_angle_rad = filterAngle(final_detection->angle_rad, stamp);
      // The public interface defines positive as a wall normal to the robot's
      // right. The fitted base_link y axis is left-positive, so invert here
      // while keeping the internal geometry and temporal filter unchanged.
      result.relative_angle_rad = -filtered_angle_rad;
      result.relative_angle_deg = degrees(-filtered_angle_rad);
      result.distance_m = final_detection->distance_m;
      result.angle_stddev_deg = final_detection->angle_stddev_deg;
      result.inlier_count =
          static_cast<std::uint32_t>(final_detection->inlier_count);
      result.horizontal_span_m = final_detection->horizontal_span_m;
      result.vertical_span_m = final_detection->vertical_span_m;
    } else {
      const double nan = std::numeric_limits<double>::quiet_NaN();
      result.detected = false;
      result.detection_reason = initialization_complete_
          ? raw_reason
          : msg::WallAngle::REASON_INITIALIZING;
      result.relative_angle_rad = nan;
      result.relative_angle_deg = nan;
      result.distance_m = nan;
      result.angle_stddev_deg = nan;
      result.inlier_count = 0;
      result.horizontal_span_m = nan;
      result.vertical_span_m = nan;
    }
    publisher_->publish(result);
    // Add this scan only after estimating the stable value. The displayed
    // submap is therefore genuinely prior evidence, not this scan duplicated.
    appendToSubmap(from_base, stamp, cloud->header.frame_id, points);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string slam_angle_topic_;
  std::string roi_cloud_topic_;
  std::string base_frame_;
  double transform_timeout_sec_{0.05};
  double min_forward_distance_m_{0.05};
  double max_forward_distance_m_{4.00};
  double max_lateral_distance_m_{0.50};
  double min_z_m_{-1.00};
  double max_z_m_{1.50};
  int max_cloud_points_{5000};
  int ransac_iterations_{400};
  double ransac_distance_threshold_m_{0.025};
  int min_inliers_{250};
  double min_horizontal_span_m_{0.70};
  double min_vertical_span_m_{0.12};
  double max_abs_relative_angle_rad_{degreesToRadians(70.0)};
  double angle_filter_time_constant_sec_{0.45};
  double angle_filter_reset_sec_{0.50};
  double angle_filter_max_rate_rad_per_sec_{degreesToRadians(8.0)};
  bool enable_submap_fusion_{true};
  double submap_history_sec_{0.75};
  int submap_points_per_scan_{600};
  int submap_max_points_{5000};
  int submap_ransac_iterations_{160};
  double fusion_max_angle_difference_rad_{degreesToRadians(3.0)};
  double fusion_max_distance_difference_m_{0.20};
  bool require_initial_alignment_{true};
  double initialization_tolerance_rad_{degreesToRadians(1.0)};
  int initialization_min_frames_{10};
  double initialization_window_sec_{1.0};
  bool initialization_complete_{false};
  std::deque<InitializationSample> initialization_samples_;
  std::optional<double> initialization_error_rad_;
  std::optional<rclcpp::Time> last_initialization_stamp_;
  std::optional<double> filtered_angle_rad_;
  std::optional<rclcpp::Time> last_angle_stamp_;
  std::deque<SubmapScan> submap_scans_;
  std::string submap_source_frame_;
  std::optional<rclcpp::Time> last_submap_stamp_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<msg::WallAngle>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr slam_angle_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr roi_cloud_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

}  // namespace autonomy_light

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomy_light::FrontWallAngleEstimator>());
  rclcpp::shutdown();
  return 0;
}
