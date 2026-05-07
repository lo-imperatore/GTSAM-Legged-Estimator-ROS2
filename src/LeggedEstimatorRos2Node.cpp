/* ----------------------------------------------------------------------------
 * Copyright (c) 2026, Chiyun Noh, RPM Robotics Lab, Seoul National University
 * All Rights Reserved
 *
 * This file is licensed under the BSD-3-Clause License.
 * Portions of this file are based on GTSAM legged-estimation examples, which
 * are licensed under the GTSAM BSD license.
 *
 * GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/typesupport_helpers.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <new>
#include <optional>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "include/LeggedEstimatorCore.h"

namespace gtsam {
namespace {

namespace fs = std::filesystem;
namespace introspection = rosidl_typesupport_introspection_cpp;

using sensor_msgs::msg::Imu;
using sensor_msgs::msg::JointState;

constexpr const char* kAnsiReset = "\033[0m";
constexpr const char* kAnsiBold = "\033[1m";
constexpr const char* kAnsiCyan = "\033[36m";
constexpr const char* kAnsiGreen = "\033[32m";
constexpr const char* kAnsiYellow = "\033[33m";
constexpr const char* kAnsiMagenta = "\033[35m";
constexpr const char* kAnsiBlue = "\033[34m";

const char* variantLogColor(size_t index) {
  static constexpr std::array<const char*, 4> kColors{
      kAnsiGreen, kAnsiCyan, kAnsiYellow, kAnsiMagenta};
  return kColors[index % kColors.size()];
}

std::string coloredVariantName(const std::string& filterName, size_t index) {
  return std::string(kAnsiBold) + variantLogColor(index) + filterName +
         kAnsiReset;
}

std::string coloredVariantList(const std::vector<std::string>& filterNames) {
  std::ostringstream output;
  for (size_t index = 0; index < filterNames.size(); ++index) {
    if (index > 0) {
      output << ", ";
    }
    output << coloredVariantName(filterNames[index], index);
  }
  return output.str();
}

struct LiveEvent {
  enum class Type { kImu, kContact };

  Type type = Type::kImu;
  double timestampS = 0.0;
  size_t sequence = 0;
  ImuSample imu;
  ContactEvent contact;
};

struct QueuedEvent {
  double timestampS = 0.0;
  int priority = 0;
  size_t sequence = 0;
  LiveEvent event;
};

struct QueuedEventGreater {
  bool operator()(const QueuedEvent& lhs, const QueuedEvent& rhs) const {
    if (std::abs(lhs.timestampS - rhs.timestampS) > 1e-12) {
      return lhs.timestampS > rhs.timestampS;
    }
    if (lhs.priority != rhs.priority) {
      return lhs.priority > rhs.priority;
    }
    return lhs.sequence > rhs.sequence;
  }
};

enum class ContactPacketStatus {
  kIgnored,
  kUsedTouchdown,
  kUsedPeriodic,
};

const introspection::MessageMember* findMember(
    const introspection::MessageMembers* members, const std::string& name) {
  for (uint32_t index = 0; index < members->member_count_; ++index) {
    const introspection::MessageMember& member = members->members_[index];
    if (member.name_ == name) {
      return &member;
    }
  }
  return nullptr;
}

const void* fieldPtr(const void* message,
                     const introspection::MessageMember* member) {
  return static_cast<const uint8_t*>(message) + member->offset_;
}

const introspection::MessageMembers* nestedMembers(
    const introspection::MessageMember* member) {
  if (member == nullptr || member->type_id_ != introspection::ROS_TYPE_MESSAGE ||
      member->members_ == nullptr || member->members_->data == nullptr) {
    throw std::runtime_error("Expected nested ROS message field");
  }
  return static_cast<const introspection::MessageMembers*>(
      member->members_->data);
}

double readNumericField(const void* message,
                        const introspection::MessageMembers* members,
                        const std::string& name) {
  const introspection::MessageMember* member = findMember(members, name);
  if (member == nullptr) {
    throw std::runtime_error("Missing numeric field: " + name);
  }
  const void* ptr = fieldPtr(message, member);
  switch (member->type_id_) {
    case introspection::ROS_TYPE_FLOAT:
      return static_cast<double>(*static_cast<const float*>(ptr));
    case introspection::ROS_TYPE_DOUBLE:
      return *static_cast<const double*>(ptr);
    case introspection::ROS_TYPE_UINT8:
    case introspection::ROS_TYPE_OCTET:
      return static_cast<double>(*static_cast<const uint8_t*>(ptr));
    case introspection::ROS_TYPE_INT8:
      return static_cast<double>(*static_cast<const int8_t*>(ptr));
    case introspection::ROS_TYPE_UINT16:
      return static_cast<double>(*static_cast<const uint16_t*>(ptr));
    case introspection::ROS_TYPE_INT16:
      return static_cast<double>(*static_cast<const int16_t*>(ptr));
    case introspection::ROS_TYPE_UINT32:
      return static_cast<double>(*static_cast<const uint32_t*>(ptr));
    case introspection::ROS_TYPE_INT32:
      return static_cast<double>(*static_cast<const int32_t*>(ptr));
    case introspection::ROS_TYPE_UINT64:
      return static_cast<double>(*static_cast<const uint64_t*>(ptr));
    case introspection::ROS_TYPE_INT64:
      return static_cast<double>(*static_cast<const int64_t*>(ptr));
    default:
      throw std::runtime_error("Field is not numeric: " + name);
  }
}

double stampToSec(const void* message,
                  const introspection::MessageMembers* members) {
  const introspection::MessageMember* headerMember = findMember(members, "header");
  if (headerMember == nullptr) {
    throw std::runtime_error("FootStateArray is missing header");
  }
  const void* header = fieldPtr(message, headerMember);
  const introspection::MessageMembers* headerMembers = nestedMembers(headerMember);
  const introspection::MessageMember* stampMember =
      findMember(headerMembers, "stamp");
  if (stampMember == nullptr) {
    throw std::runtime_error("Header is missing stamp");
  }
  const void* stamp = fieldPtr(header, stampMember);
  const introspection::MessageMembers* stampMembers = nestedMembers(stampMember);
  const double sec = readNumericField(stamp, stampMembers, "sec");
  const double nanosec = readNumericField(stamp, stampMembers, "nanosec");
  return sec + nanosec * 1e-9;
}

class DynamicRosMessage {
 public:
  explicit DynamicRosMessage(const introspection::MessageMembers* members)
      : members_(members) {
    data_ = ::operator new(members_->size_of_);
    members_->init_function(data_,
                            rosidl_runtime_cpp::MessageInitialization::ALL);
  }

  ~DynamicRosMessage() {
    if (data_ != nullptr) {
      members_->fini_function(data_);
      ::operator delete(data_);
    }
  }

  DynamicRosMessage(const DynamicRosMessage&) = delete;
  DynamicRosMessage& operator=(const DynamicRosMessage&) = delete;

  void* data() { return data_; }
  const void* data() const { return data_; }

 private:
  const introspection::MessageMembers* members_ = nullptr;
  void* data_ = nullptr;
};

std::vector<double> checkedDoubleVector(const std::vector<double>& value,
                                        size_t expectedSize,
                                        const std::string& name) {
  if (value.size() != expectedSize) {
    throw std::runtime_error(name + " must contain " +
                             std::to_string(expectedSize) + " values");
  }
  return value;
}

std::vector<double> checkedQuaternionXyzw(const std::vector<double>& value,
                                          const std::string& name) {
  const std::vector<double> quaternion = checkedDoubleVector(value, 4, name);
  double squaredNorm = 0.0;
  for (const double component : quaternion) {
    if (!std::isfinite(component)) {
      throw std::runtime_error(name + " must contain finite values");
    }
    squaredNorm += component * component;
  }
  if (squaredNorm <= 1e-24) {
    throw std::runtime_error(name + " must not be a zero quaternion");
  }
  return quaternion;
}

class VariantRunner {
 public:
  using StateCallback =
      std::function<void(const std::string&, double, const NavState&)>;
  using ContactCallback =
      std::function<void(const std::string&, double, const ExtendedPose3d&,
                         const std::vector<ContactMeasurement>&)>;

  VariantRunner(std::string filterName, DatasetMetadata metadata,
                ReplayConfig replayConfig, double maxDurationSeconds,
                bool disableFullContactInitialization,
                StateCallback stateCallback, ContactCallback contactCallback)
      : filterName_(std::move(filterName)),
        metadata_(std::move(metadata)),
        replayConfig_(std::move(replayConfig)),
        maxDurationSeconds_(maxDurationSeconds),
        disableFullContactInitialization_(disableFullContactInitialization),
        stateCallback_(std::move(stateCallback)),
        contactCallback_(std::move(contactCallback)),
        initialState_(makeInitialState(replayConfig_)),
        previousLoggedState_(initialState_) {}

  void start(const fs::path& outputDir,
             const imuBias::ConstantBias& imuBiasEstimate) {
    const Matrix footholds = makeInitialFootholds(metadata_.footNames.size());
    const Matrix covariance =
        makeInitialCovariance(metadata_.footNames.size(), replayConfig_);
    const Matrix9 baseCovariance = makeInitialBaseCovariance(replayConfig_);
    params_ = makeParams(replayConfig_, disableFullContactInitialization_,
                         imuBiasEstimate);
    estimator_ = makeEstimator(filterName_, metadata_.footNames, initialState_,
                               footholds, covariance, baseCovariance, params_,
                               replayConfig_.lagSeconds);

    fs::create_directories(outputDir);
    trajectoryOutput_.open(outputDir / (filterName_ + "_trajectory.csv"));
    if (!trajectoryOutput_) {
      throw std::runtime_error("Unable to open trajectory output for writing.");
    }
    writeTrajectoryHeader(trajectoryOutput_);

    tumTrajectoryOutput_.open(outputDir / (filterName_ + "_trajectory_imu.tum"));
    if (!tumTrajectoryOutput_) {
      throw std::runtime_error(
          "Unable to open TUM trajectory output for writing.");
    }

    metricsOutput_.open(outputDir / (filterName_ + "_metrics.csv"));
    if (!metricsOutput_) {
      throw std::runtime_error("Unable to open metrics output for writing.");
    }
    writeMetricsHeader(metricsOutput_);

    previousLoggedState_ = navStateFromEstimate(estimator_->estimate());
    havePreviousLoggedState_ = false;
    loggingEnabled_ = !params_.useFullContactInitialization;
    startWallTime_ = std::chrono::steady_clock::now();
  }

  void processEvent(const LiveEvent& event) {
    if (event.type == LiveEvent::Type::kImu) {
      processImu(event.imu);
    } else {
      processContact(event.contact);
    }
  }

  void finish() {
    if (!estimator_ || finished_) {
      return;
    }
    finished_ = true;

    const auto endWallTime = std::chrono::steady_clock::now();
    metrics_.wallTimeMs =
        std::chrono::duration<double, std::milli>(endWallTime - startWallTime_)
            .count();
    if (metrics_.contactMeasurements > 0) {
      metrics_.meanContactResidual /=
          static_cast<double>(metrics_.contactMeasurements);
      metrics_.rmsContactResidual =
          std::sqrt(metrics_.rmsContactResidual /
                    static_cast<double>(metrics_.contactMeasurements));
      metrics_.meanHeightAbsError /=
          static_cast<double>(metrics_.contactMeasurements);
    }
    metrics_.trajectoryRows = trajectoryRows_;
    metrics_.loopClosureError =
        (navStateFromEstimate(estimator_->estimate()).position() -
         initialState_.position())
            .norm();

    const ReplayOutputs outputs{
        filterName_, metrics_, navStateFromEstimate(estimator_->estimate()), {}};
    writeMetricsRow(metricsOutput_, outputs);
  }

 private:
  void processImu(const ImuSample& sample) {
    if (!estimator_ || maxDurationReached_) {
      return;
    }
    if (!haveHeldImu_) {
      heldImu_ = sample;
      haveHeldImu_ = true;
      currentTime_ = sample.timestampS;
      startTimestamp_ = sample.timestampS;
      if (loggingEnabled_) {
        logState("start", currentTime_);
      }
      return;
    }

    if (sample.timestampS - startTimestamp_ > maxDurationSeconds_) {
      maxDurationReached_ = true;
      return;
    }
    advanceTo(sample.timestampS);
    heldImu_ = sample;
    logState("imu", currentTime_);
  }

  void processContact(const ContactEvent& event) {
    if (!estimator_ || !haveHeldImu_ || maxDurationReached_) {
      return;
    }
    if (event.timestampS < startTimestamp_ - 1e-12) {
      return;
    }
    if (event.timestampS - startTimestamp_ > maxDurationSeconds_) {
      maxDurationReached_ = true;
      return;
    }
    advanceTo(event.timestampS);

    ContactPacketStatus status = ContactPacketStatus::kIgnored;
    if (hasTouchdown(event.activeContacts)) {
      status = ContactPacketStatus::kUsedTouchdown;
    } else if (replayConfig_.maxDeadReckoningSeconds <= 0.0 ||
               (std::isfinite(lastContactUpdateTime_) &&
                event.timestampS - lastContactUpdateTime_ >=
                    replayConfig_.maxDeadReckoningSeconds - 1e-12)) {
      status = ContactPacketStatus::kUsedPeriodic;
    }
    if (status == ContactPacketStatus::kIgnored) {
      return;
    }
    lastContactUpdateTime_ = event.timestampS;

    estimator_->processContacts(event.activeContacts);
    const ExtendedPose3d estimate = estimator_->estimate();
    const Matrix footholds = footholdsFromEstimate(estimate);
    if (contactCallback_) {
      contactCallback_(filterName_, event.timestampS, estimate,
                       event.activeContacts);
    }
    for (const ContactMeasurement& contact : event.activeContacts) {
      const double residual =
          contactResidualNorm(estimate, replayConfig_.body_P_imu, contact);
      metrics_.meanContactResidual += residual;
      metrics_.rmsContactResidual += residual * residual;
      metrics_.maxContactResidual =
          std::max(metrics_.maxContactResidual, residual);
      metrics_.meanHeightAbsError += std::abs(
          footholds(2, static_cast<Eigen::Index>(contact.foot)) - 0.0);
      ++metrics_.contactMeasurements;
    }
    ++metrics_.contactEvents;

    if (!loggingEnabled_ && params_.useFullContactInitialization &&
        event.activeContacts.size() == metadata_.footNames.size()) {
      loggingEnabled_ = true;
    }
    logState(status == ContactPacketStatus::kUsedTouchdown
                 ? "touchdown_contact"
                 : "periodic_contact",
             currentTime_);
  }

  void advanceTo(double timestampS) {
    if (timestampS < currentTime_ - 1e-12) {
      throw std::runtime_error(
          "ROS message timestamps are not monotonically increasing.");
    }
    const double dt = timestampS - currentTime_;
    if (dt > 0.0) {
      estimator_->predict(heldImu_.omega, heldImu_.specificForce, dt);
      currentTime_ = timestampS;
    }
  }

  void logState(const std::string& kind, double timestampS) {
    if (!loggingEnabled_) {
      return;
    }
    const ExtendedPose3d estimate = estimator_->estimate();
    const NavState state = navStateFromEstimate(estimate);
    writeTrajectoryRow(trajectoryOutput_, kind, timestampS, state);
    writeTumTrajectoryRow(tumTrajectoryOutput_, timestampS, state);

    if (havePreviousLoggedState_) {
      metrics_.pathLength +=
          (state.position() - previousLoggedState_.position()).norm();
    }
    previousLoggedState_ = state;
    havePreviousLoggedState_ = true;
    ++trajectoryRows_;

    if (stateCallback_) {
      stateCallback_(filterName_, timestampS, state);
    }
  }

  std::string filterName_;
  DatasetMetadata metadata_;
  ReplayConfig replayConfig_;
  double maxDurationSeconds_;
  bool disableFullContactInitialization_ = false;
  StateCallback stateCallback_;
  ContactCallback contactCallback_;

  NavState initialState_;
  LeggedEstimatorParams params_;
  std::unique_ptr<LeggedEstimator> estimator_;
  std::ofstream trajectoryOutput_;
  std::ofstream tumTrajectoryOutput_;
  std::ofstream metricsOutput_;
  std::chrono::steady_clock::time_point startWallTime_;

  ImuSample heldImu_;
  bool haveHeldImu_ = false;
  double currentTime_ = 0.0;
  double startTimestamp_ = 0.0;
  double lastContactUpdateTime_ = -std::numeric_limits<double>::infinity();
  bool loggingEnabled_ = false;
  bool havePreviousLoggedState_ = false;
  bool maxDurationReached_ = false;
  bool finished_ = false;
  size_t trajectoryRows_ = 0;
  NavState previousLoggedState_;
  ReplayMetrics metrics_;
};

class LeggedEstimatorRos2Node : public rclcpp::Node {
 public:
  explicit LeggedEstimatorRos2Node(const rclcpp::NodeOptions& options)
      : Node("GTSAM_legged_estimator", options) {
    readParameters();
    setupFootTypeSupport();
    setupRosInterfaces();

    logStartupConfiguration();
    RCLCPP_INFO(get_logger(),
                "C++ live estimator subscribed: imu=%s, foot=%s (%s)",
                imuTopic_.c_str(), footTopic_.c_str(), footType_.c_str());
  }

  ~LeggedEstimatorRos2Node() override { finish(); }

  void finish() {
    if (finished_) {
      return;
    }
    finished_ = true;
    flushEvents(std::numeric_limits<double>::infinity());
    if (!initialized_ && !startupBuffer_.empty()) {
      initializeFromStartupBuffer("shutdown");
    }
    for (std::unique_ptr<VariantRunner>& runner : runners_) {
      runner->finish();
    }
  }

 private:
  void readParameters() {
    imuTopic_ = declare_parameter<std::string>("topics.imu", "/imu");
    footTopic_ =
        declare_parameter<std::string>("topics.foot", "/spot/status/feet");
    footType_ = declare_parameter<std::string>(
        "topics.foot_type", "spot_msgs/msg/FootStateArray");
    jointStatesTopic_ =
        declare_parameter<std::string>("topics.joint_states", "/joint_states");
    readJointStates_ = declare_parameter<bool>("topics.read_joint_states", true);
    odomTopicPrefix_ =
        declare_parameter<std::string>("topics.odom_prefix", "/legged_estimator");

    frameId_ = declare_parameter<std::string>("frames.odom", "odom");
    childFrameId_ =
        declare_parameter<std::string>("frames.base_link", "base_link");

    footNames_ = declare_parameter<std::vector<std::string>>(
        "calibration.foot_names", {"fl", "fr", "hl", "hr"});
    if (footNames_.empty()) {
      throw std::runtime_error("calibration.foot_names cannot be empty");
    }

    const std::vector<double> offsets = declare_parameter<std::vector<double>>(
        "calibration.leg_imu_offsets",
        std::vector<double>(footNames_.size() * 3, 0.0));
    checkedDoubleVector(offsets, footNames_.size() * 3,
                        "calibration.leg_imu_offsets");
    legImuOffsets_.clear();
    for (size_t foot = 0; foot < footNames_.size(); ++foot) {
      legImuOffsets_.push_back(Vector3(offsets[3 * foot + 0],
                                       offsets[3 * foot + 1],
                                       offsets[3 * foot + 2]));
    }

    const std::vector<int64_t> fkIndices =
        declare_parameter<std::vector<int64_t>>(
            "calibration.spot_fk_position_indices",
            {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11});
    if (fkIndices.size() != 12) {
      throw std::runtime_error(
          "calibration.spot_fk_position_indices must contain 12 values");
    }
    spotFkPositionIndices_.clear();
    for (const int64_t index : fkIndices) {
      if (index < 0) {
        throw std::runtime_error(
            "calibration.spot_fk_position_indices cannot contain negatives");
      }
      spotFkPositionIndices_.push_back(static_cast<size_t>(index));
    }

    useJointFkForBodyPoint_ =
        declare_parameter<bool>("calibration.use_joint_fk_for_body_point", true);
    jointFkFallbackToMsgBodyPoint_ = declare_parameter<bool>(
        "calibration.joint_fk_fallback_to_msg_body_point", true);
    jointFkMaxJointAgeSeconds_ = declare_parameter<double>(
        "calibration.joint_fk_max_joint_age_seconds", 0.05);

    const std::vector<double> bodyPImuXyz =
        checkedDoubleVector(declare_parameter<std::vector<double>>(
                                "calibration.body_p_imu_xyz",
                                {0.0, 0.0, 0.0}),
                            3, "calibration.body_p_imu_xyz");
    const std::vector<double> bodyQImuXyzw =
        checkedQuaternionXyzw(declare_parameter<std::vector<double>>(
                                  "calibration.body_q_imu_xyzw",
                                  {0.0, 0.0, 0.0, 1.0}),
                              "calibration.body_q_imu_xyzw");
    replayConfig_.body_P_imu = Pose3(
        Rot3::Quaternion(bodyQImuXyzw[3], bodyQImuXyzw[0],
                         bodyQImuXyzw[1], bodyQImuXyzw[2]),
        Point3(bodyPImuXyz[0], bodyPImuXyz[1], bodyPImuXyz[2]));

    contactStreamMode_ =
        declare_parameter<std::string>("conversion.contact_stream_mode",
                                       "transition");
    contactStateValue_ =
        declare_parameter<int>("conversion.contact_state_value", 1);
    contactMadeCode_ = declare_parameter<int>("conversion.contact_made_code", 1);
    contactLostCode_ = declare_parameter<int>("conversion.contact_lost_code", 2);

    filterNames_ = declare_parameter<std::vector<std::string>>(
        "estimator.variants", {"invariant_ekf"});
    if (filterNames_.size() != 1) {
      throw std::runtime_error(
          "estimator.variants must contain exactly one variant; "
          "multi-variant live runs are disabled to keep RViz topics stable");
    }
    outputDir_ =
        declare_parameter<std::string>("estimator.output_dir", "./outputs/live");

    const double maxDuration =
        declare_parameter<double>("estimator.max_duration_seconds", 0.0);
    maxDurationSeconds_ =
        maxDuration > 0.0 ? maxDuration : std::numeric_limits<double>::infinity();
    replayConfig_.lagSeconds =
        declare_parameter<double>("estimator.lag_seconds",
                                  replayConfig_.lagSeconds);
    replayConfig_.maxDeadReckoningSeconds = declare_parameter<double>(
        "estimator.max_dead_reckoning_seconds",
        replayConfig_.maxDeadReckoningSeconds);
    replayConfig_.sigmaAcc =
        declare_parameter<double>("estimator.sigma_acc", replayConfig_.sigmaAcc);
    replayConfig_.biasAccRandomWalkSigma = declare_parameter<double>(
        "estimator.bias_acc_random_walk_sigma",
        replayConfig_.biasAccRandomWalkSigma);
    replayConfig_.biasOmegaRandomWalkSigma = declare_parameter<double>(
        "estimator.bias_omega_random_walk_sigma",
        replayConfig_.biasOmegaRandomWalkSigma);

    const double contactSigmaXY =
        declare_parameter<double>("estimator.contact_sigma_xy",
                                  std::sqrt(replayConfig_.contactCovariance(0, 0)));
    const double contactSigmaZ =
        declare_parameter<double>("estimator.contact_sigma_z",
                                  std::sqrt(replayConfig_.contactCovariance(2, 2)));
    replayConfig_.contactCovariance =
        Vector3(contactSigmaXY * contactSigmaXY,
                contactSigmaXY * contactSigmaXY,
                contactSigmaZ * contactSigmaZ)
            .asDiagonal();
    replayConfig_.useRobustContactNoise =
        declare_parameter<bool>("estimator.use_robust_contact_noise", false);
    replayConfig_.robustContactHuberK = declare_parameter<double>(
        "estimator.robust_contact_huber_k",
        replayConfig_.robustContactHuberK);
    replayConfig_.marginalizeLeavingFoot = declare_parameter<bool>(
        "estimator.marginalize_leaving_foot", true);
    disableFullContactInitialization_ = declare_parameter<bool>(
        "estimator.disable_full_contact_initialization", false);
    startupBiasWindowSeconds_ =
        declare_parameter<double>("estimator.startup_bias_window_seconds", 1.0);
    startupTimeoutSeconds_ =
        declare_parameter<double>("estimator.startup_timeout_seconds", 5.0);

    const std::string reliability =
        declare_parameter<std::string>("qos.reliability", "best_effort");
    qosDepth_ = declare_parameter<int>("qos.depth", 1000);
    qosReliable_ = reliability == "reliable";
    reorderDelaySeconds_ =
        declare_parameter<double>("qos.reorder_delay_seconds", 0.02);
    publishOdom_ = declare_parameter<bool>("publish_odom", true);
    publishPath_ = declare_parameter<bool>("publish_path", true);
    publishFootContacts_ = declare_parameter<bool>("publish_foot_contacts", true);
    footContactTouchdownsOnly_ =
        declare_parameter<bool>("foot_contact_touchdowns_only", false);
    footContactMarkerScale_ =
        declare_parameter<double>("foot_contact_marker_scale", 0.12);
    footContactMarkerZOffset_ =
        declare_parameter<double>("foot_contact_marker_z_offset", 0.03);
    footContactMaxPoints_ =
        declare_parameter<int>("foot_contact_max_points", 5000);
    if (footContactMarkerScale_ <= 0.0) {
      throw std::runtime_error("foot_contact_marker_scale must be > 0");
    }
    if (footContactMaxPoints_ < 0) {
      throw std::runtime_error("foot_contact_max_points must be >= 0");
    }
    pathPublishDecimation_ =
        declare_parameter<int>("path_publish_decimation", 10);
    if (pathPublishDecimation_ < 1) {
      throw std::runtime_error("path_publish_decimation must be >= 1");
    }
    pathMaxPoses_ = declare_parameter<int>("path_max_poses", 10000);
    if (pathMaxPoses_ < 0) {
      throw std::runtime_error("path_max_poses must be >= 0");
    }

    metadata_.footNames = footNames_;
    metadata_.denseContactStream = true;
    metadata_.timestampSource = "ros2_live";
    inContact_.assign(footNames_.size(), false);
    previousContactSet_.assign(footNames_.size(), false);
  }

  rclcpp::QoS makeQos() const {
    rclcpp::QoS qos(rclcpp::KeepLast(static_cast<size_t>(qosDepth_)));
    if (qosReliable_) {
      qos.reliable();
    } else {
      qos.best_effort();
    }
    return qos;
  }

  void setupFootTypeSupport() {
    cppTypeSupportLibrary_ =
        rclcpp::get_typesupport_library(footType_, "rosidl_typesupport_cpp");
    cppTypeSupport_ = rclcpp::get_message_typesupport_handle(
        footType_, "rosidl_typesupport_cpp", *cppTypeSupportLibrary_);
    introspectionTypeSupportLibrary_ = rclcpp::get_typesupport_library(
        footType_, "rosidl_typesupport_introspection_cpp");
    introspectionTypeSupport_ = rclcpp::get_message_typesupport_handle(
        footType_, "rosidl_typesupport_introspection_cpp",
        *introspectionTypeSupportLibrary_);
    footMembers_ = static_cast<const introspection::MessageMembers*>(
        introspectionTypeSupport_->data);
    serialization_ = std::make_unique<rclcpp::SerializationBase>(cppTypeSupport_);
  }

  void setupRosInterfaces() {
    const rclcpp::QoS qos = makeQos();
    imuSub_ = create_subscription<Imu>(
        imuTopic_, qos,
        [this](const Imu::SharedPtr msg) { handleImu(*msg); });
    if (readJointStates_) {
      jointStateSub_ = create_subscription<JointState>(
          jointStatesTopic_, qos,
          [this](const JointState::SharedPtr msg) { handleJointState(*msg); });
    }
    footSub_ = create_generic_subscription(
        footTopic_, footType_, qos,
        [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
          handleFootSerialized(msg);
        });

    const rclcpp::QoS pathQos =
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    for (size_t index = 0; index < filterNames_.size(); ++index) {
      const std::string odomTopic = outputTopic("odom");
      odomPublishers_.push_back(
          create_publisher<nav_msgs::msg::Odometry>(odomTopic, 10));

      const std::string pathTopic = outputTopic("path");
      pathPublishers_.push_back(
          create_publisher<nav_msgs::msg::Path>(pathTopic, pathQos));
      nav_msgs::msg::Path path;
      path.header.frame_id = frameId_;
      pathMessages_.push_back(std::move(path));
      pathStateCounters_.push_back(0);

      if (publishFootContacts_) {
        const std::string contactTopic = outputTopic("foot_contacts");
        footContactPublishers_.push_back(
            create_publisher<visualization_msgs::msg::MarkerArray>(
                contactTopic, pathQos));
        footContactMarkerArrays_.push_back(makeFootContactMarkerArray());
      }
    }
    flushTimer_ = create_wall_timer(
        std::chrono::milliseconds(10), [this]() { flushReadyEvents(); });
  }

  std::string outputTopic(const std::string& leafName) const {
    return odomTopicPrefix_ + "/" + leafName;
  }

  void logStartupConfiguration() const {
    const std::string exampleTopic =
        filterNames_.empty() ? odomTopicPrefix_ + "/odom"
                             : outputTopic("odom");
    RCLCPP_INFO(get_logger(), "%s%sEstimator variant%s: [%s]",
                kAnsiBold, kAnsiBlue, kAnsiReset,
                coloredVariantList(filterNames_).c_str());
    RCLCPP_INFO(get_logger(), "%s%sOutput mode%s: %s (%s example: %s)",
                kAnsiBold, kAnsiBlue, kAnsiReset, "fixed RViz topics",
                "stable", exampleTopic.c_str());
  }

  void handleImu(const Imu& msg) {
    LiveEvent event;
    event.type = LiveEvent::Type::kImu;
    event.timestampS = rclcpp::Time(msg.header.stamp).seconds();
    event.imu.timestampS = event.timestampS;
    event.imu.omega = Vector3(msg.angular_velocity.x, msg.angular_velocity.y,
                              msg.angular_velocity.z);
    event.imu.specificForce =
        Vector3(msg.linear_acceleration.x, msg.linear_acceleration.y,
                msg.linear_acceleration.z);
    enqueueEvent(std::move(event));
  }

  void handleJointState(const JointState& msg) {
    latestJointPositions_.assign(msg.position.begin(), msg.position.end());
    lastJointTimestampS_ = rclcpp::Time(msg.header.stamp).seconds();
    haveJointState_ = true;
  }

  void handleFootSerialized(
      const std::shared_ptr<rclcpp::SerializedMessage>& serialized) {
    try {
      DynamicRosMessage message(footMembers_);
      serialization_->deserialize_message(serialized.get(), message.data());
      std::optional<ContactEvent> contactEvent = parseFootMessage(message.data());
      if (!contactEvent) {
        return;
      }
      LiveEvent event;
      event.type = LiveEvent::Type::kContact;
      event.timestampS = contactEvent->timestampS;
      event.contact = std::move(*contactEvent);
      enqueueEvent(std::move(event));
    } catch (const std::exception& error) {
      if (!reportedFootParseError_) {
        reportedFootParseError_ = true;
        RCLCPP_ERROR(get_logger(), "Failed to parse %s: %s", footType_.c_str(),
                     error.what());
      }
    }
  }

  std::optional<ContactEvent> parseFootMessage(const void* message) {
    const double timestampS = stampToSec(message, footMembers_);
    const introspection::MessageMember* statesMember =
        findMember(footMembers_, "states");
    if (statesMember == nullptr || !statesMember->is_array_ ||
        statesMember->size_function == nullptr ||
        statesMember->get_const_function == nullptr) {
      throw std::runtime_error("FootStateArray.states is not a sequence");
    }

    const void* statesField = fieldPtr(message, statesMember);
    const size_t usable =
        std::min(statesMember->size_function(statesField), footNames_.size());
    const introspection::MessageMembers* stateMembers =
        nestedMembers(statesMember);

    std::vector<bool> currentContactSet(footNames_.size(), false);
    std::vector<ContactMeasurement> activeContacts;

    for (size_t foot = 0; foot < usable; ++foot) {
      const void* state = statesMember->get_const_function(statesField, foot);
      const int contactCode =
          static_cast<int>(readNumericField(state, stateMembers, "contact"));
      if (contactStreamMode_ == "state") {
        inContact_[foot] = contactCode == contactStateValue_;
      } else {
        if (contactCode == contactMadeCode_) {
          inContact_[foot] = true;
        } else if (contactCode == contactLostCode_) {
          inContact_[foot] = false;
        }
      }
      if (!inContact_[foot]) {
        continue;
      }

      currentContactSet[foot] = true;
      Vector3 bodyPoint = readFootBodyPoint(state, stateMembers);
      if (useJointFkForBodyPoint_) {
        const std::optional<Vector3> fk =
            computeSpotBodyPointFromJointState(foot, timestampS);
        if (fk) {
          bodyPoint = *fk;
        } else if (!jointFkFallbackToMsgBodyPoint_) {
          continue;
        }
      }
      bodyPoint += legImuOffsets_.at(foot);

      ContactMeasurement measurement;
      measurement.foot = foot;
      measurement.bodyPoint = bodyPoint;
      activeContacts.push_back(measurement);
    }

    if (activeContacts.empty()) {
      return std::nullopt;
    }

    const bool contactSetChanged =
        !havePreviousContactSet_ || currentContactSet != previousContactSet_;
    for (ContactMeasurement& measurement : activeContacts) {
      measurement.touchdown = contactSetChanged;
    }
    previousContactSet_ = std::move(currentContactSet);
    havePreviousContactSet_ = true;

    ContactEvent event;
    event.index = nextContactIndex_++;
    event.timestampS = timestampS;
    event.activeContacts = std::move(activeContacts);
    return event;
  }

  Vector3 readFootBodyPoint(const void* state,
                            const introspection::MessageMembers* stateMembers) {
    const introspection::MessageMember* pointMember =
        findMember(stateMembers, "foot_position_rt_body");
    if (pointMember == nullptr) {
      throw std::runtime_error(
          "FootState is missing foot_position_rt_body field");
    }
    const void* point = fieldPtr(state, pointMember);
    const introspection::MessageMembers* pointMembers =
        nestedMembers(pointMember);
    return Vector3(readNumericField(point, pointMembers, "x"),
                   readNumericField(point, pointMembers, "y"),
                   readNumericField(point, pointMembers, "z"));
  }

  std::optional<Vector3> computeSpotBodyPointFromJointState(
      size_t foot, double measurementTimestampS) const {
    static const std::array<std::array<Vector3, 4>, 4> kSpotLegChains{{
        {Vector3(0.0, 0.055, 0.0), Vector3(0.0, 0.110945, 0.0),
         Vector3(0.025, 0.0, -0.3205), Vector3(0.0, 0.0, -0.34)},
        {Vector3(0.0, -0.055, 0.0), Vector3(0.0, -0.110945, 0.0),
         Vector3(0.025, 0.0, -0.3205), Vector3(0.0, 0.0, -0.34)},
        {Vector3(-0.5957, 0.055, 0.0), Vector3(0.0, 0.110945, 0.0),
         Vector3(0.025, 0.0, -0.3205), Vector3(0.0, 0.0, -0.34)},
        {Vector3(-0.5957, -0.055, 0.0), Vector3(0.0, -0.110945, 0.0),
         Vector3(0.025, 0.0, -0.3205), Vector3(0.0, 0.0, -0.34)}}};

    if (foot >= kSpotLegChains.size() || !haveJointState_) {
      return std::nullopt;
    }
    if (jointFkMaxJointAgeSeconds_ > 0.0 &&
        std::abs(measurementTimestampS - lastJointTimestampS_) >
            jointFkMaxJointAgeSeconds_) {
      return std::nullopt;
    }

    const size_t i0 = spotFkPositionIndices_.at(3 * foot + 0);
    const size_t i1 = spotFkPositionIndices_.at(3 * foot + 1);
    const size_t i2 = spotFkPositionIndices_.at(3 * foot + 2);
    if (i0 >= latestJointPositions_.size() ||
        i1 >= latestJointPositions_.size() ||
        i2 >= latestJointPositions_.size()) {
      return std::nullopt;
    }

    const Rot3 a12 = Rot3::Rx(latestJointPositions_[i0]);
    const Rot3 a23 = Rot3::Ry(latestJointPositions_[i1]);
    const Rot3 a34 = Rot3::Ry(latestJointPositions_[i2]);
    const Rot3 a123 = a12 * a23;
    const Rot3 a1234 = a123 * a34;
    const auto& chain = kSpotLegChains[foot];
    return a1234.matrix() * chain[3] + a123.matrix() * chain[2] +
           a12.matrix() * chain[1] + chain[0];
  }

  void enqueueEvent(LiveEvent event) {
    std::lock_guard<std::mutex> lock(queueMutex_);
    event.sequence = nextSequence_++;
    if (event.type == LiveEvent::Type::kImu) {
      event.imu.index = nextImuIndex_++;
    }
    latestQueuedTimestampS_ =
        std::max(latestQueuedTimestampS_, event.timestampS);
    eventQueue_.push(QueuedEvent{event.timestampS,
                                 event.type == LiveEvent::Type::kImu ? 0 : 1,
                                 event.sequence, std::move(event)});
  }

  void flushReadyEvents() {
    flushEvents(latestQueuedTimestampS_ - reorderDelaySeconds_);
  }

  void flushEvents(double cutoffTimestampS) {
    std::vector<LiveEvent> ready;
    {
      std::lock_guard<std::mutex> lock(queueMutex_);
      while (!eventQueue_.empty() &&
             eventQueue_.top().timestampS <= cutoffTimestampS) {
        ready.push_back(eventQueue_.top().event);
        eventQueue_.pop();
      }
    }

    for (LiveEvent& event : ready) {
      if (event.timestampS < lastDispatchedTimestampS_ - 1e-9) {
        RCLCPP_WARN(get_logger(), "Dropping out-of-order event at %.9f",
                    event.timestampS);
        continue;
      }
      lastDispatchedTimestampS_ = event.timestampS;
      handleOrderedEvent(std::move(event));
    }
  }

  void handleOrderedEvent(LiveEvent event) {
    latestEstimatorTimestampS_ =
        std::max(latestEstimatorTimestampS_, event.timestampS);
    if (!haveFirstTimestamp_) {
      firstTimestampS_ = event.timestampS;
      haveFirstTimestamp_ = true;
    }

    if (initialized_) {
      dispatch(event);
      return;
    }

    startupBuffer_.push_back(event);
    if (readyToInitialize()) {
      initializeFromStartupBuffer("startup window");
    }
  }

  Dataset makeDatasetFromStartupBuffer() const {
    Dataset dataset;
    dataset.metadata = metadata_;

    std::vector<LiveEvent> events = startupBuffer_;
    std::stable_sort(events.begin(), events.end(),
                     [](const LiveEvent& lhs, const LiveEvent& rhs) {
                       if (std::abs(lhs.timestampS - rhs.timestampS) > 1e-12) {
                         return lhs.timestampS < rhs.timestampS;
                       }
                       const int lhsPriority =
                           lhs.type == LiveEvent::Type::kImu ? 0 : 1;
                       const int rhsPriority =
                           rhs.type == LiveEvent::Type::kImu ? 0 : 1;
                       if (lhsPriority != rhsPriority) {
                         return lhsPriority < rhsPriority;
                       }
                       return lhs.sequence < rhs.sequence;
                     });

    for (const LiveEvent& event : events) {
      if (event.type == LiveEvent::Type::kImu) {
        dataset.imuSamples.push_back(event.imu);
      }
    }
    if (dataset.imuSamples.empty()) {
      return dataset;
    }

    const double firstImuTimestamp = dataset.imuSamples.front().timestampS;
    for (const LiveEvent& event : events) {
      if (event.type == LiveEvent::Type::kContact &&
          event.contact.timestampS >= firstImuTimestamp - 1e-12) {
        dataset.contactEvents.push_back(event.contact);
      }
    }
    for (size_t index = 0; index < dataset.imuSamples.size(); ++index) {
      dataset.imuSamples[index].index = index;
    }
    for (size_t index = 0; index < dataset.contactEvents.size(); ++index) {
      dataset.contactEvents[index].index = index;
    }
    return dataset;
  }

  bool readyToInitialize() const {
    const Dataset dataset = makeDatasetFromStartupBuffer();
    if (dataset.imuSamples.empty()) {
      return false;
    }
    if (disableFullContactInitialization_) {
      return true;
    }

    const std::optional<ContactEvent> fullContact =
        firstFullContactEvent(dataset);
    if (fullContact) {
      if (latestEstimatorTimestampS_ >=
          fullContact->timestampS + startupBiasWindowSeconds_) {
        return true;
      }
      for (const ContactEvent& event : dataset.contactEvents) {
        if (event.timestampS > fullContact->timestampS + 1e-12 &&
            event.activeContacts.size() < metadata_.footNames.size()) {
          return true;
        }
      }
    }

    return startupTimeoutSeconds_ > 0.0 && haveFirstTimestamp_ &&
           latestEstimatorTimestampS_ - firstTimestampS_ >=
               startupTimeoutSeconds_;
  }

  void initializeFromStartupBuffer(const std::string& reason) {
    const Dataset dataset = makeDatasetFromStartupBuffer();
    if (dataset.imuSamples.empty()) {
      return;
    }

    InitialBiasEstimate imuBiasEstimate;
    if (firstFullContactEvent(dataset)) {
      imuBiasEstimate = estimateInitialImuBias(dataset, replayConfig_);
    } else {
      imuBiasEstimate = {imuBias::ConstantBias(),
                         "none (" + reason + ", no full-contact packet)", 0};
    }

    fs::create_directories(outputDir_);
    RCLCPP_INFO(get_logger(),
                "Initialized live estimator from %s: buffered_imu=%zu, "
                "buffered_contacts=%zu, bias_samples=%zu",
                reason.c_str(), dataset.imuSamples.size(),
                dataset.contactEvents.size(), imuBiasEstimate.sampleCount);

    for (size_t index = 0; index < filterNames_.size(); ++index) {
      const std::string& filterName = filterNames_[index];
      auto runner = std::make_unique<VariantRunner>(
          filterName, metadata_, replayConfig_, maxDurationSeconds_,
          disableFullContactInitialization_,
          [this](const std::string& name, double stamp,
                 const NavState& state) { publishState(name, stamp, state); },
          [this](const std::string& name, double stamp,
                 const ExtendedPose3d& estimate,
                 const std::vector<ContactMeasurement>& contacts) {
            publishFootContacts(name, stamp, estimate, contacts);
          });
      runner->start(outputDir_, imuBiasEstimate.bias);
      const std::string odomTopic = outputTopic("odom");
      const std::string pathTopic = outputTopic("path");
      const std::string contactTopic = outputTopic("foot_contacts");
      RCLCPP_INFO(get_logger(),
                  "%sStarting estimator runner%s: %s -> odom=%s, path=%s, "
                  "foot_contacts=%s",
                  kAnsiBold, kAnsiReset,
                  coloredVariantName(filterName, index).c_str(),
                  odomTopic.c_str(), pathTopic.c_str(), contactTopic.c_str());
      runners_.push_back(std::move(runner));
    }

    initialized_ = true;
    std::vector<LiveEvent> events = startupBuffer_;
    startupBuffer_.clear();
    std::stable_sort(events.begin(), events.end(),
                     [](const LiveEvent& lhs, const LiveEvent& rhs) {
                       if (std::abs(lhs.timestampS - rhs.timestampS) > 1e-12) {
                         return lhs.timestampS < rhs.timestampS;
                       }
                       return (lhs.type == LiveEvent::Type::kImu ? 0 : 1) <
                              (rhs.type == LiveEvent::Type::kImu ? 0 : 1);
                     });
    for (const LiveEvent& event : events) {
      dispatch(event);
    }
  }

  void dispatch(const LiveEvent& event) {
    for (std::unique_ptr<VariantRunner>& runner : runners_) {
      runner->processEvent(event);
    }
  }

  builtin_interfaces::msg::Time stampFromSeconds(double timestampS) const {
    const int64_t stampNs =
        static_cast<int64_t>(std::llround(timestampS * 1e9));
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(stampNs / 1000000000LL);
    stamp.nanosec = static_cast<uint32_t>(stampNs % 1000000000LL);
    return stamp;
  }

  geometry_msgs::msg::PoseStamped makePoseStamped(
      const builtin_interfaces::msg::Time& stamp,
      const NavState& state) const {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = frameId_;
    pose.pose.position.x = state.position().x();
    pose.pose.position.y = state.position().y();
    pose.pose.position.z = state.position().z();
    const auto quaternion = state.quaternion();
    pose.pose.orientation.x = quaternion.x();
    pose.pose.orientation.y = quaternion.y();
    pose.pose.orientation.z = quaternion.z();
    pose.pose.orientation.w = quaternion.w();
    return pose;
  }

  std::array<float, 4> footContactColor(size_t foot) const {
    static const std::array<std::array<float, 4>, 4> kColors{{
        {0.10F, 0.70F, 1.00F, 1.00F},
        {1.00F, 0.45F, 0.18F, 1.00F},
        {0.35F, 0.90F, 0.35F, 1.00F},
        {1.00F, 0.20F, 0.70F, 1.00F},
    }};
    return kColors[foot % kColors.size()];
  }

  visualization_msgs::msg::MarkerArray makeFootContactMarkerArray() const {
    visualization_msgs::msg::MarkerArray markerArray;
    markerArray.markers.reserve(footNames_.size());
    for (size_t foot = 0; foot < footNames_.size(); ++foot) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frameId_;
      marker.ns = footNames_[foot] + "/foot_contacts";
      marker.id = static_cast<int32_t>(foot);
      marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = footContactMarkerScale_;
      marker.scale.y = footContactMarkerScale_;
      marker.scale.z = footContactMarkerScale_;
      const std::array<float, 4> color = footContactColor(foot);
      marker.color.r = color[0];
      marker.color.g = color[1];
      marker.color.b = color[2];
      marker.color.a = color[3];
      markerArray.markers.push_back(std::move(marker));
    }
    return markerArray;
  }

  void publishFootContacts(
      const std::string& filterName, double timestampS,
      const ExtendedPose3d& estimate,
      const std::vector<ContactMeasurement>& contacts) {
    if (!publishFootContacts_) {
      return;
    }

    const auto it =
        std::find(filterNames_.begin(), filterNames_.end(), filterName);
    if (it == filterNames_.end()) {
      return;
    }
    const size_t index =
        static_cast<size_t>(std::distance(filterNames_.begin(), it));
    if (index >= footContactPublishers_.size() ||
        index >= footContactMarkerArrays_.size()) {
      return;
    }

    visualization_msgs::msg::MarkerArray& markerArray =
        footContactMarkerArrays_[index];
    const Matrix footholds = footholdsFromEstimate(estimate);
    const auto stamp = stampFromSeconds(timestampS);
    bool addedPoint = false;

    for (const ContactMeasurement& contact : contacts) {
      if (contact.foot >= footNames_.size() ||
          contact.foot >= static_cast<size_t>(footholds.cols())) {
        continue;
      }
      if (footContactTouchdownsOnly_ && !contact.touchdown) {
        continue;
      }

      visualization_msgs::msg::Marker& marker =
          markerArray.markers[contact.foot];
      const Eigen::Index footColumn =
          static_cast<Eigen::Index>(contact.foot);
      geometry_msgs::msg::Point point;
      point.x = footholds(0, footColumn);
      point.y = footholds(1, footColumn);
      point.z = footholds(2, footColumn) + footContactMarkerZOffset_;
      marker.points.push_back(point);
      if (footContactMaxPoints_ > 0 &&
          marker.points.size() > static_cast<size_t>(footContactMaxPoints_)) {
        const size_t extra =
            marker.points.size() - static_cast<size_t>(footContactMaxPoints_);
        marker.points.erase(marker.points.begin(),
                            marker.points.begin() + extra);
      }
      addedPoint = true;
    }

    if (!addedPoint) {
      return;
    }
    for (visualization_msgs::msg::Marker& marker : markerArray.markers) {
      marker.header.stamp = stamp;
      marker.header.frame_id = frameId_;
    }
    footContactPublishers_[index]->publish(markerArray);
  }

  void publishState(const std::string& filterName, double timestampS,
                    const NavState& state) {
    const auto it =
        std::find(filterNames_.begin(), filterNames_.end(), filterName);
    if (it == filterNames_.end()) {
      return;
    }
    const size_t index =
        static_cast<size_t>(std::distance(filterNames_.begin(), it));
    if (index >= odomPublishers_.size()) {
      return;
    }

    const builtin_interfaces::msg::Time stamp = stampFromSeconds(timestampS);
    const geometry_msgs::msg::PoseStamped pose = makePoseStamped(stamp, state);
    if (publishOdom_) {
      publishOdometry(index, pose, state);
    }
    if (publishPath_) {
      publishPath(index, pose);
    }
  }

  void publishOdometry(size_t index,
                       const geometry_msgs::msg::PoseStamped& pose,
                       const NavState& state) {
    if (index >= odomPublishers_.size()) {
      return;
    }
    nav_msgs::msg::Odometry msg;
    msg.header = pose.header;
    msg.child_frame_id = childFrameId_;
    msg.pose.pose = pose.pose;
    msg.twist.twist.linear.x = state.velocity().x();
    msg.twist.twist.linear.y = state.velocity().y();
    msg.twist.twist.linear.z = state.velocity().z();
    odomPublishers_[index]->publish(msg);
  }

  void publishPath(size_t index,
                   const geometry_msgs::msg::PoseStamped& pose) {
    if (index >= pathPublishers_.size() || index >= pathMessages_.size() ||
        index >= pathStateCounters_.size()) {
      return;
    }

    ++pathStateCounters_[index];
    if ((pathStateCounters_[index] - 1) %
            static_cast<size_t>(pathPublishDecimation_) !=
        0) {
      return;
    }

    nav_msgs::msg::Path& path = pathMessages_[index];
    path.header = pose.header;
    path.poses.push_back(pose);
    if (pathMaxPoses_ > 0 &&
        path.poses.size() > static_cast<size_t>(pathMaxPoses_)) {
      const size_t extra =
          path.poses.size() - static_cast<size_t>(pathMaxPoses_);
      path.poses.erase(path.poses.begin(), path.poses.begin() + extra);
    }
    pathPublishers_[index]->publish(path);
  }

  std::string imuTopic_;
  std::string footTopic_;
  std::string footType_;
  std::string jointStatesTopic_;
  std::string odomTopicPrefix_;
  std::string frameId_;
  std::string childFrameId_;
  bool readJointStates_ = true;
  bool publishOdom_ = true;
  bool publishPath_ = true;
  bool publishFootContacts_ = true;
  bool footContactTouchdownsOnly_ = false;
  double footContactMarkerScale_ = 0.12;
  double footContactMarkerZOffset_ = 0.03;
  int footContactMaxPoints_ = 5000;
  int pathPublishDecimation_ = 10;
  int pathMaxPoses_ = 10000;

  std::vector<std::string> footNames_;
  std::vector<Vector3> legImuOffsets_;
  std::vector<size_t> spotFkPositionIndices_;
  bool useJointFkForBodyPoint_ = true;
  bool jointFkFallbackToMsgBodyPoint_ = true;
  double jointFkMaxJointAgeSeconds_ = 0.05;
  std::string contactStreamMode_ = "transition";
  int contactStateValue_ = 1;
  int contactMadeCode_ = 1;
  int contactLostCode_ = 2;

  std::vector<std::string> filterNames_;
  fs::path outputDir_;
  ReplayConfig replayConfig_;
  DatasetMetadata metadata_;
  double maxDurationSeconds_ = std::numeric_limits<double>::infinity();
  bool disableFullContactInitialization_ = false;
  double startupBiasWindowSeconds_ = 1.0;
  double startupTimeoutSeconds_ = 5.0;

  int qosDepth_ = 1000;
  bool qosReliable_ = false;
  double reorderDelaySeconds_ = 0.02;

  std::shared_ptr<rcpputils::SharedLibrary> cppTypeSupportLibrary_;
  std::shared_ptr<rcpputils::SharedLibrary> introspectionTypeSupportLibrary_;
  const rosidl_message_type_support_t* cppTypeSupport_ = nullptr;
  const rosidl_message_type_support_t* introspectionTypeSupport_ = nullptr;
  const introspection::MessageMembers* footMembers_ = nullptr;
  std::unique_ptr<rclcpp::SerializationBase> serialization_;

  rclcpp::Subscription<Imu>::SharedPtr imuSub_;
  rclcpp::Subscription<JointState>::SharedPtr jointStateSub_;
  rclcpp::GenericSubscription::SharedPtr footSub_;
  std::vector<rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr>
      odomPublishers_;
  std::vector<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr>
      pathPublishers_;
  std::vector<
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr>
      footContactPublishers_;
  std::vector<nav_msgs::msg::Path> pathMessages_;
  std::vector<visualization_msgs::msg::MarkerArray> footContactMarkerArrays_;
  std::vector<size_t> pathStateCounters_;
  rclcpp::TimerBase::SharedPtr flushTimer_;

  std::vector<double> latestJointPositions_;
  bool haveJointState_ = false;
  double lastJointTimestampS_ = 0.0;
  std::vector<bool> inContact_;
  std::vector<bool> previousContactSet_;
  bool havePreviousContactSet_ = false;
  bool reportedFootParseError_ = false;

  std::mutex queueMutex_;
  std::priority_queue<QueuedEvent, std::vector<QueuedEvent>,
                      QueuedEventGreater>
      eventQueue_;
  size_t nextSequence_ = 0;
  size_t nextImuIndex_ = 0;
  size_t nextContactIndex_ = 0;
  double latestQueuedTimestampS_ = -std::numeric_limits<double>::infinity();
  double lastDispatchedTimestampS_ = -std::numeric_limits<double>::infinity();
  double latestEstimatorTimestampS_ = -std::numeric_limits<double>::infinity();
  double firstTimestampS_ = 0.0;
  bool haveFirstTimestamp_ = false;

  std::vector<LiveEvent> startupBuffer_;
  std::vector<std::unique_ptr<VariantRunner>> runners_;
  bool initialized_ = false;
  bool finished_ = false;
};

int runMain(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LeggedEstimatorRos2Node>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  node->finish();
  rclcpp::shutdown();
  return 0;
}

}  // namespace
}  // namespace gtsam

int main(int argc, char* argv[]) {
  return gtsam::runMain(argc, argv);
}
