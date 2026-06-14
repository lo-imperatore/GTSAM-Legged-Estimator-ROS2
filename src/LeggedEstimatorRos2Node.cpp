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
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "include/AnymalContactAdapter.h"
#include "include/LeggedEstimatorCore.h"
#include "include/SpotContactAdapter.h"

namespace gtsam {
namespace {

namespace fs = std::filesystem;

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

std::string normalizeRobotType(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](const unsigned char c) {
                   return static_cast<char>(std::tolower(c));
                 });
  if (value != "spot" && value != "anymal") {
    throw std::runtime_error("robot.type must be either 'spot' or 'anymal'");
  }
  return value;
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
    setupRosInterfaces();

    logStartupConfiguration();
    RCLCPP_INFO(get_logger(), "C++ live estimator subscribed: robot=%s, imu=%s",
                robotType_.c_str(), imuTopic_.c_str());
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
    const bool legacyUseAnymalContacts =
        declare_parameter<bool>("topics.use_anymal_contacts", false);
    robotType_ = normalizeRobotType(declare_parameter<std::string>(
        "robot.type", legacyUseAnymalContacts ? "anymal" : "spot"));
    SpotContactAdapter::Options spotOptions;
    AnymalContactAdapter::Options anymalOptions;
    if (robotType_ == "spot") {
      spotFootTopic_ =
          declare_parameter<std::string>("topics.foot", "/spot/status/feet");
      spotFootType_ = declare_parameter<std::string>(
          "topics.foot_type", "spot_msgs/msg/FootStateArray");
      jointStatesTopic_ = declare_parameter<std::string>(
          "topics.joint_states", "/joint_states");
      readJointStates_ =
          declare_parameter<bool>("topics.read_joint_states", true);
    } else {
      anymalStateTopic_ =
          declare_parameter<std::string>("topics.anymal_state", "/state");
    }
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

    if (robotType_ == "spot") {
      spotOptions.footNames = footNames_;
      spotOptions.legImuOffsets = legImuOffsets_;
      spotOptions.footType = spotFootType_;
      const std::vector<int64_t> fkIndices =
          declare_parameter<std::vector<int64_t>>(
              "calibration.spot_fk_position_indices",
              {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11});
      if (fkIndices.size() != 12) {
        throw std::runtime_error(
            "calibration.spot_fk_position_indices must contain 12 values");
      }
      spotOptions.fkPositionIndices.clear();
      for (const int64_t index : fkIndices) {
        if (index < 0) {
          throw std::runtime_error(
              "calibration.spot_fk_position_indices cannot contain negatives");
        }
        spotOptions.fkPositionIndices.push_back(static_cast<size_t>(index));
      }

      spotOptions.useJointFkForBodyPoint = declare_parameter<bool>(
          "calibration.use_joint_fk_for_body_point", true);
      spotOptions.jointFkFallbackToMsgBodyPoint = declare_parameter<bool>(
          "calibration.joint_fk_fallback_to_msg_body_point", true);
      spotOptions.jointFkMaxJointAgeSeconds = declare_parameter<double>(
          "calibration.joint_fk_max_joint_age_seconds", 0.05);
    } else {
      anymalOptions.footNames = footNames_;
      anymalOptions.legImuOffsets = legImuOffsets_;
    }

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

    if (robotType_ == "spot") {
      spotOptions.contactStreamMode =
          declare_parameter<std::string>("conversion.contact_stream_mode",
                                         "transition");
      spotOptions.contactStateValue =
          declare_parameter<int>("conversion.contact_state_value", 1);
      spotOptions.contactMadeCode =
          declare_parameter<int>("conversion.contact_made_code", 1);
      spotOptions.contactLostCode =
          declare_parameter<int>("conversion.contact_lost_code", 2);
    } else {
      anymalOptions.slippingIsContact = declare_parameter<bool>(
          "conversion.anymal_slipping_is_contact", true);
      anymalOptions.requireStateOk = declare_parameter<bool>(
          "conversion.anymal_require_state_ok", false);
    }

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
    if (robotType_ == "spot") {
      spotContactAdapter_ =
          std::make_unique<SpotContactAdapter>(std::move(spotOptions));
    } else {
      anymalContactAdapter_ =
          std::make_unique<AnymalContactAdapter>(std::move(anymalOptions));
    }
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

  void setupRosInterfaces() {
    const rclcpp::QoS qos = makeQos();
    imuSub_ = create_subscription<Imu>(
        imuTopic_, qos,
        [this](const Imu::SharedPtr msg) { handleImu(*msg); });
    if (spotContactAdapter_ && readJointStates_) {
      jointStateSub_ = create_subscription<JointState>(
          jointStatesTopic_, qos,
          [this](const JointState::SharedPtr msg) { handleJointState(*msg); });
    }
    if (spotContactAdapter_) {
      footSub_ = create_generic_subscription(
          spotFootTopic_, spotContactAdapter_->footType(), qos,
          [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
            handleFootSerialized(msg);
          });
    }
    if (anymalContactAdapter_) {
      anymalStateSub_ =
          create_subscription<anymal_msgs::msg::AnymalState>(
              anymalStateTopic_, qos,
              [this](const anymal_msgs::msg::AnymalState::SharedPtr msg) {
                handleAnymalState(*msg);
              });
    }

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
    RCLCPP_INFO(get_logger(), "%s%sRobot type%s: %s", kAnsiBold, kAnsiBlue,
                kAnsiReset, robotType_.c_str());
    if (spotContactAdapter_) {
      RCLCPP_INFO(get_logger(),
                  "%s%sSpot contact input%s: foot=%s (%s), joint_states=%s",
                  kAnsiBold, kAnsiBlue, kAnsiReset, spotFootTopic_.c_str(),
                  spotContactAdapter_->footType().c_str(),
                  readJointStates_ ? jointStatesTopic_.c_str() : "disabled");
    }
    if (anymalContactAdapter_) {
      RCLCPP_INFO(get_logger(),
                  "%s%sANYmal contact input%s: anymal_state=%s",
                  kAnsiBold, kAnsiBlue, kAnsiReset,
                  anymalStateTopic_.c_str());
    }
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
    if (spotContactAdapter_) {
      spotContactAdapter_->updateJointState(msg);
    }
  }

  void handleFootSerialized(
      const std::shared_ptr<rclcpp::SerializedMessage>& serialized) {
    if (!spotContactAdapter_) {
      return;
    }
    try {
      std::optional<ContactEvent> contactEvent =
          spotContactAdapter_->makeContactEvent(*serialized, nextContactIndex_);
      if (!contactEvent) {
        return;
      }
      ++nextContactIndex_;
      LiveEvent event;
      event.type = LiveEvent::Type::kContact;
      event.timestampS = contactEvent->timestampS;
      event.contact = std::move(*contactEvent);
      enqueueEvent(std::move(event));
    } catch (const std::exception& error) {
      if (!reportedFootParseError_) {
        reportedFootParseError_ = true;
        RCLCPP_ERROR(get_logger(), "Failed to parse %s: %s",
                     spotContactAdapter_->footType().c_str(), error.what());
      }
    }
  }

  void handleAnymalState(const anymal_msgs::msg::AnymalState& msg) {
    if (!anymalContactAdapter_) {
      return;
    }
    try {
      std::optional<ContactEvent> contactEvent =
          anymalContactAdapter_->makeContactEvent(msg, nextContactIndex_);
      if (!contactEvent) {
        return;
      }
      ++nextContactIndex_;

      LiveEvent event;
      event.type = LiveEvent::Type::kContact;
      event.timestampS = contactEvent->timestampS;
      event.contact = std::move(*contactEvent);
      enqueueEvent(std::move(event));
    } catch (const std::exception& error) {
      if (!reportedAnymalParseError_) {
        reportedAnymalParseError_ = true;
        RCLCPP_ERROR(get_logger(), "Failed to parse ANYmal state contacts: %s",
                     error.what());
      }
    }
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
  std::string robotType_ = "spot";
  std::string spotFootTopic_;
  std::string spotFootType_;
  std::string anymalStateTopic_;
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

  rclcpp::Subscription<Imu>::SharedPtr imuSub_;
  rclcpp::Subscription<JointState>::SharedPtr jointStateSub_;
  rclcpp::Subscription<anymal_msgs::msg::AnymalState>::SharedPtr anymalStateSub_;
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

  bool reportedFootParseError_ = false;
  bool reportedAnymalParseError_ = false;
  std::unique_ptr<SpotContactAdapter> spotContactAdapter_;
  std::unique_ptr<AnymalContactAdapter> anymalContactAdapter_;

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
