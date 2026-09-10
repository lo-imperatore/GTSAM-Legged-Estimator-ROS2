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

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/LeggedEstimator.h>
#include <gtsam/navigation/NavState.h>

#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>
#include <cstdio>

#include "include/LeggedEstimatorReplayUtils.h"

namespace gtsam {

struct ReplayConfig : public LeggedEstimatorParams {
  Vector3 gravity = Vector3(0.0, 0.0, -9.80665);
  // The navigation state and contact measurements are base-centred. This
  // transform therefore remains identity inside the estimator; the ROS node
  // owns the physical IMU extrinsic used to rotate sensor measurements.
  Pose3 body_P_imu = Pose3();
  Point3 initialPosition = Point3(0.0, 0.0, 0.0);
  Rot3 initialAttitude = Rot3();
  Vector3 initialVelocity = Vector3::Zero();
  Vector initialBaseCovarianceDiagonal =
      (Vector(9) << 1e-2, 1e-2, 1e-6, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05)
          .finished();
  double sigmaGyro = 8e-3;
  double sigmaIntegration = 1e-3;
  double sigmaAcc = 2e-2;
  bool negateAccelStartupRollPitch = false;
  bool useImuOrientationForStartupBias = false;
  double lagSeconds = 1.0;
  double maxDeadReckoningSeconds = 0.100;
};

struct InitialBiasEstimate {
  imuBias::ConstantBias bias;
  std::string source;
  size_t sampleCount = 0;
  std::vector<ImuSample> samples;
  std::optional<NavState> initialNavState;
};

inline std::optional<ContactEvent> firstFullContactEvent(
    const Dataset& dataset) {
  for (const ContactEvent& event : dataset.contactEvents) {
    if (event.activeContacts.size() == dataset.metadata.footNames.size()) {
      return event;
    }
  }
  return std::nullopt;
}

inline NavState fullContactInitializationNavState(
    const ReplayConfig& replayConfig,
    const std::vector<std::string>& footNames,
    const std::vector<ContactMeasurement>& activeContacts,
    const std::vector<Vector3>& planeContactPoints = {},
    const std::string& planeContactSource = "body_imu_contacts",
    const bool rotatePlaneAttitudeToImu = false) {
  const bool useProvidedContactPlane =
      planeContactPoints.size() == activeContacts.size();
  Matrix planeContacts(3, static_cast<Eigen::Index>(activeContacts.size()));
  for (size_t index = 0; index < activeContacts.size(); ++index) {
    planeContacts.col(static_cast<Eigen::Index>(index)) =
        useProvidedContactPlane
            ? planeContactPoints[index]
            : replayConfig.body_P_imu.transformTo(
                  Point3(activeContacts[index].bodyPoint));
  }

  const Vector3 centroid = planeContacts.rowwise().mean();
  const Matrix centered = planeContacts.colwise() - centroid;
  Vector3 normal =
      Eigen::JacobiSVD<Matrix>(centered, Eigen::ComputeFullU).matrixU().col(2);
  if (normal.z() < 0.0) {
    normal = -normal;
  }

  const double planeRoll = std::atan2(normal.y(), normal.z());
  const double planePitch = std::asin(-normal.x());
  const Rot3 planeAttitude = Rot3::Ypr(0.0, planePitch, planeRoll);
  const Rot3 attitude = rotatePlaneAttitudeToImu
                            ? Rot3(planeAttitude.matrix() *
                                   replayConfig.body_P_imu.rotation().matrix())
                            : planeAttitude;
  const Matrix3 attitudeMatrix = attitude.matrix();
  const double roll = std::atan2(attitudeMatrix(2, 1), attitudeMatrix(2, 2));
  const double pitch = std::asin(-attitudeMatrix(2, 0));

  double height = 0.0;
  for (Eigen::Index index = 0; index < planeContacts.cols(); ++index) {
    height -= planeAttitude.matrix().row(2).dot(planeContacts.col(index));
  }
  height /= static_cast<double>(activeContacts.size());

  std::fprintf(stderr,
    "[INIT ATTITUDE from contacts]  source=%s  plane_roll=%.4f  "
    "plane_pitch=%.4f  bias_roll=%.4f  bias_pitch=%.4f  height=%.4f  "
    "floor_normal=[%.4f, %.4f, %.4f]\n",
    useProvidedContactPlane ? planeContactSource.c_str() : "body_imu_contacts",
    planeRoll, planePitch, roll, pitch, height, normal.x(), normal.y(),
    normal.z());
  for (size_t index = 0; index < activeContacts.size(); ++index) {
    const ContactMeasurement& contact = activeContacts[index];
    const Vector3 planePoint = planeContacts.col(static_cast<Eigen::Index>(index));
    const Point3 measurement =
        replayConfig.body_P_imu.transformTo(Point3(contact.bodyPoint));
    const char* footName = contact.foot < footNames.size()
                               ? footNames[contact.foot].c_str()
                               : "unknown";
    std::fprintf(stderr,
                 "[INIT PLANE CONTACT POINT] order=%zu  foot=%-4s idx=%zu  "
                 "plane_point=[%.6f, %.6f, %.6f]  "
                 "imu_pos=[%.6f, %.6f, %.6f]  "
                 "body_point=[%.6f, %.6f, %.6f]\n",
                 index, footName, contact.foot, planePoint.x(),
                 planePoint.y(), planePoint.z(), measurement.x(),
                 measurement.y(), measurement.z(), contact.bodyPoint.x(),
                 contact.bodyPoint.y(), contact.bodyPoint.z());
  }


  return NavState(attitude, Point3(0.0, 0.0, height), Vector3::Zero());
}

inline std::optional<NavState> anymalPoseInitializationNavState(
    const ContactEvent& event) {
  if (!event.hasWorldFromBody) {
    return std::nullopt;
  }

  const double squaredNorm =
      event.worldFromBodyW * event.worldFromBodyW +
      event.worldFromBodyX * event.worldFromBodyX +
      event.worldFromBodyY * event.worldFromBodyY +
      event.worldFromBodyZ * event.worldFromBodyZ;
  if (squaredNorm <= 1.0e-24) {
    return std::nullopt;
  }

  const double invNorm = 1.0 / std::sqrt(squaredNorm);
  const Rot3 worldFromBody = Rot3::Quaternion(
      event.worldFromBodyW * invNorm, event.worldFromBodyX * invNorm,
      event.worldFromBodyY * invNorm, event.worldFromBodyZ * invNorm);
  const Matrix3 rotation = worldFromBody.matrix();
  const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
  const double pitch = std::asin(-rotation(2, 0));
  const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  std::fprintf(stderr,
               "[INIT ATTITUDE from ANYmal pose]  roll=%.4f  pitch=%.4f  "
               "yaw=%.4f\n",
               roll, pitch, yaw);
  return NavState(worldFromBody, Point3(0.0, 0.0, 0.0), Vector3::Zero());
}

inline std::optional<NavState> accelerometerWindowInitializationNavState(
    const std::vector<ImuSample>& samples, const ReplayConfig& replayConfig,
    const std::string& /*source*/) {
  if (samples.empty()) {
    return std::nullopt;
  }

  Vector3 meanSpecificForce = Vector3::Zero();
  for (const ImuSample& sample : samples) {
    meanSpecificForce += sample.specificForce;
  }
  meanSpecificForce /= static_cast<double>(samples.size());

  const double meanAccelNorm = meanSpecificForce.norm();
  const double gravityMagnitude = replayConfig.gravity.norm();
  if (meanAccelNorm <= 1.0e-9 || gravityMagnitude <= 1.0e-9) {
    return std::nullopt;
  }

  const Vector3 accelDirection = meanSpecificForce / meanAccelNorm;
  double pitch =
      std::asin(std::clamp(-accelDirection.x(), -1.0, 1.0));
  double roll = std::atan2(accelDirection.y(), accelDirection.z());
  if (replayConfig.negateAccelStartupRollPitch) {
    roll = -roll;
    pitch = -pitch;
  }
  const double yaw = 0.0;
  const Rot3 worldFromBody = Rot3::Ypr(yaw, pitch, roll);
  return NavState(worldFromBody, Point3(0.0, 0.0, 0.0), Vector3::Zero());
}

inline Vector3 rollPitchYawFromAttitude(const Rot3& attitude) {
  const Matrix3 rotation = attitude.matrix();
  return Vector3(std::atan2(rotation(2, 1), rotation(2, 2)),
                 std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0)),
                 std::atan2(rotation(1, 0), rotation(0, 0)));
}

inline Rot3 attitudeWithYawFrom(const Rot3& rollPitchAttitude,
                                const Rot3& yawAttitude) {
  const Vector3 rollPitchRpy =
      rollPitchYawFromAttitude(rollPitchAttitude);
  const double yaw = rollPitchYawFromAttitude(yawAttitude).z();
  return Rot3::Ypr(yaw, rollPitchRpy.y(), rollPitchRpy.x());
}

inline double initialBiasWindowEndTime(const Dataset& dataset,
                                       double startTime,
                                       double windowSeconds) {
  double endTime = startTime + windowSeconds;
  bool seenStart = false;
  for (const ContactEvent& event : dataset.contactEvents) {
    if (!seenStart) {
      if (std::abs(event.timestampS - startTime) < 1e-12) {
        seenStart = true;
      }
      continue;
    }
    if (event.activeContacts.size() < dataset.metadata.footNames.size()) {
      endTime = std::min(endTime, event.timestampS);
      break;
    }
  }
  return endTime;
}

inline bool samplesLookStationary(const std::vector<ImuSample>& samples,
                                  double gravityMagnitude) {
  if (samples.empty()) {
    return false;
  }
  double meanOmegaNorm = 0.0;
  double meanAccelNorm = 0.0;
  for (const ImuSample& sample : samples) {
    meanOmegaNorm += sample.omega.norm();
    meanAccelNorm += sample.specificForce.norm();
  }
  meanOmegaNorm /= static_cast<double>(samples.size());
  meanAccelNorm /= static_cast<double>(samples.size());
  return meanOmegaNorm < 1e-2 &&
         std::abs(meanAccelNorm - gravityMagnitude) < 0.15;
}

inline std::vector<ImuSample> initialStaticImuSamplesBefore(
    const Dataset& dataset, double endTime) {
  std::vector<ImuSample> samples;
  const double startTime = dataset.imuSamples.front().timestampS;
  const double gravityMagnitude = ReplayConfig().gravity.norm();
  constexpr double kMaxWindowSeconds = 4.0;
  constexpr double kMaxOmegaNorm = 1e-2;
  constexpr double kMaxAccelNormError = 0.2;

  for (const ImuSample& sample : dataset.imuSamples) {
    if (sample.timestampS > endTime ||
        sample.timestampS - startTime > kMaxWindowSeconds) {
      break;
    }
    const bool stationary = sample.omega.norm() <= kMaxOmegaNorm &&
                            std::abs(sample.specificForce.norm() -
                                     gravityMagnitude) <= kMaxAccelNormError;
    if (!stationary && !samples.empty()) {
      break;
    }
    if (stationary) {
      samples.push_back(sample);
    }
  }

  if (samples.empty()) {
    const size_t fallbackCount =
        std::min<size_t>(dataset.imuSamples.size(), 200);
    samples.assign(dataset.imuSamples.begin(),
                   dataset.imuSamples.begin() + fallbackCount);
  }
  return samples;
}

inline std::vector<ImuSample> initialStationaryImuSamples(
    const Dataset& dataset, double startTime, double endTime) {
  std::vector<ImuSample> samples;

  for (const ImuSample& sample : dataset.imuSamples) {
    if (sample.timestampS < startTime) {
      continue;
    }
    if (sample.timestampS > endTime) {
      break;
    }
    samples.push_back(sample);
  }

  if (samples.empty()) {
    const size_t fallbackCount =
        std::min<size_t>(dataset.imuSamples.size(), 200);
    samples.assign(dataset.imuSamples.begin(),
                   dataset.imuSamples.begin() + fallbackCount);
  }
  return samples;
}

inline imuBias::ConstantBias estimateBiasFromSamples(
    const std::vector<ImuSample>& samples, const NavState& initialNavState,
    const ReplayConfig& replayConfig) {
  Vector3 meanOmega = Vector3::Zero();
  Vector3 meanSpecificForce = Vector3::Zero();
  for (const ImuSample& sample : samples) {
    meanOmega += sample.omega;
    meanSpecificForce += sample.specificForce;
  }
  meanOmega /= static_cast<double>(samples.size());
  meanSpecificForce /= static_cast<double>(samples.size());

  const Vector3 expectedSpecificForce =
      initialNavState.attitude().unrotate(-replayConfig.gravity);
  const Vector3 accelBias = meanSpecificForce - expectedSpecificForce;
  return imuBias::ConstantBias(accelBias, meanOmega);
}

inline std::optional<imuBias::ConstantBias>
estimateBiasFromSamplesUsingImuAttitude(
    const std::vector<ImuSample>& samples,
    const ReplayConfig& replayConfig) {
  Vector3 meanOmega = Vector3::Zero();
  Vector3 meanSpecificForce = Vector3::Zero();
  Vector3 meanExpectedSpecificForce = Vector3::Zero();
  size_t validCount = 0;
  for (const ImuSample& sample : samples) {
    if (!sample.hasAttitude) {
      continue;
    }
    meanOmega += sample.omega;
    meanSpecificForce += sample.specificForce;
    meanExpectedSpecificForce +=
        sample.attitude.unrotate(-replayConfig.gravity);
    ++validCount;
  }
  if (validCount == 0) {
    return std::nullopt;
  }
  const double denominator = static_cast<double>(validCount);
  meanOmega /= denominator;
  meanSpecificForce /= denominator;
  meanExpectedSpecificForce /= denominator;
  const Vector3 accelBias =
      meanSpecificForce - meanExpectedSpecificForce;
  std::fprintf(stderr,
               "[IMU ORIENTATION BIAS] samples=%zu\n"
               "  mean_accel     = [%9.6f, %9.6f, %9.6f] m/s²\n"
               "  expected_accel = [%9.6f, %9.6f, %9.6f] m/s²\n"
               "  accel_bias     = [%9.6f, %9.6f, %9.6f] m/s²\n",
               validCount, meanSpecificForce.x(), meanSpecificForce.y(),
               meanSpecificForce.z(), meanExpectedSpecificForce.x(),
               meanExpectedSpecificForce.y(), meanExpectedSpecificForce.z(),
               accelBias.x(), accelBias.y(), accelBias.z());
  return imuBias::ConstantBias(accelBias, meanOmega);
}

inline InitialBiasEstimate estimateInitialImuBias(
    const Dataset& dataset, const ReplayConfig& replayConfig,
    double biasWindowSeconds) {
  const std::optional<ContactEvent> event = firstFullContactEvent(dataset);
  if (!event) {
    return {imuBias::ConstantBias(), "none", 0, {}, std::nullopt};
  }
  const double endTime = initialBiasWindowEndTime(
      dataset, event->timestampS, biasWindowSeconds);
  const std::vector<ImuSample> postInitSamples =
      initialStationaryImuSamples(dataset, event->timestampS, endTime);
  if (samplesLookStationary(postInitSamples, replayConfig.gravity.norm())) {
    InitialBiasEstimate result;
    const std::optional<NavState> accelNavState =
        accelerometerWindowInitializationNavState(
            postInitSamples, replayConfig, "post-init accelerometer window");
    if (accelNavState) {
      const auto imuAttitudeBias =
          replayConfig.useImuOrientationForStartupBias
              ? estimateBiasFromSamplesUsingImuAttitude(postInitSamples,
                                                        replayConfig)
              : std::nullopt;
      const size_t validAttitudeSamples = static_cast<size_t>(std::count_if(
          postInitSamples.begin(), postInitSamples.end(),
          [](const ImuSample& sample) { return sample.hasAttitude; }));
      result = {imuAttitudeBias.value_or(estimateBiasFromSamples(
                    postInitSamples, *accelNavState, replayConfig)),
                imuAttitudeBias ? "post-init IMU-orientation window"
                                : "post-init accelerometer window",
                imuAttitudeBias ? validAttitudeSamples : postInitSamples.size(),
                postInitSamples, accelNavState};
    } else {
      result = {imuBias::ConstantBias(),
                "none (invalid post-init accelerometer window)", 0, {},
                std::nullopt};
    }
    return result;
  }

  const std::vector<ImuSample> preInitSamples =
      initialStaticImuSamplesBefore(dataset, event->timestampS);

  InitialBiasEstimate result;
  const std::optional<NavState> accelNavState =
      accelerometerWindowInitializationNavState(
          preInitSamples, replayConfig,
          "pre-init accelerometer window fallback");
  if (accelNavState) {
    const auto imuAttitudeBias =
        replayConfig.useImuOrientationForStartupBias
            ? estimateBiasFromSamplesUsingImuAttitude(preInitSamples,
                                                      replayConfig)
            : std::nullopt;
    const size_t validAttitudeSamples = static_cast<size_t>(std::count_if(
        preInitSamples.begin(), preInitSamples.end(),
        [](const ImuSample& sample) { return sample.hasAttitude; }));
    result = {imuAttitudeBias.value_or(estimateBiasFromSamples(
                  preInitSamples, *accelNavState, replayConfig)),
              imuAttitudeBias
                  ? "pre-init IMU-orientation window fallback"
                  : "pre-init accelerometer window fallback",
              imuAttitudeBias ? validAttitudeSamples : preInitSamples.size(),
              preInitSamples, accelNavState};
  } else {
    result = {imuBias::ConstantBias(),
              "none (invalid pre-init accelerometer window fallback)", 0, {},
              std::nullopt};
  }
  return result;
}

inline LeggedEstimatorParams makeParams(
    const ReplayConfig& replayConfig, const bool disableFullContactInitialization,
    const imuBias::ConstantBias& imuBiasEstimate) {
  auto preintegrationParams =
      std::make_shared<PreintegrationParams>(replayConfig.gravity);
  preintegrationParams->gyroscopeCovariance =
      Matrix3::Identity() * (replayConfig.sigmaGyro * replayConfig.sigmaGyro);
  preintegrationParams->integrationCovariance =
      Matrix3::Identity() *
      (replayConfig.sigmaIntegration * replayConfig.sigmaIntegration);
  preintegrationParams->accelerometerCovariance =
      Matrix3::Identity() * (replayConfig.sigmaAcc * replayConfig.sigmaAcc);

  LeggedEstimatorParams params = replayConfig;
  params.preintegrationParams = preintegrationParams;
  params.imuBias = imuBiasEstimate;
  params.useFullContactInitialization = !disableFullContactInitialization;
  return params;
}

inline NavState makeInitialState(const ReplayConfig& replayConfig) {
  const Pose3 worldPBase(replayConfig.initialAttitude,
                         replayConfig.initialPosition);
  return NavState(worldPBase.rotation(), worldPBase.translation(),
                  replayConfig.initialVelocity);
}

inline NavState makeInitialState(const ReplayConfig& replayConfig,
                                 const Rot3& initialAttitude) {
  const Pose3 worldPBase(initialAttitude, replayConfig.initialPosition);
  return NavState(worldPBase.rotation(), worldPBase.translation(),
                  replayConfig.initialVelocity);
}

inline Matrix makeInitialCovariance(size_t numFeet,
                                    const ReplayConfig& replayConfig) {
  const int dim = 9 + 3 * static_cast<int>(numFeet);
  Matrix covariance = Matrix::Zero(dim, dim);
  covariance.diagonal().head(9) = replayConfig.initialBaseCovarianceDiagonal;
  const double footholdVariance =
      replayConfig.footholdInitSigma * replayConfig.footholdInitSigma;
  for (size_t foot = 0; foot < numFeet; ++foot) {
    covariance.diagonal()
        .segment(9 + 3 * static_cast<int>(foot), 3)
        .setConstant(footholdVariance);
  }
  return covariance;
}

inline Matrix9 makeInitialBaseCovariance(const ReplayConfig& replayConfig) {
  Matrix9 covariance = Matrix9::Zero();
  covariance.diagonal() = replayConfig.initialBaseCovarianceDiagonal;
  return covariance;
}

inline Matrix makeInitialFootholds(size_t numFeet) {
  return Matrix::Zero(3, static_cast<Eigen::Index>(numFeet));
}

inline NavState navStateFromEstimate(const ExtendedPose3d& estimate) {
  return NavState(estimate.rotation(), estimate.x(0), estimate.x(1));
}

inline Matrix footholdsFromEstimate(const ExtendedPose3d& estimate) {
  const Eigen::Index numFeet =
      static_cast<Eigen::Index>(estimate.k() - static_cast<size_t>(2));
  return estimate.xMatrix().rightCols(numFeet);
}

inline double contactResidualNorm(const ExtendedPose3d& estimate,
                                  const Pose3& bodyPImu,
                                  const ContactMeasurement& contact) {
  const Matrix footholds = footholdsFromEstimate(estimate);
  const NavState navState = navStateFromEstimate(estimate);
  const Point3 foothold(footholds.col(static_cast<Eigen::Index>(contact.foot)));
  const Point3 prediction = navState.pose().transformTo(foothold);
  const Point3 measurement = bodyPImu.transformTo(Point3(contact.bodyPoint));
  return (prediction - measurement).norm();
}

inline bool hasTouchdown(const std::vector<ContactMeasurement>& contacts) {
  return std::any_of(
      contacts.begin(), contacts.end(),
      [](const ContactMeasurement& contact) { return contact.touchdown; });
}

inline std::unique_ptr<LeggedEstimator> makeEstimator(
    const std::string& filterName, const std::vector<std::string>& footNames,
    const NavState& initialState, const Matrix& footholds,
    const Matrix& covariance, const Matrix9& baseCovariance,
    const LeggedEstimatorParams& params, double lagSeconds) {
  if (filterName == "invariant_ekf") {
    return std::make_unique<LeggedInvariantEKF>(initialState, footholds,
                                                covariance, params, footNames);
  }
  if (filterName == "invariant_graph") {
    return std::make_unique<LeggedInvariantIEKF>(initialState, footholds,
                                                 covariance, params, footNames);
  }
  if (filterName == "fixed_lag_single_bias") {
    return std::make_unique<LeggedFixedLagSmoother>(
        initialState, footholds, baseCovariance, params, lagSeconds, footNames);
  }
  if (filterName == "fixed_lag_combined_bias") {
    return std::make_unique<LeggedCombinedFixedLagSmoother>(
        initialState, footholds, baseCovariance, params, lagSeconds, footNames);
  }
  throw std::runtime_error("Unknown filter name: " + filterName);
}

}  // namespace gtsam
