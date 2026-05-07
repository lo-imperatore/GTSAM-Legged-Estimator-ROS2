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

#include "include/LeggedEstimatorReplayUtils.h"

namespace gtsam {

struct ReplayConfig : public LeggedEstimatorParams {
  Vector3 gravity = Vector3(0.0, 0.0, -9.81);
  // Contact body points are already expressed in IMU frame.
  // Keep body->IMU extrinsic as identity to avoid double-transform.
  Pose3 body_P_imu = Pose3();
  Point3 initialPosition = Point3(0.0, 0.0, 0.0);
  Vector3 initialVelocity = Vector3::Zero();
  Vector initialBaseCovarianceDiagonal =
      (Vector(9) << 1e-2, 1e-2, 1e-6, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05)
          .finished();
  double sigmaGyro = 8e-4;
  double sigmaIntegration = 1e-3;
  double sigmaAcc = 2e-2;
  double lagSeconds = 1.0;
  double maxDeadReckoningSeconds = 0.100;
};

struct InitialBiasEstimate {
  imuBias::ConstantBias bias;
  std::string source;
  size_t sampleCount = 0;
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
    const std::vector<ContactMeasurement>& activeContacts) {
  Matrix imuContacts(3, static_cast<Eigen::Index>(activeContacts.size()));
  for (size_t index = 0; index < activeContacts.size(); ++index) {
    imuContacts.col(static_cast<Eigen::Index>(index)) =
        replayConfig.body_P_imu.transformTo(
            Point3(activeContacts[index].bodyPoint));
  }

  const Vector3 centroid = imuContacts.rowwise().mean();
  const Matrix centered = imuContacts.colwise() - centroid;
  Vector3 normal =
      Eigen::JacobiSVD<Matrix>(centered, Eigen::ComputeFullU).matrixU().col(2);
  if (normal.z() < 0.0) {
    normal = -normal;
  }

  const double roll = std::atan2(normal.y(), normal.z());
  const double pitch = std::asin(-normal.x());
  const Rot3 attitude = Rot3::Ypr(0.0, pitch, roll);

  double height = 0.0;
  for (const ContactMeasurement& contact : activeContacts) {
    const Point3 measurement =
        replayConfig.body_P_imu.transformTo(Point3(contact.bodyPoint));
    height -= attitude.matrix().row(2).dot(measurement);
  }
  height /= static_cast<double>(activeContacts.size());
  return NavState(attitude, Point3(0.0, 0.0, height), Vector3::Zero());
}

inline double initialBiasWindowEndTime(const Dataset& dataset,
                                       double startTime) {
  double endTime = startTime + 1.0;
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

inline InitialBiasEstimate estimateInitialImuBias(
    const Dataset& dataset, const ReplayConfig& replayConfig) {
  const std::optional<ContactEvent> event = firstFullContactEvent(dataset);
  if (!event) {
    return {imuBias::ConstantBias(), "none", 0};
  }
  const double endTime = initialBiasWindowEndTime(dataset, event->timestampS);
  const std::vector<ImuSample> postInitSamples =
      initialStationaryImuSamples(dataset, event->timestampS, endTime);
  const NavState initialNavState =
      fullContactInitializationNavState(replayConfig, event->activeContacts);
  if (samplesLookStationary(postInitSamples, replayConfig.gravity.norm())) {
    return {
        estimateBiasFromSamples(postInitSamples, initialNavState, replayConfig),
        "post-init full-contact", postInitSamples.size()};
  }

  const std::vector<ImuSample> preInitSamples =
      initialStaticImuSamplesBefore(dataset, event->timestampS);
  return {
      estimateBiasFromSamples(preInitSamples, initialNavState, replayConfig),
      "pre-init static fallback", preInitSamples.size()};
}

inline InitialBiasEstimate estimateInitialImuBias(const Dataset& dataset) {
  return estimateInitialImuBias(dataset, ReplayConfig());
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
  return NavState(Rot3(), replayConfig.initialPosition,
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
