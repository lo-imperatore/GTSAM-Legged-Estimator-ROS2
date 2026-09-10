#include "include/UnitreeG1Adapter.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace gtsam {
UnitreeG1Adapter::UnitreeG1Adapter(Options options)
    : options_(std::move(options)) {
  if (options_.footNames.size() != 2) {
    throw std::runtime_error("Unitree G1 adapter requires exactly two feet");
  }
  if (options_.legImuOffsets.empty()) {
    options_.legImuOffsets.assign(2, Vector3::Zero());
  }
  if (options_.legImuOffsets.size() != 2) {
    throw std::runtime_error(
        "Unitree G1 footNames and legImuOffsets sizes differ");
  }
  if (options_.contactForceOn <= options_.contactForceOff ||
      options_.contactForceOff < 0.0 ||
      options_.contactForceFilterAlpha <= 0.0 ||
      options_.contactForceFilterAlpha > 1.0 ||
      options_.wrenchDamping <= 0.0 ||
      options_.maxValidContactForce <= options_.contactForceOn ||
      options_.contactHeightTolerance < 0.0 ||
      options_.contactReleaseHeight < options_.contactHeightTolerance) {
    throw std::runtime_error("Invalid Unitree G1 contact detector thresholds");
  }
  for (const size_t index : options_.legMotorIndices) {
    if (index >= 35) {
      throw std::runtime_error("Unitree G1 motor index must be below 35");
    }
  }
}

UnitreeG1Adapter::Result UnitreeG1Adapter::convert(
    const unitree_hg::msg::LowState& msg, const double timestampS,
    const size_t imuIndex, const size_t contactIndex) {
  Result result;
  result.imu.index = imuIndex;
  result.imu.timestampS = timestampS;
  result.imu.omega = Vector3(msg.imu_state.gyroscope[0],
                             msg.imu_state.gyroscope[1],
                             msg.imu_state.gyroscope[2]);
  result.imu.specificForce = Vector3(msg.imu_state.accelerometer[0],
                                     msg.imu_state.accelerometer[1],
                                     msg.imu_state.accelerometer[2]);

  std::array<LegKinematics, 2> kinematics{
      legKinematics(0, msg), legKinematics(1, msg)};
  std::array<Vector3, 2> positions{
      kinematics[0].footPosition, kinematics[1].footPosition};
  const double lowestZ = std::min(positions[0].z(), positions[1].z());
  ContactEvent event;
  event.index = contactIndex;
  event.timestampS = timestampS;
  for (size_t foot = 0; foot < 2; ++foot) {
    const double verticalForce =
        estimateVerticalContactForce(foot, kinematics[foot], msg);
    result.rawVerticalForces[foot] = verticalForce;
    if (!havePreviousSample_) {
      filteredVerticalForces_[foot] = verticalForce;
    } else {
      filteredVerticalForces_[foot] =
          options_.contactForceFilterAlpha * verticalForce +
          (1.0 - options_.contactForceFilterAlpha) *
              filteredVerticalForces_[foot];
    }

    const double forceThreshold = previousContactActive_[foot]
                                      ? options_.contactForceOff
                                      : options_.contactForceOn;
    result.filteredVerticalForces[foot] = filteredVerticalForces_[foot];
    result.activeThresholds[foot] = forceThreshold;
    const double heightLimit = previousContactActive_[foot]
                                   ? options_.contactReleaseHeight
                                   : options_.contactHeightTolerance;
    const bool heightOk = !options_.useHeightGuard ||
                          positions[foot].z() <= lowestZ + heightLimit;
    const bool forceValid =
        filteredVerticalForces_[foot] <= options_.maxValidContactForce;
    const bool active = heightOk && forceValid &&
                        filteredVerticalForces_[foot] >= forceThreshold;
    result.contactsActive[foot] = active;
    if (active) {
      ContactMeasurement measurement;
      measurement.foot = foot;
      measurement.bodyPoint = positions[foot] + options_.legImuOffsets[foot];
      measurement.touchdown = !previousContactActive_[foot];
      event.activeContacts.push_back(measurement);
      event.worldRelativeContactPoints.push_back(positions[foot]);
    }
    previousContactActive_[foot] = active;
  }

  havePreviousSample_ = true;
  // Keep publishing empty packets too: they tell the estimator that both
  // feet have left contact and allow leaving-foot marginalization.
  result.contact = std::move(event);
  return result;
}

UnitreeG1Adapter::LegKinematics UnitreeG1Adapter::legKinematics(
    const size_t foot, const unitree_hg::msg::LowState& msg) const {
  // G1 23/29-DoF lower-body ordering is identical. These transforms follow
  // the Unitree G1 model from pelvis through the centre of the sole.
  const bool left = foot == 0;
  const size_t offset = foot * 6;
  std::array<double, 6> q{};
  for (size_t joint = 0; joint < q.size(); ++joint) {
    q[joint] = msg.motor_state[options_.legMotorIndices[offset + joint]].q;
  }

  LegKinematics result;
  std::array<Vector3, 6> jointOrigins;
  std::array<Vector3, 6> jointAxes;
  size_t joint = 0;
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  transform.translate(
      Vector3(0.0, left ? 0.064452 : -0.064452, -0.1027));
  jointOrigins[joint] = transform.translation();
  jointAxes[joint++] = transform.linear() * Vector3::UnitY();
  transform.rotate(Eigen::AngleAxisd(q[0], Vector3::UnitY()));
  transform.translate(Vector3(0.0, left ? 0.052 : -0.052, -0.030465));
  jointOrigins[joint] = transform.translation();
  jointAxes[joint++] = transform.linear() * Vector3::UnitX();
  transform.rotate(Eigen::AngleAxisd(q[1], Vector3::UnitX()));
  transform.translate(Vector3(0.025, 0.0, -0.12412));
  jointOrigins[joint] = transform.translation();
  jointAxes[joint++] = transform.linear() * Vector3::UnitZ();
  transform.rotate(Eigen::AngleAxisd(q[2], Vector3::UnitZ()));
  transform.translate(Vector3(-0.078273, 0.0, -0.17734));
  jointOrigins[joint] = transform.translation();
  jointAxes[joint++] = transform.linear() * Vector3::UnitY();
  transform.rotate(Eigen::AngleAxisd(q[3], Vector3::UnitY()));
  transform.translate(Vector3(0.0, 0.0, -0.30001));
  jointOrigins[joint] = transform.translation();
  jointAxes[joint++] = transform.linear() * Vector3::UnitY();
  transform.rotate(Eigen::AngleAxisd(q[4], Vector3::UnitY()));
  transform.translate(Vector3(0.0, 0.0, -0.017558));
  jointOrigins[joint] = transform.translation();
  jointAxes[joint++] = transform.linear() * Vector3::UnitX();
  transform.rotate(Eigen::AngleAxisd(q[5], Vector3::UnitX()));
  transform.translate(Vector3(0.035, 0.0, -0.03));
  result.footPosition = transform.translation();

  for (size_t column = 0; column < jointAxes.size(); ++column) {
    result.jacobian.block<3, 1>(0, column) =
        jointAxes[column].cross(result.footPosition - jointOrigins[column]);
    result.jacobian.block<3, 1>(3, column) = jointAxes[column];
  }
  return result;
}

double UnitreeG1Adapter::estimateVerticalContactForce(
    const size_t foot, const LegKinematics& kinematics,
    const unitree_hg::msg::LowState& msg) const {
  Vector6 jointTorques = Vector6::Zero();
  const size_t offset = foot * 6;
  for (size_t joint = 0; joint < 6; ++joint) {
    jointTorques(static_cast<Eigen::Index>(joint)) =
        msg.motor_state[options_.legMotorIndices[offset + joint]].tau_est;
  }

  // tau = J^T wrench. Damping bounds the reconstruction near straight-leg
  // singularities. The absolute vertical component makes the classifier
  // independent of the actuator/external-wrench sign convention.
  const Matrix6 normal = kinematics.jacobian * kinematics.jacobian.transpose() +
                         options_.wrenchDamping * options_.wrenchDamping *
                             Matrix6::Identity();
  const Vector6 wrench = normal.ldlt().solve(
      kinematics.jacobian * jointTorques);
  const double verticalForce = std::abs(wrench(2));
  return std::isfinite(verticalForce)
             ? verticalForce
             : options_.maxValidContactForce + 1.0;
}

}  // namespace gtsam
