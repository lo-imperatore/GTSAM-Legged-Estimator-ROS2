#include "include/JointTorqueGrfEstimator.h"

#ifdef GTSAM_LEGGED_HAVE_ANYMAL_MSGS
#include <any_msgs/msg/extended_joint_state.hpp>
#endif

#include <Eigen/Geometry>
#ifdef GTSAM_LEGGED_HAVE_PINOCCHIO
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#endif

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <unordered_map>

namespace gtsam {
namespace {

using Matrix3 = Eigen::Matrix3d;

std::string canonical(const std::string& value) {
  std::string result;
  result.reserve(value.size());
  for (const char character : value) {
    const auto c = static_cast<unsigned char>(character);
    if (std::isalnum(c)) {
      result.push_back(static_cast<char>(std::toupper(c)));
    }
  }
  return result;
}

std::string legPrefix(const std::string& footName) {
  const std::string name = canonical(footName);
  if (name.find("LF") != std::string::npos ||
      name.find("FL") != std::string::npos) {
    return "LF";
  }
  if (name.find("RF") != std::string::npos ||
      name.find("FR") != std::string::npos) {
    return "RF";
  }
  if (name.find("LH") != std::string::npos ||
      name.find("HL") != std::string::npos ||
      name.find("RL") != std::string::npos) {
    return "LH";
  }
  if (name.find("RH") != std::string::npos ||
      name.find("HR") != std::string::npos ||
      name.find("RR") != std::string::npos) {
    return "RH";
  }
  return {};
}

struct JointSample {
  double position = 0.0;
  double velocity = 0.0;
  double effort = 0.0;
  bool valid = false;
};

template <typename JointMessage>
JointSample sampleFor(const JointMessage& joints, const std::string& name) {
  const std::string expected = canonical(name);
  for (size_t index = 0; index < joints.name.size(); ++index) {
    if (canonical(joints.name[index]) != expected ||
        index >= joints.position.size() || index >= joints.velocity.size() ||
        index >= joints.effort.size()) {
      continue;
    }
    const double position = joints.position[index];
    const double velocity = joints.velocity[index];
    const double effort = joints.effort[index];
    return {position, velocity, effort,
            std::isfinite(position) && std::isfinite(velocity) &&
                std::isfinite(effort)};
  }
  return {};
}

struct Geometry {
  Vector3 haaOrigin;
  Vector3 haaToHfe;
  Vector3 hfeToKfe;
  Vector3 kfeToFoot;
  double hfeAxisSign;
  double kfeAxisSign;
};

Geometry geometryFor(const std::string& robotModel, const std::string& leg) {
  const double front = (leg == "LF" || leg == "RF") ? 1.0 : -1.0;
  const double left = (leg == "LF" || leg == "LH") ? 1.0 : -1.0;
  if (robotModel == "spot") {
    return {
        Vector3(front > 0.0 ? 0.0 : -0.5957, left * 0.055, 0.0),
        Vector3(0.0, left * 0.110945, 0.0),
        Vector3(0.025, 0.0, -0.3205),
        Vector3(0.0, 0.0, -0.34),
        1.0,
        1.0,
    };
  }
  // Values are the zero-configuration transforms from the ANYmal-D URDF.
  return {
      Vector3(front * 0.304, left * 0.109, 0.0),
      Vector3(front * 0.069, left * 0.006, 0.0),
      Vector3(0.0, left * 0.1805, -0.285),
      Vector3(front * 0.100, left * 0.02225, -0.39246),
      1.0,
      1.0,
  };
}

std::array<std::string, 3> spotJointNames(const std::string& leg) {
  if (leg == "LF") {
    return {"front_left_hip_x", "front_left_hip_y", "front_left_knee"};
  }
  if (leg == "RF") {
    return {"front_right_hip_x", "front_right_hip_y", "front_right_knee"};
  }
  if (leg == "LH") {
    return {"rear_left_hip_x", "rear_left_hip_y", "rear_left_knee"};
  }
  return {"rear_right_hip_x", "rear_right_hip_y", "rear_right_knee"};
}

}  // namespace

#ifdef GTSAM_LEGGED_HAVE_PINOCCHIO
struct JointTorqueGrfEstimator::Dynamics {
  explicit Dynamics(const std::string& urdfPath) {
    if (urdfPath.empty()) {
      throw std::runtime_error(
          "URDF path is required for joint-torque bias compensation");
    }
    pinocchio::urdf::buildModel(
        urdfPath, pinocchio::JointModelFreeFlyer(), model);
    data = std::make_unique<pinocchio::Data>(model);
  }

  pinocchio::Model model;
  std::unique_ptr<pinocchio::Data> data;
};
#else
struct JointTorqueGrfEstimator::Dynamics {};
#endif

JointTorqueGrfEstimator::JointTorqueGrfEstimator(
    const double damping, const double torqueScale,
    const std::string& urdfPath, const std::string& robotModel)
    : damping_(damping),
      torqueScale_(torqueScale),
      robotModel_(canonical(robotModel)) {
  if (!std::isfinite(damping_) || damping_ < 0.0 ||
      !std::isfinite(torqueScale_)) {
    throw std::runtime_error("Invalid joint-torque GRF estimator parameters");
  }
  std::transform(robotModel_.begin(), robotModel_.end(), robotModel_.begin(),
                 [](const unsigned char c) {
                   return static_cast<char>(std::tolower(c));
                 });
  if (robotModel_ != "anymal" && robotModel_ != "spot") {
    throw std::runtime_error(
        "Joint-torque GRF robot model must be anymal or spot");
  }
  if (!urdfPath.empty()) {
#ifdef GTSAM_LEGGED_HAVE_PINOCCHIO
    dynamics_ = std::make_unique<Dynamics>(urdfPath);
#else
    throw std::runtime_error(
        "URDF dynamics support is not available in this build");
#endif
  } else if (robotModel_ == "anymal" || robotModel_ == "spot") {
    throw std::runtime_error(
        "A URDF path is required for joint-torque GRF support");
  }
}

JointTorqueGrfEstimator::~JointTorqueGrfEstimator() = default;

#ifdef GTSAM_LEGGED_HAVE_ANYMAL_MSGS
std::vector<JointTorqueGrfEstimator::Result> JointTorqueGrfEstimator::estimate(
    const anymal_msgs::msg::AnymalState& msg,
    const std::vector<std::string>& footNames) const {
  Eigen::VectorXd modelQ = pinocchio::neutral(dynamics_->model);
  Eigen::VectorXd modelV = Eigen::VectorXd::Zero(dynamics_->model.nv);
  Eigen::VectorXd modelA = Eigen::VectorXd::Zero(dynamics_->model.nv);

  Eigen::Quaterniond worldFromBody(
      msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
      msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
  if (!worldFromBody.coeffs().allFinite() ||
      worldFromBody.norm() <= 1.0e-12) {
    return std::vector<Result>(footNames.size());
  }
  worldFromBody.normalize();
  modelQ.segment<3>(0) =
      Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y,
              msg.pose.pose.position.z);
  // Pinocchio's free-flyer quaternion order is [x, y, z, w].
  modelQ.segment<4>(3) = worldFromBody.coeffs();

  for (pinocchio::JointIndex joint = 2;
       joint < dynamics_->model.joints.size(); ++joint) {
    const auto& jointModel = dynamics_->model.joints[joint];
    if (jointModel.nq() != 1 || jointModel.nv() != 1) {
      continue;
    }
    const JointSample sample = sampleFor(msg.joints, dynamics_->model.names[joint]);
    if (!sample.valid) {
      return std::vector<Result>(footNames.size());
    }
    modelQ[jointModel.idx_q()] = sample.position;
    modelV[jointModel.idx_v()] = sample.velocity;
  }

  // With zero base velocity and zero generalized acceleration, RNEA returns
  // Pronto's default h_q(q, qdot, R): gravity plus velocity bias torques.
  const Eigen::VectorXd bias =
      pinocchio::rnea(dynamics_->model, *dynamics_->data,
                      modelQ, modelV, modelA);

  std::vector<Result> results(footNames.size());
  for (size_t foot = 0; foot < footNames.size(); ++foot) {
    const std::string leg = legPrefix(footNames[foot]);
    if (leg.empty()) {
      continue;
    }
    const JointSample haa = sampleFor(msg.joints, leg + "_HAA");
    const JointSample hfe = sampleFor(msg.joints, leg + "_HFE");
    const JointSample kfe = sampleFor(msg.joints, leg + "_KFE");
    if (!haa.valid || !hfe.valid || !kfe.valid) {
      continue;
    }

    const Geometry geometry = geometryFor("anymal", leg);
    const Matrix3 bodyRhaa =
        Eigen::AngleAxisd(haa.position, Vector3::UnitX()).toRotationMatrix();
    const Vector3 hfeAxis =
        bodyRhaa * (geometry.hfeAxisSign * Vector3::UnitY());
    const Matrix3 bodyRhfe =
        bodyRhaa *
        Eigen::AngleAxisd(
            geometry.hfeAxisSign * hfe.position, Vector3::UnitY())
            .toRotationMatrix();
    const Vector3 kfeAxis =
        bodyRhfe * (geometry.kfeAxisSign * Vector3::UnitY());
    const Matrix3 bodyRkfe =
        bodyRhfe *
        Eigen::AngleAxisd(
            geometry.kfeAxisSign * kfe.position, Vector3::UnitY())
            .toRotationMatrix();

    const Vector3 haaOrigin = geometry.haaOrigin;
    const Vector3 hfeOrigin =
        haaOrigin + bodyRhaa * geometry.haaToHfe;
    const Vector3 kfeOrigin =
        hfeOrigin + bodyRhfe * geometry.hfeToKfe;
    const Vector3 footPosition =
        kfeOrigin + bodyRkfe * geometry.kfeToFoot;

    Matrix3 jacobian;
    jacobian.col(0) =
        Vector3::UnitX().cross(footPosition - haaOrigin);
    jacobian.col(1) = hfeAxis.cross(footPosition - hfeOrigin);
    jacobian.col(2) = kfeAxis.cross(footPosition - kfeOrigin);
    const Vector3 jointFootVelocity =
        jacobian * Vector3(haa.velocity, hfe.velocity, kfe.velocity);

    const Vector3 jointTorques =
        torqueScale_ * Vector3(haa.effort, hfe.effort, kfe.effort);
    Vector3 biasTorques;
    const std::array<std::string, 3> jointNames = {
        leg + "_HAA", leg + "_HFE", leg + "_KFE"};
    bool biasValid = true;
    for (size_t joint = 0; joint < jointNames.size(); ++joint) {
      const pinocchio::JointIndex jointId =
          dynamics_->model.getJointId(jointNames[joint]);
      if (jointId == 0 || dynamics_->model.joints[jointId].nv() != 1) {
        biasValid = false;
        break;
      }
      biasTorques[joint] =
          bias[dynamics_->model.joints[jointId].idx_v()];
    }
    if (!biasValid || !biasTorques.allFinite()) {
      continue;
    }
    const Matrix3 normal =
        jacobian * jacobian.transpose() +
        damping_ * damping_ * Matrix3::Identity();
    // Pronto's default inputs set all accelerations and base velocity to zero:
    // f = J^dagger (-tau + h_q(q, qdot, R)).
    const Vector3 force =
        normal.ldlt().solve(jacobian * (biasTorques - jointTorques));
    if (!footPosition.allFinite() || !jointFootVelocity.allFinite() ||
        !force.allFinite()) {
      continue;
    }
    results[foot] = {footPosition, jointFootVelocity, force, true};
  }
  return results;
}
#endif

std::vector<JointTorqueGrfEstimator::Result> JointTorqueGrfEstimator::estimate(
    const sensor_msgs::msg::JointState& msg,
    const std::vector<std::string>& footNames) const {
#ifdef GTSAM_LEGGED_HAVE_PINOCCHIO
  Eigen::VectorXd bias;
  Eigen::VectorXd modelQ;
  Eigen::VectorXd modelV;
  if (dynamics_) {
    modelQ = pinocchio::neutral(dynamics_->model);
    modelV = Eigen::VectorXd::Zero(dynamics_->model.nv);
    const Eigen::VectorXd modelA = Eigen::VectorXd::Zero(dynamics_->model.nv);
    const std::array<std::string, 4> legs = {"LF", "RF", "LH", "RH"};
    for (const std::string& leg : legs) {
      const std::array<std::string, 3> inputNames = spotJointNames(leg);
      const std::array<std::string, 3> modelNames = {
          leg + "_HAA", leg + "_HFE", leg + "_KFE"};
      for (size_t joint = 0; joint < modelNames.size(); ++joint) {
        const JointSample sample = sampleFor(msg, inputNames[joint]);
        const pinocchio::JointIndex jointId =
            dynamics_->model.getJointId(modelNames[joint]);
        if (!sample.valid || jointId == 0 ||
            dynamics_->model.joints[jointId].nq() != 1 ||
            dynamics_->model.joints[jointId].nv() != 1) {
          return std::vector<Result>(footNames.size());
        }
        modelQ[dynamics_->model.joints[jointId].idx_q()] = sample.position;
        modelV[dynamics_->model.joints[jointId].idx_v()] = sample.velocity;
      }
    }
    bias = pinocchio::rnea(dynamics_->model, *dynamics_->data,
                           modelQ, modelV, modelA);
    pinocchio::forwardKinematics(dynamics_->model, *dynamics_->data,
                                 modelQ, modelV);
    pinocchio::updateFramePlacements(dynamics_->model, *dynamics_->data);
  }
#endif
  std::vector<Result> results(footNames.size());
  for (size_t foot = 0; foot < footNames.size(); ++foot) {
    const std::string leg = legPrefix(footNames[foot]);
    if (leg.empty()) {
      continue;
    }
    const std::array<std::string, 3> names =
        robotModel_ == "spot"
            ? spotJointNames(leg)
            : std::array<std::string, 3>{leg + "_HAA", leg + "_HFE",
                                         leg + "_KFE"};
    const JointSample haa = sampleFor(msg, names[0]);
    const JointSample hfe = sampleFor(msg, names[1]);
    const JointSample kfe = sampleFor(msg, names[2]);
    if (!haa.valid || !hfe.valid || !kfe.valid) {
      continue;
    }

    const Vector3 jointVelocity(haa.velocity, hfe.velocity, kfe.velocity);
    Vector3 footPosition;
    Matrix3 jacobian;
#ifdef GTSAM_LEGGED_HAVE_PINOCCHIO
    if (dynamics_) {
      const std::string footFrameName = leg + "_FOOT";
      const pinocchio::FrameIndex frameId =
          dynamics_->model.getFrameId(footFrameName);
      if (frameId >= dynamics_->model.frames.size()) {
        continue;
      }
      Eigen::Matrix<double, 6, Eigen::Dynamic> frameJacobian(
          6, dynamics_->model.nv);
      frameJacobian.setZero();
      pinocchio::computeFrameJacobian(
          dynamics_->model, *dynamics_->data, modelQ, frameId,
          pinocchio::LOCAL_WORLD_ALIGNED, frameJacobian);
      const std::array<std::string, 3> modelNames = {
          leg + "_HAA", leg + "_HFE", leg + "_KFE"};
      bool modelValid = true;
      for (size_t joint = 0; joint < modelNames.size(); ++joint) {
        const pinocchio::JointIndex jointId =
            dynamics_->model.getJointId(modelNames[joint]);
        if (jointId == 0 || dynamics_->model.joints[jointId].nv() != 1) {
          modelValid = false;
          break;
        }
        jacobian.col(joint) = frameJacobian.block<3, 1>(
            0, dynamics_->model.joints[jointId].idx_v());
      }
      if (!modelValid) {
        continue;
      }
      footPosition = dynamics_->data->oMf[frameId].translation();
    } else
#endif
    {
      const Geometry geometry = geometryFor(robotModel_, leg);
      const Matrix3 bodyRhaa =
          Eigen::AngleAxisd(haa.position, Vector3::UnitX()).toRotationMatrix();
      const Vector3 hfeAxis = bodyRhaa * Vector3::UnitY();
      const Matrix3 bodyRhfe =
          bodyRhaa *
          Eigen::AngleAxisd(hfe.position, Vector3::UnitY()).toRotationMatrix();
      const Vector3 kfeAxis = bodyRhfe * Vector3::UnitY();
      const Matrix3 bodyRkfe =
          bodyRhfe *
          Eigen::AngleAxisd(kfe.position, Vector3::UnitY()).toRotationMatrix();
      const Vector3 haaOrigin = geometry.haaOrigin;
      const Vector3 hfeOrigin = haaOrigin + bodyRhaa * geometry.haaToHfe;
      const Vector3 kfeOrigin = hfeOrigin + bodyRhfe * geometry.hfeToKfe;
      footPosition = kfeOrigin + bodyRkfe * geometry.kfeToFoot;
      jacobian.col(0) =
          Vector3::UnitX().cross(footPosition - haaOrigin);
      jacobian.col(1) = hfeAxis.cross(footPosition - hfeOrigin);
      jacobian.col(2) = kfeAxis.cross(footPosition - kfeOrigin);
    }
    const Vector3 jointFootVelocity = jacobian * jointVelocity;
    const Vector3 jointTorques =
        torqueScale_ * Vector3(haa.effort, hfe.effort, kfe.effort);
    Vector3 biasTorques = Vector3::Zero();
#ifdef GTSAM_LEGGED_HAVE_PINOCCHIO
    if (dynamics_) {
      const std::array<std::string, 3> modelNames = {
          leg + "_HAA", leg + "_HFE", leg + "_KFE"};
      for (size_t joint = 0; joint < modelNames.size(); ++joint) {
        const pinocchio::JointIndex jointId =
            dynamics_->model.getJointId(modelNames[joint]);
        biasTorques[joint] =
            bias[dynamics_->model.joints[jointId].idx_v()];
      }
    }
#endif
    const Matrix3 normal =
        jacobian * jacobian.transpose() +
        damping_ * damping_ * Matrix3::Identity();
    const Vector3 force =
        normal.ldlt().solve(jacobian * (biasTorques - jointTorques));
    if (footPosition.allFinite() && jointFootVelocity.allFinite() &&
        force.allFinite()) {
      results[foot] = {footPosition, jointFootVelocity, force, true};
    }
  }
  return results;
}

}  // namespace gtsam
