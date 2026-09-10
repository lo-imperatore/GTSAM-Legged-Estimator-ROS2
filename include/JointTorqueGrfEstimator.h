#pragma once

#include <sensor_msgs/msg/joint_state.hpp>
#ifdef GTSAM_LEGGED_HAVE_ANYMAL_MSGS
#include <anymal_msgs/msg/anymal_state.hpp>
#endif

#include <gtsam/base/Matrix.h>

#include <string>
#include <memory>
#include <vector>

namespace gtsam {

/// Robot-agnostic foot GRF reconstruction from measured joint torques.
class JointTorqueGrfEstimator {
 public:
  struct Result {
    Vector3 bodyFootPosition = Vector3::Zero();
    Vector3 bodyJointFootVelocity = Vector3::Zero();
    Vector3 bodyGroundReactionForce = Vector3::Zero();
    bool valid = false;
  };

  explicit JointTorqueGrfEstimator(double damping = 0.02,
                                   double torqueScale = 1.0,
                                   const std::string& urdfPath = {},
                                   const std::string& robotModel = "anymal");
  ~JointTorqueGrfEstimator();

  JointTorqueGrfEstimator(const JointTorqueGrfEstimator&) = delete;
  JointTorqueGrfEstimator& operator=(const JointTorqueGrfEstimator&) = delete;

#ifdef GTSAM_LEGGED_HAVE_ANYMAL_MSGS
  std::vector<Result> estimate(
      const anymal_msgs::msg::AnymalState& msg,
      const std::vector<std::string>& footNames) const;
#endif
  std::vector<Result> estimate(
      const sensor_msgs::msg::JointState& msg,
      const std::vector<std::string>& footNames) const;

 private:
  struct Dynamics;
  double damping_;
  double torqueScale_;
  std::string robotModel_;
  std::unique_ptr<Dynamics> dynamics_;
};

}  // namespace gtsam
