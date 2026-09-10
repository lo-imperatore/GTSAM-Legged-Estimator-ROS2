#pragma once

#include <unitree_hg/msg/low_state.hpp>

#include <gtsam/base/Matrix.h>

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "include/LeggedEstimatorReplayUtils.h"

namespace gtsam {

class UnitreeG1Adapter {
 public:
  struct Options {
    std::vector<std::string> footNames{"left", "right"};
    std::vector<Vector3> legImuOffsets;
    std::array<size_t, 12> legMotorIndices{
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    double contactForceOn = 40.0;
    double contactForceOff = 20.0;
    double contactForceFilterAlpha = 0.2;
    double wrenchDamping = 0.01;
    double maxValidContactForce = 2000.0;
    bool useHeightGuard = false;
    double contactHeightTolerance = 0.025;
    double contactReleaseHeight = 0.045;
  };

  struct Result {
    ImuSample imu;
    std::optional<ContactEvent> contact;
    std::array<double, 2> rawVerticalForces{0.0, 0.0};
    std::array<double, 2> filteredVerticalForces{0.0, 0.0};
    std::array<double, 2> activeThresholds{0.0, 0.0};
    std::array<bool, 2> contactsActive{false, false};
  };

  explicit UnitreeG1Adapter(Options options);

  Result convert(const unitree_hg::msg::LowState& msg, double timestampS,
                 size_t imuIndex, size_t contactIndex);

 private:
  struct LegKinematics {
    Vector3 footPosition = Vector3::Zero();
    Matrix6 jacobian = Matrix6::Zero();
  };

  LegKinematics legKinematics(
      size_t foot, const unitree_hg::msg::LowState& msg) const;
  double estimateVerticalContactForce(
      size_t foot, const LegKinematics& kinematics,
      const unitree_hg::msg::LowState& msg) const;

  Options options_;
  std::array<double, 2> filteredVerticalForces_{0.0, 0.0};
  std::array<bool, 2> previousContactActive_{false, false};
  bool havePreviousSample_ = false;
};

}  // namespace gtsam
