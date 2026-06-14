#pragma once

#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <gtsam/base/Matrix.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "include/LeggedEstimatorReplayUtils.h"

namespace gtsam {

class SpotContactAdapter {
 public:
  struct Options {
    std::vector<std::string> footNames;
    std::vector<Vector3> legImuOffsets;
    std::string footType = "spot_msgs/msg/FootStateArray";
    std::string contactStreamMode = "transition";
    int contactStateValue = 1;
    int contactMadeCode = 1;
    int contactLostCode = 2;
    bool useJointFkForBodyPoint = true;
    bool jointFkFallbackToMsgBodyPoint = true;
    double jointFkMaxJointAgeSeconds = 0.05;
    std::vector<size_t> fkPositionIndices{
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
  };

  explicit SpotContactAdapter(Options options);
  ~SpotContactAdapter();

  SpotContactAdapter(const SpotContactAdapter&) = delete;
  SpotContactAdapter& operator=(const SpotContactAdapter&) = delete;

  const std::string& footType() const;
  void updateJointState(const sensor_msgs::msg::JointState& msg);
  std::optional<ContactEvent> makeContactEvent(
      rclcpp::SerializedMessage& serialized, size_t eventIndex);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gtsam
