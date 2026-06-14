#pragma once

#include <anymal_msgs/msg/anymal_state.hpp>
#include <anymal_msgs/msg/contact.hpp>

#include <gtsam/base/Matrix.h>

#include <optional>
#include <string>
#include <vector>

#include "include/LeggedEstimatorReplayUtils.h"

namespace gtsam {

class AnymalContactAdapter {
 public:
  struct Options {
    std::vector<std::string> footNames;
    std::vector<Vector3> legImuOffsets;
    bool slippingIsContact = true;
    bool requireStateOk = false;
  };

  explicit AnymalContactAdapter(Options options);

  std::optional<ContactEvent> makeContactEvent(
      const anymal_msgs::msg::AnymalState& msg, size_t eventIndex);

 private:
  static double stampToSeconds(const builtin_interfaces::msg::Time& stamp);
  static std::string canonicalFootName(const std::string& name);
  static std::vector<std::string> aliasesForContactName(
      const std::string& name);
  std::optional<size_t> footIndexForContactName(
      const std::string& name) const;
  bool isActiveContact(const anymal_msgs::msg::Contact& contact) const;
  Vector3 bodyPointFromWorldContact(
      const anymal_msgs::msg::Contact& contact,
      const geometry_msgs::msg::PoseStamped& bodyPose) const;

  Options options_;
  std::vector<std::string> canonicalFootNames_;
  std::vector<bool> previousContactActive_;
  std::vector<bool> havePreviousContactStates_;
};

}  // namespace gtsam
