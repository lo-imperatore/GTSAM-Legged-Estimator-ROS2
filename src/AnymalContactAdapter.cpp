#include "include/AnymalContactAdapter.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace gtsam {
namespace {

bool containsToken(const std::string& value, const std::string& token) {
  return value.find(token) != std::string::npos;
}

}  // namespace

AnymalContactAdapter::AnymalContactAdapter(Options options)
    : options_(std::move(options)) {
  if (options_.footNames.empty()) {
    throw std::runtime_error("ANYmal contact adapter needs at least one foot");
  }
  if (options_.legImuOffsets.empty()) {
    options_.legImuOffsets.assign(options_.footNames.size(), Vector3::Zero());
  }
  if (options_.legImuOffsets.size() != options_.footNames.size()) {
    throw std::runtime_error(
        "ANYmal contact adapter footNames and legImuOffsets sizes differ");
  }

  canonicalFootNames_.reserve(options_.footNames.size());
  for (const std::string& footName : options_.footNames) {
    canonicalFootNames_.push_back(canonicalFootName(footName));
  }
  previousContactActive_.assign(options_.footNames.size(), false);
  havePreviousContactStates_.assign(options_.footNames.size(), false);
}

std::optional<ContactEvent> AnymalContactAdapter::makeContactEvent(
    const anymal_msgs::msg::AnymalState& msg, const size_t eventIndex) {
  if (options_.requireStateOk &&
      msg.state != anymal_msgs::msg::AnymalState::STATE_OK) {
    return std::nullopt;
  }

  std::vector<ContactMeasurement> activeContacts;

  for (const anymal_msgs::msg::Contact& contact : msg.contacts) {
    const std::optional<size_t> foot = footIndexForContactName(contact.name);
    if (!foot) {
      continue;
    }

    const bool hadPrevious = havePreviousContactStates_.at(*foot);
    const bool previousActive = previousContactActive_.at(*foot);
    const bool active = isActiveContact(contact);
    const bool touchdown = active && (!hadPrevious || !previousActive);

    previousContactActive_.at(*foot) = active;
    havePreviousContactStates_.at(*foot) = true;

    if (!active) {
      continue;
    }

    ContactMeasurement measurement;
    measurement.foot = *foot;
    measurement.bodyPoint = bodyPointFromWorldContact(contact, msg.pose) +
                            options_.legImuOffsets.at(*foot);
    measurement.touchdown = touchdown;
    activeContacts.push_back(measurement);
  }

  if (activeContacts.empty()) {
    return std::nullopt;
  }

  ContactEvent event;
  event.index = eventIndex;
  event.timestampS = stampToSeconds(msg.header.stamp);
  event.activeContacts = std::move(activeContacts);
  return event;
}

double AnymalContactAdapter::stampToSeconds(
    const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) +
         static_cast<double>(stamp.nanosec) * 1e-9;
}

std::string AnymalContactAdapter::canonicalFootName(const std::string& name) {
  std::string canonical;
  canonical.reserve(name.size());
  for (const char value : name) {
    const auto c = static_cast<unsigned char>(value);
    if (std::isalnum(c)) {
      canonical.push_back(static_cast<char>(std::toupper(c)));
    }
  }
  return canonical;
}

std::vector<std::string> AnymalContactAdapter::aliasesForContactName(
    const std::string& name) {
  const std::string canonical = canonicalFootName(name);
  if (containsToken(canonical, "FL") || containsToken(canonical, "LF")) {
    return {"FL", "LF"};
  }
  if (containsToken(canonical, "FR") || containsToken(canonical, "RF")) {
    return {"FR", "RF"};
  }
  if (containsToken(canonical, "HL") || containsToken(canonical, "LH") ||
      containsToken(canonical, "RL")) {
    return {"HL", "LH", "RL"};
  }
  if (containsToken(canonical, "HR") || containsToken(canonical, "RH") ||
      containsToken(canonical, "RR")) {
    return {"HR", "RH", "RR"};
  }
  return {canonical};
}

std::optional<size_t> AnymalContactAdapter::footIndexForContactName(
    const std::string& name) const {
  const std::vector<std::string> aliases = aliasesForContactName(name);
  for (const std::string& alias : aliases) {
    const auto found = std::find(
        canonicalFootNames_.begin(), canonicalFootNames_.end(), alias);
    if (found != canonicalFootNames_.end()) {
      return static_cast<size_t>(
          std::distance(canonicalFootNames_.begin(), found));
    }
  }
  return std::nullopt;
}

bool AnymalContactAdapter::isActiveContact(
    const anymal_msgs::msg::Contact& contact) const {
  if (options_.slippingIsContact) {
    return contact.state != anymal_msgs::msg::Contact::STATE_OPEN;
  }
  return contact.state == anymal_msgs::msg::Contact::STATE_CLOSED;
}

Vector3 AnymalContactAdapter::bodyPointFromWorldContact(
    const anymal_msgs::msg::Contact& contact,
    const geometry_msgs::msg::PoseStamped& bodyPose) const {
  const Vector3 footWorld(
      contact.position.x, contact.position.y, contact.position.z);
  const Vector3 bodyWorld(bodyPose.pose.position.x, bodyPose.pose.position.y,
                          bodyPose.pose.position.z);

  Eigen::Quaterniond worldFromBody(
      bodyPose.pose.orientation.w, bodyPose.pose.orientation.x,
      bodyPose.pose.orientation.y, bodyPose.pose.orientation.z);

  if (worldFromBody.norm() <= 1.0e-12) {
    return footWorld - bodyWorld;
  }

  worldFromBody.normalize();
  return worldFromBody.inverse() * (footWorld - bodyWorld);
}

}  // namespace gtsam
