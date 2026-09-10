#include "include/AnymalContactAdapter.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
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
  std::transform(options_.contactClassifier.begin(),
                 options_.contactClassifier.end(),
                 options_.contactClassifier.begin(), [](unsigned char c) {
                   return static_cast<char>(std::tolower(c));
                 });
  if (options_.contactClassifier != "state" &&
      options_.contactClassifier != "joint_torque_grf") {
    throw std::runtime_error(
        "ANYmal contact classifier must be 'state' or 'joint_torque_grf'");
  }
  if (options_.contactClassifier == "joint_torque_grf") {
    jointTorqueGrfEstimator_ = std::make_unique<JointTorqueGrfEstimator>(
        options_.jointTorqueGrfDamping, options_.jointTorqueScale,
        options_.anymalUrdfPath);
    if (!std::isfinite(options_.contactForceOn) ||
        options_.contactForceOn < 0.0 ||
        !std::isfinite(options_.contactForceOff) ||
        options_.contactForceOff < 0.0 ||
        options_.contactForceOff > options_.contactForceOn ||
        !std::isfinite(options_.contactMaxForceDelta) ||
        options_.contactMaxForceDelta < 0.0 ||
        options_.contactForceWindowSize < 1 ||
        options_.contactOnSamples < 1 ||
        options_.contactForceDeltaOffSamples < 1) {
      throw std::runtime_error(
          "Invalid ANYmal joint-torque GRF contact classifier parameters");
    }
  }

  canonicalFootNames_.reserve(options_.footNames.size());
  for (const std::string& footName : options_.footNames) {
    canonicalFootNames_.push_back(canonicalFootName(footName));
  }
  previousContactActive_.assign(options_.footNames.size(), false);
  havePreviousContactStates_.assign(options_.footNames.size(), false);
  if (options_.contactClassifier == "joint_torque_grf") {
    forceDeltaStates_.resize(options_.footNames.size());
  }
}

std::optional<ContactEvent> AnymalContactAdapter::makeContactEvent(
    const anymal_msgs::msg::AnymalState& msg, const size_t eventIndex,
    Diagnostics* diagnostics) {
  if (options_.requireStateOk &&
      msg.state != anymal_msgs::msg::AnymalState::STATE_OK) {
    return std::nullopt;
  }

  std::vector<ContactMeasurement> activeContacts;
  std::vector<Vector3> worldRelativeContactPoints;
  const Vector3 bodyWorld(msg.pose.pose.position.x, msg.pose.pose.position.y,
                          msg.pose.pose.position.z);
  Eigen::Quaterniond worldFromBody(
      msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
      msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
  if (worldFromBody.norm() <= 1e-12) {
    worldFromBody = Eigen::Quaterniond::Identity();
  } else {
    worldFromBody.normalize();
  }

  if (diagnostics) {
    diagnostics->feet.clear();
    diagnostics->feet.resize(options_.footNames.size());
    for (size_t foot = 0; foot < options_.footNames.size(); ++foot) {
      diagnostics->feet[foot].footName = options_.footNames[foot];
    }
    for (const anymal_msgs::msg::Contact& contact : msg.contacts) {
      const std::optional<size_t> foot = footIndexForContactName(contact.name);
      if (!foot) {
        continue;
      }
      FootDiagnostic& diagnostic = diagnostics->feet[*foot];
      diagnostic.hasMeasuredContact = true;
      diagnostic.measuredWrenchZ =
          static_cast<double>(contact.wrench.force.z);
      diagnostic.measuredContactState = contact.state;
    }
  }

  if (options_.contactClassifier == "joint_torque_grf") {
    const std::vector<JointTorqueGrfEstimator::Result> estimates =
        jointTorqueGrfEstimator_->estimate(msg, options_.footNames);
    for (size_t foot = 0; foot < estimates.size(); ++foot) {
      const JointTorqueGrfEstimator::Result& estimate = estimates[foot];
      const bool hadPrevious = havePreviousContactStates_[foot];
      const bool previousActive = previousContactActive_[foot];
      const double verticalForce =
          options_.contactUseAbsoluteForceZ
              ? std::abs(estimate.bodyGroundReactionForce.z())
              : estimate.bodyGroundReactionForce.z();
      const bool active = classifyForceContact(
          estimate.valid ? verticalForce
                         : std::numeric_limits<double>::quiet_NaN(),
          foot);
      if (diagnostics) {
        FootDiagnostic& diagnostic = diagnostics->feet[foot];
        diagnostic.hasEstimatedGrf = estimate.valid;
        diagnostic.estimatedGrfZ =
            estimate.valid ? estimate.bodyGroundReactionForce.z() : 0.0;
        diagnostic.classifiedContact = active;
      }
      const bool touchdown = active && (!hadPrevious || !previousActive);
      previousContactActive_[foot] = active;
      havePreviousContactStates_[foot] = estimate.valid;
      if (!active) {
        continue;
      }

      ContactMeasurement measurement;
      measurement.foot = foot;
      measurement.bodyPoint =
          estimate.bodyFootPosition + options_.legImuOffsets[foot];
      measurement.touchdown = touchdown;
      activeContacts.push_back(measurement);
      worldRelativeContactPoints.push_back(
          worldFromBody * estimate.bodyFootPosition);
    }
  } else {

    for (const anymal_msgs::msg::Contact& contact : msg.contacts) {
      const std::optional<size_t> foot = footIndexForContactName(contact.name);
      if (!foot) {
        continue;
      }

      const bool hadPrevious = havePreviousContactStates_.at(*foot);
      const bool previousActive = previousContactActive_.at(*foot);
      const bool active = isActiveContact(contact);
      if (diagnostics) {
        diagnostics->feet[*foot].classifiedContact = active;
      }
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
      const Vector3 footWorld(contact.position.x, contact.position.y,
                              contact.position.z);
      worldRelativeContactPoints.push_back(footWorld - bodyWorld);
    }
  }

  if (activeContacts.empty()) {
    return std::nullopt;
  }

  ContactEvent event;
  event.index = eventIndex;
  event.timestampS = stampToSeconds(msg.header.stamp);
  event.activeContacts = std::move(activeContacts);
  event.worldRelativeContactPoints = std::move(worldRelativeContactPoints);
  event.hasWorldFromBody = true;
  event.worldFromBodyW = msg.pose.pose.orientation.w;
  event.worldFromBodyX = msg.pose.pose.orientation.x;
  event.worldFromBodyY = msg.pose.pose.orientation.y;
  event.worldFromBodyZ = msg.pose.pose.orientation.z;
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

bool AnymalContactAdapter::classifyForceContact(
    const double verticalForce, const size_t foot) {
  ForceDeltaState& state = forceDeltaStates_.at(foot);
  if (!std::isfinite(verticalForce)) {
    state = ForceDeltaState{};
    return false;
  }

  std::optional<double> forceDelta;
  if (state.previousForce) {
    forceDelta = std::abs(verticalForce - *state.previousForce);
    state.forceDeltas.push_back(*forceDelta);
    while (state.forceDeltas.size() > options_.contactForceWindowSize) {
      state.forceDeltas.pop_front();
    }
  }
  state.previousForce = verticalForce;

  if (state.active) {
    const bool deltaTooLarge =
        !forceDelta || *forceDelta > options_.contactMaxForceDelta;
    state.forceDeltaOffCount =
        deltaTooLarge ? state.forceDeltaOffCount + 1 : 0;
    state.active =
        verticalForce > options_.contactForceOff &&
        state.forceDeltaOffCount < options_.contactForceDeltaOffSamples;
    if (!state.active) {
      state.contactOnCount = 0;
      state.forceDeltaOffCount = 0;
    }
    return state.active;
  }

  bool stable = false;
  if (options_.contactUseForceWindow) {
    stable = state.forceDeltas.size() == options_.contactForceWindowSize;
    for (const double delta : state.forceDeltas) {
      stable = stable && delta <= options_.contactMaxForceDelta;
    }
  } else {
    stable = forceDelta && *forceDelta <= options_.contactMaxForceDelta;
  }

  state.contactOnCount =
      verticalForce >= options_.contactForceOn && stable
          ? state.contactOnCount + 1
          : 0;
  state.active = state.contactOnCount >= options_.contactOnSamples;
  if (state.active) {
    state.contactOnCount = 0;
    state.forceDeltaOffCount = 0;
  }
  return state.active;
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
