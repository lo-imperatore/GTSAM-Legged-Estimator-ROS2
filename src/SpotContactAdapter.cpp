#include "include/SpotContactAdapter.h"

#include <rclcpp/serialization.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/typesupport_helpers.hpp>
#include <rcpputils/shared_library.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include <gtsam/geometry/Rot3.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <new>
#include <stdexcept>
#include <utility>
#include <vector>

namespace gtsam {
namespace {

namespace introspection = rosidl_typesupport_introspection_cpp;

const introspection::MessageMember* findMember(
    const introspection::MessageMembers* members, const std::string& name) {
  for (uint32_t index = 0; index < members->member_count_; ++index) {
    const introspection::MessageMember& member = members->members_[index];
    if (member.name_ == name) {
      return &member;
    }
  }
  return nullptr;
}

const void* fieldPtr(const void* message,
                     const introspection::MessageMember* member) {
  return static_cast<const uint8_t*>(message) + member->offset_;
}

const introspection::MessageMembers* nestedMembers(
    const introspection::MessageMember* member) {
  if (member == nullptr || member->type_id_ != introspection::ROS_TYPE_MESSAGE ||
      member->members_ == nullptr || member->members_->data == nullptr) {
    throw std::runtime_error("Expected nested ROS message field");
  }
  return static_cast<const introspection::MessageMembers*>(
      member->members_->data);
}

double readNumericField(const void* message,
                        const introspection::MessageMembers* members,
                        const std::string& name) {
  const introspection::MessageMember* member = findMember(members, name);
  if (member == nullptr) {
    throw std::runtime_error("Missing numeric field: " + name);
  }
  const void* ptr = fieldPtr(message, member);
  switch (member->type_id_) {
    case introspection::ROS_TYPE_FLOAT:
      return static_cast<double>(*static_cast<const float*>(ptr));
    case introspection::ROS_TYPE_DOUBLE:
      return *static_cast<const double*>(ptr);
    case introspection::ROS_TYPE_UINT8:
    case introspection::ROS_TYPE_OCTET:
      return static_cast<double>(*static_cast<const uint8_t*>(ptr));
    case introspection::ROS_TYPE_INT8:
      return static_cast<double>(*static_cast<const int8_t*>(ptr));
    case introspection::ROS_TYPE_UINT16:
      return static_cast<double>(*static_cast<const uint16_t*>(ptr));
    case introspection::ROS_TYPE_INT16:
      return static_cast<double>(*static_cast<const int16_t*>(ptr));
    case introspection::ROS_TYPE_UINT32:
      return static_cast<double>(*static_cast<const uint32_t*>(ptr));
    case introspection::ROS_TYPE_INT32:
      return static_cast<double>(*static_cast<const int32_t*>(ptr));
    case introspection::ROS_TYPE_UINT64:
      return static_cast<double>(*static_cast<const uint64_t*>(ptr));
    case introspection::ROS_TYPE_INT64:
      return static_cast<double>(*static_cast<const int64_t*>(ptr));
    default:
      throw std::runtime_error("Field is not numeric: " + name);
  }
}

double stampToSec(const void* message,
                  const introspection::MessageMembers* members) {
  const introspection::MessageMember* headerMember =
      findMember(members, "header");
  if (headerMember == nullptr) {
    throw std::runtime_error("FootStateArray is missing header");
  }
  const void* header = fieldPtr(message, headerMember);
  const introspection::MessageMembers* headerMembers =
      nestedMembers(headerMember);
  const introspection::MessageMember* stampMember =
      findMember(headerMembers, "stamp");
  if (stampMember == nullptr) {
    throw std::runtime_error("Header is missing stamp");
  }
  const void* stamp = fieldPtr(header, stampMember);
  const introspection::MessageMembers* stampMembers =
      nestedMembers(stampMember);
  const double sec = readNumericField(stamp, stampMembers, "sec");
  const double nanosec = readNumericField(stamp, stampMembers, "nanosec");
  return sec + nanosec * 1e-9;
}

class DynamicRosMessage {
 public:
  explicit DynamicRosMessage(const introspection::MessageMembers* members)
      : members_(members) {
    data_ = ::operator new(members_->size_of_);
    members_->init_function(data_,
                            rosidl_runtime_cpp::MessageInitialization::ALL);
  }

  ~DynamicRosMessage() {
    if (data_ != nullptr) {
      members_->fini_function(data_);
      ::operator delete(data_);
    }
  }

  DynamicRosMessage(const DynamicRosMessage&) = delete;
  DynamicRosMessage& operator=(const DynamicRosMessage&) = delete;

  void* data() { return data_; }
  const void* data() const { return data_; }

 private:
  const introspection::MessageMembers* members_ = nullptr;
  void* data_ = nullptr;
};

}  // namespace

class SpotContactAdapter::Impl {
 public:
  explicit Impl(Options options) : options_(std::move(options)) {
    if (options_.footNames.empty()) {
      throw std::runtime_error("Spot contact adapter needs at least one foot");
    }
    if (options_.legImuOffsets.empty()) {
      options_.legImuOffsets.assign(options_.footNames.size(),
                                    Vector3::Zero());
    }
    if (options_.legImuOffsets.size() != options_.footNames.size()) {
      throw std::runtime_error(
          "Spot contact adapter footNames and legImuOffsets sizes differ");
    }
    if (options_.fkPositionIndices.size() != 12) {
      throw std::runtime_error(
          "Spot contact adapter fkPositionIndices must contain 12 values");
    }

    setupFootTypeSupport();
    inContact_.assign(options_.footNames.size(), false);
    previousContactSet_.assign(options_.footNames.size(), false);
  }

  const std::string& footType() const { return options_.footType; }

  void updateJointState(const sensor_msgs::msg::JointState& msg) {
    latestJointPositions_.assign(msg.position.begin(), msg.position.end());
    lastJointTimestampS_ = rclcpp::Time(msg.header.stamp).seconds();
    haveJointState_ = true;
  }

  std::optional<ContactEvent> makeContactEvent(
      rclcpp::SerializedMessage& serialized, const size_t eventIndex) {
    DynamicRosMessage message(footMembers_);
    serialization_->deserialize_message(&serialized, message.data());
    return parseFootMessage(message.data(), eventIndex);
  }

 private:
  void setupFootTypeSupport() {
    cppTypeSupportLibrary_ = rclcpp::get_typesupport_library(
        options_.footType, "rosidl_typesupport_cpp");
    cppTypeSupport_ = rclcpp::get_message_typesupport_handle(
        options_.footType, "rosidl_typesupport_cpp", *cppTypeSupportLibrary_);
    introspectionTypeSupportLibrary_ = rclcpp::get_typesupport_library(
        options_.footType, "rosidl_typesupport_introspection_cpp");
    introspectionTypeSupport_ = rclcpp::get_message_typesupport_handle(
        options_.footType, "rosidl_typesupport_introspection_cpp",
        *introspectionTypeSupportLibrary_);
    footMembers_ = static_cast<const introspection::MessageMembers*>(
        introspectionTypeSupport_->data);
    serialization_ =
        std::make_unique<rclcpp::SerializationBase>(cppTypeSupport_);
  }

  std::optional<ContactEvent> parseFootMessage(const void* message,
                                               const size_t eventIndex) {
    const double timestampS = stampToSec(message, footMembers_);
    const introspection::MessageMember* statesMember =
        findMember(footMembers_, "states");
    if (statesMember == nullptr || !statesMember->is_array_ ||
        statesMember->size_function == nullptr ||
        statesMember->get_const_function == nullptr) {
      throw std::runtime_error("FootStateArray.states is not a sequence");
    }

    const void* statesField = fieldPtr(message, statesMember);
    const size_t usable = std::min(statesMember->size_function(statesField),
                                   options_.footNames.size());
    const introspection::MessageMembers* stateMembers =
        nestedMembers(statesMember);

    std::vector<bool> currentContactSet(options_.footNames.size(), false);
    std::vector<ContactMeasurement> activeContacts;

    for (size_t foot = 0; foot < usable; ++foot) {
      const void* state = statesMember->get_const_function(statesField, foot);
      const int contactCode =
          static_cast<int>(readNumericField(state, stateMembers, "contact"));
      if (options_.contactStreamMode == "state") {
        inContact_[foot] = contactCode == options_.contactStateValue;
      } else {
        if (contactCode == options_.contactMadeCode) {
          inContact_[foot] = true;
        } else if (contactCode == options_.contactLostCode) {
          inContact_[foot] = false;
        }
      }
      if (!inContact_[foot]) {
        continue;
      }

      currentContactSet[foot] = true;
      Vector3 bodyPoint = readFootBodyPoint(state, stateMembers);
      if (options_.useJointFkForBodyPoint) {
        const std::optional<Vector3> fk =
            computeBodyPointFromJointState(foot, timestampS);
        if (fk) {
          bodyPoint = *fk;
        } else if (!options_.jointFkFallbackToMsgBodyPoint) {
          continue;
        }
      }
      bodyPoint += options_.legImuOffsets.at(foot);

      ContactMeasurement measurement;
      measurement.foot = foot;
      measurement.bodyPoint = bodyPoint;
      activeContacts.push_back(measurement);
    }

    if (activeContacts.empty()) {
      return std::nullopt;
    }

    const bool contactSetChanged =
        !havePreviousContactSet_ || currentContactSet != previousContactSet_;
    for (ContactMeasurement& measurement : activeContacts) {
      measurement.touchdown = contactSetChanged;
    }
    previousContactSet_ = std::move(currentContactSet);
    havePreviousContactSet_ = true;

    ContactEvent event;
    event.index = eventIndex;
    event.timestampS = timestampS;
    event.activeContacts = std::move(activeContacts);
    return event;
  }

  Vector3 readFootBodyPoint(
      const void* state,
      const introspection::MessageMembers* stateMembers) const {
    const introspection::MessageMember* pointMember =
        findMember(stateMembers, "foot_position_rt_body");
    if (pointMember == nullptr) {
      throw std::runtime_error(
          "FootState is missing foot_position_rt_body field");
    }
    const void* point = fieldPtr(state, pointMember);
    const introspection::MessageMembers* pointMembers =
        nestedMembers(pointMember);
    return Vector3(readNumericField(point, pointMembers, "x"),
                   readNumericField(point, pointMembers, "y"),
                   readNumericField(point, pointMembers, "z"));
  }

  std::optional<Vector3> computeBodyPointFromJointState(
      const size_t foot, const double measurementTimestampS) const {
    static const std::array<std::array<Vector3, 4>, 4> kSpotLegChains{{
        {Vector3(0.0, 0.055, 0.0), Vector3(0.0, 0.110945, 0.0),
         Vector3(0.025, 0.0, -0.3205), Vector3(0.0, 0.0, -0.34)},
        {Vector3(0.0, -0.055, 0.0), Vector3(0.0, -0.110945, 0.0),
         Vector3(0.025, 0.0, -0.3205), Vector3(0.0, 0.0, -0.34)},
        {Vector3(-0.5957, 0.055, 0.0), Vector3(0.0, 0.110945, 0.0),
         Vector3(0.025, 0.0, -0.3205), Vector3(0.0, 0.0, -0.34)},
        {Vector3(-0.5957, -0.055, 0.0), Vector3(0.0, -0.110945, 0.0),
         Vector3(0.025, 0.0, -0.3205), Vector3(0.0, 0.0, -0.34)}}};

    if (foot >= kSpotLegChains.size() || !haveJointState_) {
      return std::nullopt;
    }
    if (options_.jointFkMaxJointAgeSeconds > 0.0 &&
        std::abs(measurementTimestampS - lastJointTimestampS_) >
            options_.jointFkMaxJointAgeSeconds) {
      return std::nullopt;
    }

    const size_t i0 = options_.fkPositionIndices.at(3 * foot + 0);
    const size_t i1 = options_.fkPositionIndices.at(3 * foot + 1);
    const size_t i2 = options_.fkPositionIndices.at(3 * foot + 2);
    if (i0 >= latestJointPositions_.size() ||
        i1 >= latestJointPositions_.size() ||
        i2 >= latestJointPositions_.size()) {
      return std::nullopt;
    }

    const Rot3 a12 = Rot3::Rx(latestJointPositions_[i0]);
    const Rot3 a23 = Rot3::Ry(latestJointPositions_[i1]);
    const Rot3 a34 = Rot3::Ry(latestJointPositions_[i2]);
    const Rot3 a123 = a12 * a23;
    const Rot3 a1234 = a123 * a34;
    const auto& chain = kSpotLegChains[foot];
    return a1234.matrix() * chain[3] + a123.matrix() * chain[2] +
           a12.matrix() * chain[1] + chain[0];
  }

  Options options_;
  std::shared_ptr<rcpputils::SharedLibrary> cppTypeSupportLibrary_;
  std::shared_ptr<rcpputils::SharedLibrary> introspectionTypeSupportLibrary_;
  const rosidl_message_type_support_t* cppTypeSupport_ = nullptr;
  const rosidl_message_type_support_t* introspectionTypeSupport_ = nullptr;
  const introspection::MessageMembers* footMembers_ = nullptr;
  std::unique_ptr<rclcpp::SerializationBase> serialization_;
  std::vector<double> latestJointPositions_;
  bool haveJointState_ = false;
  double lastJointTimestampS_ = 0.0;
  std::vector<bool> inContact_;
  std::vector<bool> previousContactSet_;
  bool havePreviousContactSet_ = false;
};

SpotContactAdapter::SpotContactAdapter(Options options)
    : impl_(std::make_unique<Impl>(std::move(options))) {}

SpotContactAdapter::~SpotContactAdapter() = default;

const std::string& SpotContactAdapter::footType() const {
  return impl_->footType();
}

void SpotContactAdapter::updateJointState(
    const sensor_msgs::msg::JointState& msg) {
  impl_->updateJointState(msg);
}

std::optional<ContactEvent> SpotContactAdapter::makeContactEvent(
    rclcpp::SerializedMessage& serialized, const size_t eventIndex) {
  return impl_->makeContactEvent(serialized, eventIndex);
}

}  // namespace gtsam
