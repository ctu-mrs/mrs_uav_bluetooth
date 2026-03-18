// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/bridge/generic_message_bridge.hpp"

#include "mrs_uav_bluetooth/bridge/payload_codec.hpp"

#include <rclcpp/create_generic_publisher.hpp>
#include <rclcpp/create_generic_subscription.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/typesupport_helpers.hpp>
#include <rcpputils/shared_library.hpp>
#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include <algorithm>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <utility>

namespace mrs_uav_bluetooth::bridge {

namespace {

using rosidl_typesupport_introspection_cpp::MessageMember;
using rosidl_typesupport_introspection_cpp::MessageMembers;

struct MessageInstance {
    explicit MessageInstance(const MessageMembers& members)
        : members_(members), storage_(members.size_of_) {
        members_.init_function(storage_.data(), rosidl_runtime_cpp::MessageInitialization::ALL);
    }

    ~MessageInstance() {
        members_.fini_function(storage_.data());
    }

    void* data() { return storage_.data(); }
    const void* data() const { return storage_.data(); }

private:
    const MessageMembers& members_;
    std::vector<uint8_t> storage_;
};

const MessageMember& find_member(const MessageMembers& members, const std::string& name) {
    for (uint32_t index = 0; index < members.member_count_; ++index) {
        const auto& member = members.members_[index];
        if (name == member.name_) {
            return member;
        }
    }
    throw std::runtime_error("Unknown member path segment: " + name);
}

const MessageMembers& get_nested_members(const MessageMember& member) {
    const auto* type_support = static_cast<const rosidl_message_type_support_t*>(member.members_);
    if (type_support == nullptr || type_support->data == nullptr) {
        throw std::runtime_error("Missing nested message type support for member: " + std::string(member.name_));
    }
    return *static_cast<const MessageMembers*>(type_support->data);
}

const void* get_const_member_pointer(const void* message, const MessageMember& member, int index) {
    const auto* field = static_cast<const uint8_t*>(message) + member.offset_;
    if (!member.is_array_) {
        if (index >= 0) {
            throw std::runtime_error("Indexed access on non-array member: " + std::string(member.name_));
        }
        return field;
    }
    if (index < 0) {
        throw std::runtime_error("Array member requires explicit index: " + std::string(member.name_));
    }
    const size_t size = member.size_function ? member.size_function(field) : member.array_size_;
    if (static_cast<size_t>(index) >= size) {
        throw std::runtime_error("Array index out of range for member: " + std::string(member.name_));
    }
    if (!member.get_const_function) {
        throw std::runtime_error("Array member lacks const accessor: " + std::string(member.name_));
    }
    return member.get_const_function(field, static_cast<size_t>(index));
}

void* get_mutable_member_pointer(void* message, const MessageMember& member, int index) {
    auto* field = static_cast<uint8_t*>(message) + member.offset_;
    if (!member.is_array_) {
        if (index >= 0) {
            throw std::runtime_error("Indexed access on non-array member: " + std::string(member.name_));
        }
        return field;
    }
    if (index < 0) {
        throw std::runtime_error("Array member requires explicit index: " + std::string(member.name_));
    }
    if (member.resize_function) {
        const size_t size = member.size_function ? member.size_function(field) : member.array_size_;
        if (static_cast<size_t>(index) >= size) {
            member.resize_function(field, static_cast<size_t>(index + 1));
        }
    }
    const size_t size = member.size_function ? member.size_function(field) : member.array_size_;
    if (static_cast<size_t>(index) >= size) {
        throw std::runtime_error("Array index out of range for member: " + std::string(member.name_));
    }
    if (!member.get_function) {
        throw std::runtime_error("Array member lacks mutable accessor: " + std::string(member.name_));
    }
    return member.get_function(field, static_cast<size_t>(index));
}

const MessageMember& resolve_leaf_member(const MessageMembers& root_members,
                                         const void* root_message,
                                         const std::vector<PathSegment>& path,
                                         const void** leaf_pointer) {
    const MessageMembers* current_members = &root_members;
    const void* current_message = root_message;
    const MessageMember* current_member = nullptr;

    for (size_t index = 0; index < path.size(); ++index) {
        current_member = &find_member(*current_members, path[index].name);
        current_message = get_const_member_pointer(current_message, *current_member, path[index].index);
        if (index + 1 < path.size()) {
            if (current_member->type_id_ != rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
                throw std::runtime_error("Non-message member in the middle of path: " + path[index].name);
            }
            current_members = &get_nested_members(*current_member);
        }
    }

    *leaf_pointer = current_message;
    return *current_member;
}

const MessageMember& resolve_leaf_member_mutable(const MessageMembers& root_members,
                                                 void* root_message,
                                                 const std::vector<PathSegment>& path,
                                                 void** leaf_pointer) {
    const MessageMembers* current_members = &root_members;
    void* current_message = root_message;
    const MessageMember* current_member = nullptr;

    for (size_t index = 0; index < path.size(); ++index) {
        current_member = &find_member(*current_members, path[index].name);
        current_message = get_mutable_member_pointer(current_message, *current_member, path[index].index);
        if (index + 1 < path.size()) {
            if (current_member->type_id_ != rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
                throw std::runtime_error("Non-message member in the middle of path: " + path[index].name);
            }
            current_members = &get_nested_members(*current_member);
        }
    }

    *leaf_pointer = current_message;
    return *current_member;
}

uint64_t read_time_ns(const MessageMember& member, const void* value_ptr) {
    if (member.type_id_ != rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
        throw std::runtime_error("time_ns bridge member must target a ROS time field");
    }
    const auto& nested_members = get_nested_members(member);
    const auto& sec_member = find_member(nested_members, "sec");
    const auto& nanosec_member = find_member(nested_members, "nanosec");
    const auto* sec_ptr = static_cast<const uint8_t*>(value_ptr) + sec_member.offset_;
    const auto* nanosec_ptr = static_cast<const uint8_t*>(value_ptr) + nanosec_member.offset_;
    return stamp_to_ns(*reinterpret_cast<const int32_t*>(sec_ptr), *reinterpret_cast<const uint32_t*>(nanosec_ptr));
}

void write_time_ns(const MessageMember& member, void* value_ptr, uint64_t value_ns) {
    if (member.type_id_ != rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
        throw std::runtime_error("time_ns bridge member must target a ROS time field");
    }
    const auto& nested_members = get_nested_members(member);
    const auto& sec_member = find_member(nested_members, "sec");
    const auto& nanosec_member = find_member(nested_members, "nanosec");
    auto [sec, nanosec] = ns_to_stamp(value_ns);
    auto* sec_ptr = static_cast<uint8_t*>(value_ptr) + sec_member.offset_;
    auto* nanosec_ptr = static_cast<uint8_t*>(value_ptr) + nanosec_member.offset_;
    *reinterpret_cast<int32_t*>(sec_ptr) = sec;
    *reinterpret_cast<uint32_t*>(nanosec_ptr) = nanosec;
}

double read_numeric(const MessageMember& member, const void* value_ptr) {
    switch (member.type_id_) {
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL:
            return *static_cast<const bool*>(value_ptr) ? 1.0 : 0.0;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
            return *static_cast<const int8_t*>(value_ptr);
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
            return *static_cast<const uint8_t*>(value_ptr);
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
            return *static_cast<const int16_t*>(value_ptr);
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
            return *static_cast<const uint16_t*>(value_ptr);
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
            return *static_cast<const int32_t*>(value_ptr);
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
            return *static_cast<const uint32_t*>(value_ptr);
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
            return static_cast<double>(*static_cast<const int64_t*>(value_ptr));
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
            return static_cast<double>(*static_cast<const uint64_t*>(value_ptr));
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
            return *static_cast<const float*>(value_ptr);
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
            return *static_cast<const double*>(value_ptr);
        default:
            throw std::runtime_error("Unsupported bridge member leaf type");
    }
}

void write_numeric(const MessageMember& member, void* value_ptr, double value) {
    switch (member.type_id_) {
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL:
            *static_cast<bool*>(value_ptr) = value != 0.0;
            return;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
            *static_cast<int8_t*>(value_ptr) = static_cast<int8_t>(value);
            return;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
            *static_cast<uint8_t*>(value_ptr) = static_cast<uint8_t>(value);
            return;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
            *static_cast<int16_t*>(value_ptr) = static_cast<int16_t>(value);
            return;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
            *static_cast<uint16_t*>(value_ptr) = static_cast<uint16_t>(value);
            return;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
            *static_cast<int32_t*>(value_ptr) = static_cast<int32_t>(value);
            return;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
            *static_cast<uint32_t*>(value_ptr) = static_cast<uint32_t>(value);
            return;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
            *static_cast<int64_t*>(value_ptr) = static_cast<int64_t>(value);
            return;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
            *static_cast<uint64_t*>(value_ptr) = static_cast<uint64_t>(value);
            return;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
            *static_cast<float*>(value_ptr) = static_cast<float>(value);
            return;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
            *static_cast<double*>(value_ptr) = value;
            return;
        default:
            throw std::runtime_error("Unsupported bridge member leaf type");
    }
}

std::vector<uint8_t> serialized_to_bytes(const rclcpp::SerializedMessage& serialized_message) {
    const auto& raw = serialized_message.get_rcl_serialized_message();
    return std::vector<uint8_t>(raw.buffer, raw.buffer + raw.buffer_length);
}

rclcpp::SerializedMessage bytes_to_serialized(const std::vector<uint8_t>& payload) {
    rclcpp::SerializedMessage serialized_message(payload.size());
    auto& raw = serialized_message.get_rcl_serialized_message();
    if (!payload.empty()) {
        std::memcpy(raw.buffer, payload.data(), payload.size());
    }
    raw.buffer_length = payload.size();
    return serialized_message;
}

}  // namespace

struct GenericMessageBridge::Impl {
    explicit Impl(std::string type_name)
        : message_type(std::move(type_name)),
          cpp_library(rclcpp::get_typesupport_library(message_type, "rosidl_typesupport_cpp")),
          introspection_library(rclcpp::get_typesupport_library(message_type, "rosidl_typesupport_introspection_cpp")),
                    cpp_type_support(rclcpp::get_message_typesupport_handle(message_type, "rosidl_typesupport_cpp", *cpp_library)),
                    introspection_type_support(rclcpp::get_message_typesupport_handle(message_type, "rosidl_typesupport_introspection_cpp", *introspection_library)),
          members(*static_cast<const MessageMembers*>(introspection_type_support->data)),
          serializer(std::make_unique<rclcpp::SerializationBase>(cpp_type_support)) {}

    std::string message_type;
    std::shared_ptr<rcpputils::SharedLibrary> cpp_library;
    std::shared_ptr<rcpputils::SharedLibrary> introspection_library;
    const rosidl_message_type_support_t* cpp_type_support;
    const rosidl_message_type_support_t* introspection_type_support;
    const MessageMembers& members;
    std::unique_ptr<rclcpp::SerializationBase> serializer;
    mutable std::mutex mutex;
};

GenericMessageBridge::GenericMessageBridge(std::string message_type)
    : impl_(std::make_shared<Impl>(std::move(message_type))) {}

const std::string& GenericMessageBridge::message_type() const {
    return impl_->message_type;
}

std::shared_ptr<rclcpp::GenericSubscription> GenericMessageBridge::create_subscription(
    rclcpp::Node& node,
    const std::string& topic_name,
    std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback,
    size_t queue_depth) const {
    return rclcpp::create_generic_subscription(
        node.get_node_topics_interface(),
        topic_name,
        impl_->message_type,
        rclcpp::QoS(queue_depth),
        std::move(callback));
}

std::shared_ptr<rclcpp::GenericPublisher> GenericMessageBridge::create_publisher(
    rclcpp::Node& node,
    const std::string& topic_name,
    size_t queue_depth) const {
    return rclcpp::create_generic_publisher(
        node.get_node_topics_interface(),
        topic_name,
        impl_->message_type,
        rclcpp::QoS(queue_depth));
}

std::vector<uint8_t> GenericMessageBridge::encode_payload(
    const rclcpp::SerializedMessage& serialized_message,
    const std::vector<config::BridgeMemberSpec>& member_specs,
    const std::string& payload_format) const {
    if (payload_format == "ros2" || member_specs.empty()) {
        return serialized_to_bytes(serialized_message);
    }

    std::lock_guard<std::mutex> lock(impl_->mutex);
    MessageInstance instance(impl_->members);
    impl_->serializer->deserialize_message(&serialized_message, instance.data());

    std::vector<ScalarValue> values;
    values.reserve(member_specs.size());
    for (const auto& spec : member_specs) {
        const auto path = parse_member_path(spec.path);
        const void* leaf_ptr = nullptr;
        const auto& member = resolve_leaf_member(impl_->members, instance.data(), path, &leaf_ptr);
        if (spec.value_type == "time_ns") {
            values.push_back(static_cast<uint64_t>(read_time_ns(member, leaf_ptr)));
        } else {
            values.push_back(coerce_outgoing(read_numeric(member, leaf_ptr), spec.value_type));
        }
    }
    return encode_struct_payload(values, member_specs);
}

rclcpp::SerializedMessage GenericMessageBridge::decode_payload(
    const std::vector<uint8_t>& payload,
    const std::vector<config::BridgeMemberSpec>& member_specs,
    const std::string& payload_format) const {
    if (payload_format == "ros2" || member_specs.empty()) {
        return bytes_to_serialized(payload);
    }

    std::lock_guard<std::mutex> lock(impl_->mutex);
    MessageInstance instance(impl_->members);
    const auto values = decode_struct_payload(payload, member_specs);
    for (size_t index = 0; index < member_specs.size(); ++index) {
        const auto path = parse_member_path(member_specs[index].path);
        void* leaf_ptr = nullptr;
        const auto& member = resolve_leaf_member_mutable(impl_->members, instance.data(), path, &leaf_ptr);
        if (member_specs[index].value_type == "time_ns") {
            const auto value_ns = std::visit([](auto&& value) {
                return static_cast<uint64_t>(value);
            }, values[index]);
            write_time_ns(member, leaf_ptr, value_ns);
        } else {
            write_numeric(member, leaf_ptr, coerce_incoming(values[index], member_specs[index].value_type));
        }
    }

    rclcpp::SerializedMessage serialized_message;
    impl_->serializer->serialize_message(instance.data(), &serialized_message);
    return serialized_message;
}

}  // namespace mrs_uav_bluetooth::bridge