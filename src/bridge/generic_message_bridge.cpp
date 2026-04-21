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
#include <limits>
#include <mutex>
#include <optional>
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

constexpr size_t kDynamicBindingCount = std::numeric_limits<size_t>::max();

struct ConstLeafBinding {
    const MessageMember* member;
    const void* pointer;
};

struct MutableLeafBinding {
    const MessageMember* member;
    void* pointer;
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

bool is_selector_present(const PathSegment& segment) {
    return segment.index >= 0 || segment.has_slice;
}

bool is_dynamic_array_member(const MessageMember& member) {
    return member.is_array_ && member.resize_function != nullptr;
}

size_t fixed_array_size(const MessageMember& member) {
    return member.array_size_;
}

size_t array_size_const(const MessageMember& member, const void* field) {
    return member.size_function ? member.size_function(field) : member.array_size_;
}

size_t array_size_mutable(const MessageMember& member, void* field) {
    return member.size_function ? member.size_function(field) : member.array_size_;
}

size_t count_range(size_t start, size_t stop, size_t step) {
    if (start >= stop) {
        return 0;
    }
    return 1 + ((stop - start - 1) / step);
}

size_t selection_count_for_path(const MessageMembers& members,
                                const std::vector<PathSegment>& path,
                                size_t path_index) {
    const auto& segment = path[path_index];
    const auto& member = find_member(members, segment.name);
    const bool is_last = path_index + 1 == path.size();

    if (!member.is_array_) {
        if (is_selector_present(segment)) {
            throw std::runtime_error("Array selector applied to non-array member: " + segment.name);
        }
        if (is_last) {
            return 1;
        }
        if (member.type_id_ != rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
            throw std::runtime_error("Non-message member in the middle of path: " + segment.name);
        }
        return selection_count_for_path(get_nested_members(member), path, path_index + 1);
    }

    if (!is_last && !is_selector_present(segment)) {
        throw std::runtime_error("Non-leaf array member requires an explicit index or slice: " + segment.name);
    }

    size_t selected_count = 0;
    if (segment.index >= 0) {
        selected_count = 1;
    } else if (segment.has_slice) {
        const size_t step = static_cast<size_t>(segment.slice_step);
        if (segment.slice_stop.has_value()) {
            const size_t start = static_cast<size_t>(segment.slice_start.value_or(0));
            const size_t stop = static_cast<size_t>(*segment.slice_stop);
            if (is_dynamic_array_member(member)) {
                selected_count = count_range(start, stop, step);
            } else {
                const size_t size = fixed_array_size(member);
                if (stop > size) {
                    throw std::runtime_error("Slice stop is out of range for fixed-size array member: " + segment.name);
                }
                selected_count = count_range(start, stop, step);
            }
        } else {
            if (!is_last) {
                throw std::runtime_error("Open-ended slices are only supported on the final array path segment");
            }
            if (is_dynamic_array_member(member)) {
                return kDynamicBindingCount;
            }
            const size_t size = fixed_array_size(member);
            const size_t start = static_cast<size_t>(segment.slice_start.value_or(0));
            if (start > size) {
                throw std::runtime_error("Slice start is out of range for fixed-size array member: " + segment.name);
            }
            selected_count = count_range(start, size, step);
        }
    } else {
        if (!is_last) {
            throw std::runtime_error("Non-leaf array member requires an explicit index or slice: " + segment.name);
        }
        if (is_dynamic_array_member(member)) {
            return kDynamicBindingCount;
        }
        selected_count = fixed_array_size(member);
    }

    if (is_last) {
        return selected_count;
    }
    if (member.type_id_ != rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
        throw std::runtime_error("Non-message member in the middle of path: " + segment.name);
    }

    const size_t nested_count = selection_count_for_path(get_nested_members(member), path, path_index + 1);
    if (nested_count == kDynamicBindingCount) {
        throw std::runtime_error("Open-ended dynamic arrays are only supported on the final path segment");
    }
    return selected_count * nested_count;
}

std::vector<size_t> select_indices_const(const MessageMember& member,
                                         const PathSegment& segment,
                                         bool is_last,
                                         const void* field) {
    const size_t size = array_size_const(member, field);
    std::vector<size_t> indices;
    if (segment.index >= 0) {
        if (static_cast<size_t>(segment.index) >= size) {
            throw std::runtime_error("Array index out of range for member: " + segment.name);
        }
        indices.push_back(static_cast<size_t>(segment.index));
        return indices;
    }

    if (segment.has_slice) {
        const size_t start = static_cast<size_t>(segment.slice_start.value_or(0));
        const size_t stop = static_cast<size_t>(segment.slice_stop.value_or(static_cast<int>(size)));
        const size_t step = static_cast<size_t>(segment.slice_step);
        if (start > size || stop > size) {
            throw std::runtime_error("Slice is out of range for member: " + segment.name);
        }
        for (size_t index = start; index < stop; index += step) {
            indices.push_back(index);
        }
        return indices;
    }

    if (!is_last) {
        throw std::runtime_error("Non-leaf array member requires an explicit index or slice: " + segment.name);
    }
    indices.reserve(size);
    for (size_t index = 0; index < size; ++index) {
        indices.push_back(index);
    }
    return indices;
}

std::vector<size_t> select_indices_mutable(const MessageMember& member,
                                           const PathSegment& segment,
                                           bool is_last,
                                           void* field,
                                           const std::optional<size_t>& dynamic_count) {
    auto ensure_size = [&](size_t required_size) {
        const size_t current_size = array_size_mutable(member, field);
        if (required_size <= current_size) {
            return;
        }
        if (!member.resize_function) {
            throw std::runtime_error("Array selection exceeds fixed-size member: " + segment.name);
        }
        member.resize_function(field, required_size);
    };

    std::vector<size_t> indices;
    if (segment.index >= 0) {
        const auto index = static_cast<size_t>(segment.index);
        ensure_size(index + 1);
        indices.push_back(index);
        return indices;
    }

    if (segment.has_slice) {
        const size_t start = static_cast<size_t>(segment.slice_start.value_or(0));
        const size_t step = static_cast<size_t>(segment.slice_step);
        if (segment.slice_stop.has_value()) {
            const size_t stop = static_cast<size_t>(*segment.slice_stop);
            ensure_size(stop);
            for (size_t index = start; index < stop; index += step) {
                indices.push_back(index);
            }
            return indices;
        }

        size_t count = 0;
        if (dynamic_count.has_value()) {
            count = *dynamic_count;
            const size_t required_size = count == 0 ? start : start + (count - 1) * step + 1;
            ensure_size(required_size);
        } else {
            if (!is_last) {
                throw std::runtime_error("Open-ended slices are only supported on the final array path segment");
            }
            count = count_range(start, array_size_mutable(member, field), step);
        }
        indices.reserve(count);
        for (size_t item = 0; item < count; ++item) {
            indices.push_back(start + item * step);
        }
        return indices;
    }

    if (!is_last) {
        throw std::runtime_error("Non-leaf array member requires an explicit index or slice: " + segment.name);
    }

    size_t count = array_size_mutable(member, field);
    if (dynamic_count.has_value()) {
        count = *dynamic_count;
        ensure_size(count);
    }
    indices.reserve(count);
    for (size_t index = 0; index < count; ++index) {
        indices.push_back(index);
    }
    return indices;
}

void collect_const_leaf_bindings(const MessageMembers& members,
                                 const void* message,
                                 const std::vector<PathSegment>& path,
                                 size_t path_index,
                                 std::vector<ConstLeafBinding>& out) {
    const auto& segment = path[path_index];
    const auto& member = find_member(members, segment.name);
    const bool is_last = path_index + 1 == path.size();

    if (!member.is_array_) {
        if (is_selector_present(segment)) {
            throw std::runtime_error("Array selector applied to non-array member: " + segment.name);
        }
        const auto* field = static_cast<const uint8_t*>(message) + member.offset_;
        if (is_last) {
            out.push_back({&member, field});
            return;
        }
        if (member.type_id_ != rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
            throw std::runtime_error("Non-message member in the middle of path: " + segment.name);
        }
        collect_const_leaf_bindings(get_nested_members(member), field, path, path_index + 1, out);
        return;
    }

    const auto* field = static_cast<const uint8_t*>(message) + member.offset_;
    const auto indices = select_indices_const(member, segment, is_last, field);
    for (const auto index : indices) {
        if (!member.get_const_function) {
            throw std::runtime_error("Array member lacks const accessor: " + segment.name);
        }
        const void* element = member.get_const_function(field, index);
        if (is_last) {
            out.push_back({&member, element});
            continue;
        }
        if (member.type_id_ != rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
            throw std::runtime_error("Non-message member in the middle of path: " + segment.name);
        }
        collect_const_leaf_bindings(get_nested_members(member), element, path, path_index + 1, out);
    }
}

void collect_mutable_leaf_bindings(const MessageMembers& members,
                                   void* message,
                                   const std::vector<PathSegment>& path,
                                   size_t path_index,
                                   std::vector<MutableLeafBinding>& out,
                                   const std::optional<size_t>& dynamic_count) {
    const auto& segment = path[path_index];
    const auto& member = find_member(members, segment.name);
    const bool is_last = path_index + 1 == path.size();

    if (!member.is_array_) {
        if (is_selector_present(segment)) {
            throw std::runtime_error("Array selector applied to non-array member: " + segment.name);
        }
        auto* field = static_cast<uint8_t*>(message) + member.offset_;
        if (is_last) {
            out.push_back({&member, field});
            return;
        }
        if (member.type_id_ != rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
            throw std::runtime_error("Non-message member in the middle of path: " + segment.name);
        }
        collect_mutable_leaf_bindings(get_nested_members(member), field, path, path_index + 1, out, dynamic_count);
        return;
    }

    auto* field = static_cast<uint8_t*>(message) + member.offset_;
    const auto indices = select_indices_mutable(member, segment, is_last, field, dynamic_count);
    for (const auto index : indices) {
        if (!member.get_function) {
            throw std::runtime_error("Array member lacks mutable accessor: " + segment.name);
        }
        void* element = member.get_function(field, index);
        if (is_last) {
            out.push_back({&member, element});
            continue;
        }
        if (member.type_id_ != rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
            throw std::runtime_error("Non-message member in the middle of path: " + segment.name);
        }
        collect_mutable_leaf_bindings(get_nested_members(member), element, path, path_index + 1, out, dynamic_count);
    }
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
    std::vector<config::BridgeMemberSpec> flattened_specs;
    for (const auto& spec : member_specs) {
        const auto path = parse_member_path(spec.path);
        std::vector<ConstLeafBinding> bindings;
        collect_const_leaf_bindings(impl_->members, instance.data(), path, 0, bindings);
        flattened_specs.reserve(flattened_specs.size() + bindings.size());
        values.reserve(values.size() + bindings.size());
        for (const auto& binding : bindings) {
            flattened_specs.push_back({spec.path, spec.value_type});
            if (spec.value_type == "time_ns") {
                values.push_back(static_cast<uint64_t>(read_time_ns(*binding.member, binding.pointer)));
            } else {
                values.push_back(coerce_outgoing(read_numeric(*binding.member, binding.pointer), spec.value_type));
            }
        }
    }
    return encode_struct_payload(values, flattened_specs);
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

    size_t fixed_payload_bytes = 0;
    std::optional<size_t> dynamic_spec_index;
    size_t dynamic_value_size = 0;
    for (size_t index = 0; index < member_specs.size(); ++index) {
        const auto path = parse_member_path(member_specs[index].path);
        const size_t binding_count = selection_count_for_path(impl_->members, path, 0);
        const size_t value_size = wire_size(member_specs[index].value_type);
        if (binding_count == kDynamicBindingCount) {
            if (dynamic_spec_index.has_value()) {
                throw std::runtime_error("At most one open-ended dynamic array member may be decoded per bridge payload");
            }
            dynamic_spec_index = index;
            dynamic_value_size = value_size;
            continue;
        }
        fixed_payload_bytes += binding_count * value_size;
    }

    std::optional<size_t> dynamic_binding_count;
    if (dynamic_spec_index.has_value()) {
        if (payload.size() < fixed_payload_bytes) {
            throw std::runtime_error("Payload is shorter than the fixed-width bridge mapping requires");
        }
        const size_t remaining_bytes = payload.size() - fixed_payload_bytes;
        if (dynamic_value_size == 0 || remaining_bytes % dynamic_value_size != 0) {
            throw std::runtime_error("Payload size does not align with the open-ended array bridge mapping");
        }
        dynamic_binding_count = remaining_bytes / dynamic_value_size;
    } else if (payload.size() != fixed_payload_bytes) {
        throw std::runtime_error("Payload size does not match the configured bridge mapping");
    }

    std::vector<config::BridgeMemberSpec> flattened_specs;
    std::vector<MutableLeafBinding> flattened_bindings;
    for (size_t index = 0; index < member_specs.size(); ++index) {
        const auto path = parse_member_path(member_specs[index].path);
        std::vector<MutableLeafBinding> bindings;
        collect_mutable_leaf_bindings(
            impl_->members,
            instance.data(),
            path,
            0,
            bindings,
            dynamic_spec_index.has_value() && *dynamic_spec_index == index ? dynamic_binding_count : std::optional<size_t>{});
        flattened_specs.reserve(flattened_specs.size() + bindings.size());
        flattened_bindings.reserve(flattened_bindings.size() + bindings.size());
        for (auto& binding : bindings) {
            flattened_specs.push_back({member_specs[index].path, member_specs[index].value_type});
            flattened_bindings.push_back(binding);
        }
    }

    const auto values = decode_struct_payload(payload, flattened_specs);
    for (size_t index = 0; index < flattened_bindings.size(); ++index) {
        const auto& binding = flattened_bindings[index];
        const auto& spec = flattened_specs[index];
        if (spec.value_type == "time_ns") {
            const auto value_ns = std::visit([](auto&& value) {
                return static_cast<uint64_t>(value);
            }, values[index]);
            write_time_ns(*binding.member, binding.pointer, value_ns);
        } else {
            write_numeric(*binding.member, binding.pointer, coerce_incoming(values[index], spec.value_type));
        }
    }

    rclcpp::SerializedMessage serialized_message;
    impl_->serializer->serialize_message(instance.data(), &serialized_message);
    return serialized_message;
}

}  // namespace mrs_uav_bluetooth::bridge