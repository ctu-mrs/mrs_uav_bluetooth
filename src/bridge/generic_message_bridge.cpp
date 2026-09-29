// SPDX-License-Identifier: BSD-3-Clause
/// \file src/bridge/generic_message_bridge.cpp
/// \brief Implements the generic message bridge component of the transport-independent ROS message bridge.

#include "mrs_uav_bluetooth/bridge/generic_message_bridge.hpp"

#include "mrs_uav_bluetooth/bridge/payload_codec.hpp"
#include "mrs_uav_bluetooth/bridge/math_expression.hpp"

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
#include <cmath>
#include <cstring>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace mrs_uav_bluetooth::bridge {

namespace {

using rosidl_typesupport_introspection_cpp::MessageMember;
using rosidl_typesupport_introspection_cpp::MessageMembers;

struct MessageInstance {
    /// \brief Allocate aligned message storage and run the ROS type initialization hook.
    /// \param members ROS introspection schema used to allocate initialize and destroy the message.
    explicit MessageInstance(const MessageMembers& members)
        : members_(members), storage_(members.size_of_) {
        // Allocate aligned message storage and run the ROS type initialization hook.
        members_.init_function(storage_.data(), rosidl_runtime_cpp::MessageInitialization::ALL);
    }

    /// \brief Run the matching ROS finalizer before releasing the raw message storage.
    ~MessageInstance() {
        // Run the matching ROS finalizer before releasing the raw message storage.
        members_.fini_function(storage_.data());
    }

    /// \brief Expose mutable initialized storage to the ROS serializer and field writers.
    void* data() {
        // Expose mutable initialized storage to the ROS serializer and field writers.
        return storage_.data();
    }
    /// \brief Expose initialized storage to read-only field traversal and serialization.
    const void* data() const {
        // Expose initialized storage to read-only field traversal and serialization.
        return storage_.data();
    }

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

/// \brief Resolve one named field from ROS introspection metadata or reject it.
/// \param members ROS introspection schema searched for the named field.
/// \param name ROS field name to find in the introspection table.
/// \return Matching member, if present.
const MessageMember& find_member(const MessageMembers& members, const std::string& name) {
    // Resolve one named field from ROS introspection metadata or reject it.
    for (uint32_t index = 0; index < members.member_count_; ++index) {
        const auto& member = members.members_[index];
        if (name == member.name_) {
            return member;
        }
    }
    throw std::runtime_error("Unknown member path segment: " + name);
}

/// \brief Resolve introspection metadata for a nested ROS message field or reject missing support.
/// \param member ROS introspection metadata for the field being accessed.
/// \return Requested nested members.
const MessageMembers& get_nested_members(const MessageMember& member) {
    // Resolve introspection metadata for a nested ROS message field or reject missing support.
    const auto* type_support = static_cast<const rosidl_message_type_support_t*>(member.members_);
    if (type_support == nullptr || type_support->data == nullptr) {
        throw std::runtime_error("Missing nested message type support for member: " + std::string(member.name_));
    }
    return *static_cast<const MessageMembers*>(type_support->data);
}

/// \brief Test whether a parsed field path selects an array element or slice.
/// \param segment parsed member-path segment selecting a field or array range.
/// \return True if the path segment contains an array index or slice; otherwise false.
bool is_selector_present(const PathSegment& segment) {
    // Either a concrete index or a slice changes scalar field traversal.
    return segment.index >= 0 || segment.has_slice;
}

/// \brief Test whether ROS introspection exposes a resizable array field.
/// \param member ROS introspection metadata for the field being accessed.
/// \return True if ROS introspection marks the member as an unbounded array; otherwise false.
bool is_dynamic_array_member(const MessageMember& member) {
    // A resize callback distinguishes sequences from fixed-size arrays.
    return member.is_array_ && member.resize_function != nullptr;
}

/// \brief Return the compile-time length of a fixed ROS array member.
/// \param member ROS introspection metadata for the field being accessed.
/// \return Compile-time ROS array length or zero for nonarrays.
size_t fixed_array_size(const MessageMember& member) {
    // Return the compile-time length of a fixed ROS array member.
    return member.array_size_;
}

/// \brief Read a dynamic or fixed ROS array length from const introspection storage.
/// \param member ROS introspection metadata for the field being accessed.
/// \param field Address of the ROS array whose current size is queried.
/// \return Current number of readable elements in the ROS array field.
size_t array_size_const(const MessageMember& member, const void* field) {
    // Read a dynamic or fixed ROS array length from const introspection storage.
    return member.size_function ? member.size_function(field) : member.array_size_;
}

/// \brief Read a dynamic or fixed ROS array length from mutable introspection storage.
/// \param member ROS introspection metadata for the field being accessed.
/// \param field Address of the ROS array whose current size is queried.
/// \return Current number of writable elements in the ROS array field.
size_t array_size_mutable(const MessageMember& member, void* field) {
    // Read a dynamic or fixed ROS array length from mutable introspection storage.
    return member.size_function ? member.size_function(field) : member.array_size_;
}

/// \brief Validate an array slice and return the number of selected elements.
/// \param start First selected array index.
/// \param stop final iterator delimiting the selected array range.
/// \param step Positive slice stride.
/// \return Number of indices generated by the half-open slice.
size_t count_range(size_t start, size_t stop, size_t step) {
    // Validate an array slice and return the number of selected elements.
    if (start >= stop) {
        return 0;
    }
    return 1 + ((stop - start - 1) / step);
}

/// \brief Count how many scalar leaves a member path selects after array expansion.
/// \param members ROS introspection schema traversed by the parsed member path.
/// \param path Parsed member path whose selected leaf count is required.
/// \param path_index current segment position while walking a nested member path.
/// \return Number of scalar leaves selected by the remaining member path.
size_t selection_count_for_path(const MessageMembers& members,
                                const std::vector<PathSegment>& path,
                                size_t path_index) {
    // Count how many scalar leaves a member path selects after array expansion.
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

/// \brief Validate a path selector against a source array and return the selected indices.
/// \param member ROS introspection metadata for the field being accessed.
/// \param segment parsed member-path segment selecting a field or array range.
/// \param is_last whether this path segment selects the final scalar field.
/// \param field Address of the source ROS array whose selected indices are validated.
/// \return Validated source indices selected by the path segment.
std::vector<size_t> select_indices_const(const MessageMember& member,
                                         const PathSegment& segment,
                                         bool is_last,
                                         const void* field) {
    // Expand one source index or slice without ever resizing the message.
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

/// \brief Resize dynamic destination arrays when allowed and return selected indices.
/// \param member ROS introspection metadata for the field being accessed.
/// \param segment parsed member-path segment selecting a field or array range.
/// \param is_last whether this path segment selects the final scalar field.
/// \param field Address of the destination ROS array that may be resized.
/// \param dynamic_count requested element count when resizing a dynamic ROS array.
/// \return Validated destination indices after any permitted resize.
std::vector<size_t> select_indices_mutable(const MessageMember& member,
                                           const PathSegment& segment,
                                           bool is_last,
                                           void* field,
                                           const std::optional<size_t>& dynamic_count) {
    // Grow dynamic destinations only as far as the requested index or slice requires.
    auto ensure_size = [&](size_t required_size) {
        // Resize dynamic arrays once and report a range error for short fixed arrays.
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

/// \brief Traverse a source message path and collect every selected scalar address.
/// \param members ROS introspection schema for the current nested message.
/// \param message Root ROS message whose selected leaves are being collected.
/// \param path Parsed member path to traverse.
/// \param path_index current segment position while walking a nested member path.
/// \param out destination buffer, stream, or size receiving the result.
void collect_const_leaf_bindings(const MessageMembers& members,
                                 const void* message,
                                 const std::vector<PathSegment>& path,
                                 size_t path_index,
                                 std::vector<ConstLeafBinding>& out) {
    // Recurse through nested messages and append each selected source scalar.
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

/// \brief Traverse a destination path resize its dynamic leaf and collect scalar addresses.
/// \param members ROS introspection schema for the current nested message.
/// \param message Root ROS message whose selected leaves are being collected.
/// \param path Parsed member path to traverse.
/// \param path_index current segment position while walking a nested member path.
/// \param out destination buffer, stream, or size receiving the result.
/// \param dynamic_count requested element count when resizing a dynamic ROS array.
void collect_mutable_leaf_bindings(const MessageMembers& members,
                                   void* message,
                                   const std::vector<PathSegment>& path,
                                   size_t path_index,
                                   std::vector<MutableLeafBinding>& out,
                                   const std::optional<size_t>& dynamic_count) {
    // Recurse through nested messages and append each writable destination scalar.
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

/// \brief Resolve the selected scalar element without permitting mutation.
/// \param message ROS message or nested object containing the requested field.
/// \param member ROS introspection metadata for the field being accessed.
/// \param index Selected array index or -1 for a scalar field.
const void* get_const_member_pointer(const void* message, const MessageMember& member, int index) {
    // Resolve the selected scalar element without permitting mutation.
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

/// \brief Resolve the selected scalar element for an incoming payload assignment.
/// \param message ROS message or nested object containing the requested field.
/// \param member ROS introspection metadata for the field being accessed.
/// \param index Selected array index or -1 for a scalar field.
void* get_mutable_member_pointer(void* message, const MessageMember& member, int index) {
    // Resolve the selected scalar element for an incoming payload assignment.
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

/// \brief Resolve one scalar source path and return its introspection record and address.
/// \param root_members introspection metadata for the root ROS message.
/// \param root_message base address of the ROS message containing the selected field.
/// \param path Parsed member path to resolve.
/// \param leaf_pointer output address of the resolved scalar ROS field.
/// \return Resolved leaf member.
const MessageMember& resolve_leaf_member(const MessageMembers& root_members,
                                         const void* root_message,
                                         const std::vector<PathSegment>& path,
                                         const void** leaf_pointer) {
    // Follow one scalar path through nested introspection records and array indices.
    const MessageMembers* current_members = &root_members;
    const void* current_message = root_message;
    const MessageMember* current_member = nullptr;

    // Keep this resolver safe if a future caller bypasses parse_member_path().
    if (path.empty()) {
        throw std::invalid_argument("Member path must not be empty");
    }

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

/// \brief Resolve one scalar destination path and return its introspection record and address.
/// \param root_members introspection metadata for the root ROS message.
/// \param root_message base address of the ROS message containing the selected field.
/// \param path Parsed member path to resolve.
/// \param leaf_pointer output address of the resolved scalar ROS field.
/// \return Resolved leaf member mutable.
const MessageMember& resolve_leaf_member_mutable(const MessageMembers& root_members,
                                                 void* root_message,
                                                 const std::vector<PathSegment>& path,
                                                 void** leaf_pointer) {
    // Follow one scalar destination path while retaining its writable address.
    const MessageMembers* current_members = &root_members;
    void* current_message = root_message;
    const MessageMember* current_member = nullptr;

    // Keep this resolver safe if a future caller bypasses parse_member_path().
    if (path.empty()) {
        throw std::invalid_argument("Member path must not be empty");
    }

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

/// \brief Read a supported ROS scalar as a double without losing type validation.
/// \param member ROS introspection metadata for the field being accessed.
/// \param value_ptr address of the concrete ROS scalar field.
/// \return Decoded numeric.
double read_numeric(const MessageMember& member, const void* value_ptr) {
    // Convert any supported scalar ROS field to the expression evaluator double type.
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

/// \brief Read a supported integral ROS scalar into an exact signed-or-unsigned representation.
/// \param member ROS introspection metadata for the field being accessed.
/// \param value_ptr address of the concrete ROS scalar field.
/// \return Exact signed or unsigned integer or std::nullopt for nonintegral ROS types.
std::optional<ExactInteger> read_integer(const MessageMember& member,
                                          const void* value_ptr) {
    // Preserve integral ROS fields exactly and decline floating-point fields.
    switch (member.type_id_) {
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL:
            return *static_cast<const bool*>(value_ptr) ? 1 : 0;
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
            return *static_cast<const int64_t*>(value_ptr);
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
            return *static_cast<const uint64_t*>(value_ptr);
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
            return std::nullopt;
        default:
            throw std::runtime_error("Unsupported bridge member leaf type");
    }
}

/// \brief Range-check an exact transport integer for a ROS integer field.
/// \tparam Integer Concrete ROS signed or unsigned integer type.
/// \param value Exact decoded value to assign.
/// \return Value narrowed safely to Integer.
template<typename Integer>
Integer checked_ros_exact(ExactInteger value) {
    // Range-check an exact decoded integer before assigning a ROS integer field.
    if (value < static_cast<ExactInteger>(std::numeric_limits<Integer>::min()) ||
        value > static_cast<ExactInteger>(std::numeric_limits<Integer>::max())) {
        throw std::runtime_error("Decoded value is outside its ROS integer field range");
    }
    return static_cast<Integer>(value);
}

/// \brief Range-check an exact integer before storing it in the concrete ROS scalar type.
/// \param member ROS introspection metadata for the field being accessed.
/// \param value_ptr address of the concrete ROS scalar field.
/// \param value Exact decoded integer to range-check and assign.
void write_integer(const MessageMember& member, void* value_ptr,
                   ExactInteger value) {
    // Range-check an exact decoded integer before assigning the concrete ROS field type.
    switch (member.type_id_) {
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL:
            *static_cast<bool*>(value_ptr) = value != 0;
            return;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
            *static_cast<int8_t*>(value_ptr) = checked_ros_exact<int8_t>(value);
            return;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
            *static_cast<uint8_t*>(value_ptr) = checked_ros_exact<uint8_t>(value);
            return;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
            *static_cast<int16_t*>(value_ptr) = checked_ros_exact<int16_t>(value);
            return;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
            *static_cast<uint16_t*>(value_ptr) = checked_ros_exact<uint16_t>(value);
            return;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
            *static_cast<int32_t*>(value_ptr) = checked_ros_exact<int32_t>(value);
            return;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
            *static_cast<uint32_t*>(value_ptr) = checked_ros_exact<uint32_t>(value);
            return;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
            *static_cast<int64_t*>(value_ptr) = checked_ros_exact<int64_t>(value);
            return;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
            *static_cast<uint64_t*>(value_ptr) = checked_ros_exact<uint64_t>(value);
            return;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
            *static_cast<float*>(value_ptr) = static_cast<float>(value);
            return;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
            *static_cast<double*>(value_ptr) = static_cast<double>(value);
            return;
        default:
            throw std::runtime_error("Unsupported bridge member leaf type");
    }
}

/// \brief Round and range-check a floating expression result for a ROS integer field.
/// \tparam Integer Concrete ROS signed or unsigned integer type.
/// \param value Finite expression result to assign.
/// \return Rounded value narrowed safely to Integer.
template<typename Integer>
Integer checked_ros_integer(double value) {
    // Every expression is evaluated as double, even when its literals and
    // input wire values are integers. Round only at the final ROS assignment.
    // Use an exclusive power-of-two upper bound so uint64_t and int64_t do
    // not accidentally admit 2^64 or 2^63 after conversion to double.
    if (!std::isfinite(value)) {
        throw std::runtime_error("Cannot assign a non-finite value to a ROS integer field");
    }
    const double rounded = std::round(value);
    const double upper = std::ldexp(1.0, std::numeric_limits<Integer>::digits);
    const double lower = std::numeric_limits<Integer>::is_signed ? -upper : 0.0;
    if (rounded < lower || rounded >= upper) {
        throw std::runtime_error("Decoded value is outside its ROS integer field range");
    }
    return static_cast<Integer>(rounded);
}

/// \brief Range-check and convert a floating result into the concrete ROS scalar type.
/// \param member ROS introspection metadata for the field being accessed.
/// \param value_ptr address of the concrete ROS scalar field.
/// \param value Evaluated numeric result to range-check and assign.
void write_numeric(const MessageMember& member, void* value_ptr, double value) {
    // Convert an evaluated double to the concrete ROS scalar type with range checks.
    switch (member.type_id_) {
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL:
            *static_cast<bool*>(value_ptr) = value != 0.0;
            return;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
            *static_cast<int8_t*>(value_ptr) = checked_ros_integer<int8_t>(value);
            return;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
            *static_cast<uint8_t*>(value_ptr) = checked_ros_integer<uint8_t>(value);
            return;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
            *static_cast<int16_t*>(value_ptr) = checked_ros_integer<int16_t>(value);
            return;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
            *static_cast<uint16_t*>(value_ptr) = checked_ros_integer<uint16_t>(value);
            return;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
            *static_cast<int32_t*>(value_ptr) = checked_ros_integer<int32_t>(value);
            return;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
            *static_cast<uint32_t*>(value_ptr) = checked_ros_integer<uint32_t>(value);
            return;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
            *static_cast<int64_t*>(value_ptr) = checked_ros_integer<int64_t>(value);
            return;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
            *static_cast<uint64_t*>(value_ptr) = checked_ros_integer<uint64_t>(value);
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

/// Resolve an expression identifier against a numeric ROS leaf. Parsing the
/// member path here keeps the expression evaluator independent of ROS types.
/// \param members Root ROS introspection schema used to resolve the expression field path.
/// \param message ROS message supplying the expression variable.
/// \param path ROS member path named by the expression variable.
/// \return Decoded expression field.
double read_expression_field(const MessageMembers& members,
                             const void* message, const std::string& path) {
    // Resolve the expression path to a ROS leaf and read it as a numeric value.
    const void* field = nullptr;
    const auto& member = resolve_leaf_member(
        members, message, parse_member_path(path), &field);
    return read_numeric(member, field);
}

/// \brief Resolve an expression variable as an exact integer when its ROS type permits.
/// \param members Root ROS introspection schema used to resolve the expression field path.
/// \param message ROS message supplying the expression variable.
/// \param path ROS member path named by the expression variable.
/// \return Exact integer field value or std::nullopt for a nonintegral field.
std::optional<ExactInteger> read_expression_integer_field(
    const MessageMembers& members, const void* message,
    const std::string& path) {
    // Resolve the expression path and preserve an integral ROS leaf exactly.
    const void* field = nullptr;
    const auto& member = resolve_leaf_member(
        members, message, parse_member_path(path), &field);
    return read_integer(member, field);
}

/// \brief Resolve an assignment path and store its evaluated floating-point result.
/// \param members Root ROS introspection schema used to resolve the expression field path.
/// \param message ROS message receiving the assignment.
/// \param path ROS member path of the assignment target.
/// \param value Floating-point expression result to assign.
void write_expression_field(const MessageMembers& members, void* message,
                            const std::string& path, double value) {
    // Resolve the assignment target and write the evaluated numeric result.
    void* field = nullptr;
    const auto& member = resolve_leaf_member_mutable(
        members, message, parse_member_path(path), &field);
    write_numeric(member, field, value);
}

/// \brief Resolve an assignment path and store its exact integer result.
/// \param members Root ROS introspection schema used to resolve the expression field path.
/// \param message ROS message receiving the assignment.
/// \param path ROS member path of the assignment target.
/// \param value Exact integer expression result to assign.
void write_expression_integer_field(const MessageMembers& members,
                                    void* message, const std::string& path,
                                    ExactInteger value) {
    // Resolve the assignment target and write the exact integral result.
    void* field = nullptr;
    const auto& member = resolve_leaf_member_mutable(
        members, message, parse_member_path(path), &field);
    write_integer(member, field, value);
}

/// \brief Copy the used portion of a serialized ROS message into the transport payload.
/// \param serialized_message ROS serialized buffer to encode or decode.
/// \return Owned copy of the ROS serialized buffer's used bytes.
std::vector<uint8_t> serialized_to_bytes(const rclcpp::SerializedMessage& serialized_message) {
    // Copy the used portion of a serialized ROS message into the transport payload.
    const auto& raw = serialized_message.get_rcl_serialized_message();
    return std::vector<uint8_t>(raw.buffer, raw.buffer + raw.buffer_length);
}

/// \brief Copy transport bytes into an owned ROS serialized-message buffer.
/// \param payload Raw ROS serialization bytes to copy into owned storage.
/// \return ROS serialized message owning a copy of the transport bytes.
rclcpp::SerializedMessage bytes_to_serialized(const std::vector<uint8_t>& payload) {
    // Copy transport bytes into an owned ROS serialized-message buffer.
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
    /// \brief Load serialization and introspection type support for this runtime ROS message type.
    /// \param type_name fully qualified ROS type whose support libraries are loaded.
    explicit Impl(std::string type_name)
        : message_type(std::move(type_name)),
          cpp_library(rclcpp::get_typesupport_library(message_type, "rosidl_typesupport_cpp")),
          introspection_library(rclcpp::get_typesupport_library(message_type, "rosidl_typesupport_introspection_cpp")),
                    cpp_type_support(rclcpp::get_message_typesupport_handle(message_type, "rosidl_typesupport_cpp", *cpp_library)),
                    introspection_type_support(rclcpp::get_message_typesupport_handle(message_type, "rosidl_typesupport_introspection_cpp", *introspection_library)),
          members(*static_cast<const MessageMembers*>(introspection_type_support->data)),
          serializer(std::make_unique<rclcpp::SerializationBase>(cpp_type_support)) {
              // Load serialization and introspection type support for this runtime ROS message type.
          }

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
    : impl_(std::make_shared<Impl>(std::move(message_type))) {
        // Share immutable type support and a serialized-message codec for this ROS type.
    }

const std::string& GenericMessageBridge::message_type() const {
    // Expose the runtime ROS type loaded by this generic bridge.
    return impl_->message_type;
}

std::shared_ptr<rclcpp::GenericSubscription> GenericMessageBridge::create_subscription(
    rclcpp::Node& node,
    const std::string& topic_name,
    std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback,
    size_t queue_depth) const {
    // Create a type-erased ROS subscription using this bridge message type and queue depth.
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
    // Create a type-erased ROS publisher using this bridge message type and queue depth.
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
    // Pass through ROS serialization or extract configured fields into the compact codec.
    if (payload_format == "ros2" || member_specs.empty()) {
        return serialized_to_bytes(serialized_message);
    }

    std::lock_guard<std::mutex> lock(impl_->mutex);
    MessageInstance instance(impl_->members);
    impl_->serializer->deserialize_message(&serialized_message, instance.data());

    std::vector<ScalarValue> values;
    std::vector<config::BridgeMemberSpec> flattened_specs;
    for (const auto& spec : member_specs) {
        if (!spec.expression.empty()) {
            const auto exact = try_evaluate_integer_expression(
                spec.expression, [&](const std::string& identifier) {
                    // Resolve each identifier as an exact integer so bit-width checks remain lossless.
                    return read_expression_integer_field(
                        impl_->members, instance.data(), identifier);
                });
            if (exact) {
                values.push_back(coerce_outgoing_integer(*exact, spec));
            } else {
                const auto numeric = evaluate_math_expression(
                    spec.expression, [&](const std::string& identifier) {
                        // Resolve each identifier as a numeric ROS field for floating-point expression evaluation.
                        return read_expression_field(
                            impl_->members, instance.data(), identifier);
                    });
                values.push_back(coerce_outgoing(numeric, spec));
            }
            flattened_specs.push_back(spec);
            continue;
        }
        const auto path = parse_member_path(spec.target);
        std::vector<ConstLeafBinding> bindings;
        collect_const_leaf_bindings(impl_->members, instance.data(), path, 0, bindings);
        flattened_specs.reserve(flattened_specs.size() + bindings.size());
        values.reserve(values.size() + bindings.size());
        for (const auto& binding : bindings) {
            flattened_specs.push_back(spec);
            const auto exact = read_integer(*binding.member, binding.pointer);
            values.push_back(exact
                ? coerce_outgoing_integer(*exact, spec)
                : coerce_outgoing(read_numeric(*binding.member, binding.pointer), spec));
        }
    }
    return encode_struct_payload(values, flattened_specs);
}

rclcpp::SerializedMessage GenericMessageBridge::decode_payload(
    const std::vector<uint8_t>& payload,
    const std::vector<config::BridgeMemberSpec>& member_specs,
    const std::string& payload_format,
    const std::vector<config::BridgeAssignmentSpec>& decode_assignments) const {
    if (payload_format == "ros2" || member_specs.empty()) {
        return bytes_to_serialized(payload);
    }

    std::lock_guard<std::mutex> lock(impl_->mutex);
    MessageInstance instance(impl_->members);

    size_t fixed_payload_bytes = 0;
    std::optional<size_t> dynamic_spec_index;
    size_t dynamic_value_size = 0;
    for (size_t index = 0; index < member_specs.size(); ++index) {
        const size_t binding_count = member_specs[index].expression.empty()
            ? selection_count_for_path(impl_->members,
                                       parse_member_path(member_specs[index].target), 0)
            : 1;
        const size_t value_size = wire_size(member_specs[index]);
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
    std::vector<std::optional<MutableLeafBinding>> flattened_bindings;
    for (size_t index = 0; index < member_specs.size(); ++index) {
        if (!member_specs[index].expression.empty()) {
            flattened_specs.push_back(member_specs[index]);
            flattened_bindings.push_back(std::nullopt);
            continue;
        }
        const auto path = parse_member_path(member_specs[index].target);
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
            flattened_specs.push_back(member_specs[index]);
            flattened_bindings.push_back(binding);
        }
    }

    const auto values = decode_struct_payload(payload, flattened_specs);
    std::unordered_map<std::string, double> decoded_names;
    std::unordered_map<std::string, ExactInteger> decoded_integers;
    for (size_t index = 0; index < flattened_bindings.size(); ++index) {
        const auto& spec = flattened_specs[index];
        decoded_names.emplace(spec.target, coerce_incoming(values[index], spec));
        const auto exact = coerce_incoming_integer(values[index]);
        if (exact) decoded_integers.emplace(spec.target, *exact);
        if (!flattened_bindings[index]) continue;
        const auto& binding = *flattened_bindings[index];
        if (exact) {
            write_integer(*binding.member, binding.pointer, *exact);
        } else {
            write_numeric(*binding.member, binding.pointer,
                          coerce_incoming(values[index], spec));
        }
    }

    // Run reconstruction after all wire scalars are decoded. This allows any
    // assignment to depend on several members, such as three orientation
    // angles yielding four quaternion fields, without a codec-specific path.
    for (const auto& assignment : decode_assignments) {
        const auto exact = try_evaluate_integer_expression(
            assignment.expression, [&](const std::string& name)
                -> std::optional<ExactInteger> {
                // Resolve decoded integral names exactly; force a floating evaluation for other known names.
                const auto found = decoded_integers.find(name);
                if (found != decoded_integers.end()) return found->second;
                if (decoded_names.find(name) == decoded_names.end()) {
                    throw std::runtime_error("Unknown decoded member: " + name);
                }
                return std::nullopt;
            });
        if (exact) {
            write_expression_integer_field(impl_->members, instance.data(),
                                           assignment.target, *exact);
            continue;
        }
        const double result = evaluate_math_expression(
            assignment.expression, [&](const std::string& name) {
                // Resolve each expression name from the scalars decoded earlier in this payload.
                const auto found = decoded_names.find(name);
                if (found == decoded_names.end()) {
                    throw std::runtime_error("Unknown decoded member: " + name);
                }
                return found->second;
            });
        write_expression_field(impl_->members, instance.data(),
                               assignment.target, result);
    }

    rclcpp::SerializedMessage serialized_message;
    impl_->serializer->serialize_message(instance.data(), &serialized_message);
    return serialized_message;
}

}  // namespace mrs_uav_bluetooth::bridge
