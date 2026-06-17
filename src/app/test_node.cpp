// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/app/test_node.hpp"

#include "mrs_uav_bluetooth/util/hostname_utils.hpp"
#include "mrs_uav_bluetooth/util/topic_utils.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace {

constexpr const char* kDefaultOdometryTopic = "/{hostname}/mavros/local_position/odom";
constexpr const char* kDefaultAdvertisementTopic = "/{hostname}/ble/adv_local_extra";
constexpr const char* kDefaultAdvertisementObserveTopic = "/{hostname}/ble/advertisements";
constexpr const char* kDefaultPeerTopicPrefix = "/{hostname}/ble/peers";
constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kTimestampPayloadSize = 7;
constexpr uint64_t kTimestampResolutionNs = 1000;
constexpr std::size_t kPackedOdometryDataSize = 21;
constexpr std::size_t kFixedFieldCount = 12;
constexpr std::size_t kFixedOdometryPayloadSize = kTimestampPayloadSize + kPackedOdometryDataSize;
constexpr uint8_t kPositionXyBits = 17;
constexpr uint8_t kPositionZBits = 14;
constexpr uint8_t kAngleBits = 15;
constexpr uint8_t kLinearVelocityBits = 13;
constexpr uint8_t kAngularVelocityBits = 12;
constexpr double kPositionScale = 50.0; // 50 = 2 cm precision, range xy +-655 m, z +-164 m
constexpr double kLinearVelocityScale = 50.0; // 50 = 2 cm/s precision, range +-82 m/s
constexpr double kAngularVelocityScale = 200.0; // 200 = 0.005 rad/s precision, range +-10.24 rad/s
constexpr std::size_t kPackedOdometryBits =
    2 * kPositionXyBits + kPositionZBits +
    3 * kAngleBits +
    3 * kLinearVelocityBits +
    3 * kAngularVelocityBits;
static_assert(kPackedOdometryBits == kPackedOdometryDataSize * 8,
              "Packed advertisement odometry data must be exactly 21 bytes");
static_assert(kFixedOdometryPayloadSize == 28,
              "Fixed advertisement odometry payload must be exactly 28 bytes");

struct Quaternion {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double w{1.0};
};

struct RollPitchYaw {
    double roll{0.0};
    double pitch{0.0};
    double yaw{0.0};
};

RollPitchYaw quaternion_to_rpy(double x, double y, double z, double w) {
    const double norm = std::sqrt(x * x + y * y + z * z + w * w);
    if (norm > 0.0) {
        x /= norm;
        y /= norm;
        z /= norm;
        w /= norm;
    } else {
        x = 0.0;
        y = 0.0;
        z = 0.0;
        w = 1.0;
    }

    RollPitchYaw rpy;
    const double sin_roll_cos_pitch = 2.0 * (w * x + y * z);
    const double cos_roll_cos_pitch = 1.0 - 2.0 * (x * x + y * y);
    rpy.roll = std::atan2(sin_roll_cos_pitch, cos_roll_cos_pitch);

    const double sin_pitch = 2.0 * (w * y - z * x);
    if (std::abs(sin_pitch) >= 1.0) {
        rpy.pitch = std::copysign(kPi / 2.0, sin_pitch);
    } else {
        rpy.pitch = std::asin(sin_pitch);
    }

    const double sin_yaw_cos_pitch = 2.0 * (w * z + x * y);
    const double cos_yaw_cos_pitch = 1.0 - 2.0 * (y * y + z * z);
    rpy.yaw = std::atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch);
    return rpy;
}

Quaternion rpy_to_quaternion(double roll, double pitch, double yaw) {
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return q;
}

double normalize_angle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

int64_t signed_min(uint8_t bit_width) {
    return -(int64_t{1} << (bit_width - 1));
}

int64_t signed_max(uint8_t bit_width) {
    return (int64_t{1} << (bit_width - 1)) - 1;
}

int64_t encode_angle(double angle, uint8_t bit_width) {
    const double normalized = normalize_angle(angle);
    const auto max_value = signed_max(bit_width);
    const double scaled = std::round((normalized / kPi) * static_cast<double>(max_value));
    return static_cast<int64_t>(std::clamp(scaled,
                                           -static_cast<double>(max_value),
                                           static_cast<double>(max_value)));
}

double decode_angle(int64_t value, uint8_t bit_width) {
    return (static_cast<double>(value) / static_cast<double>(signed_max(bit_width))) * kPi;
}

int64_t encode_fixed(double value, double scale, uint8_t bit_width) {
    const double scaled = std::round(value * scale);
    return static_cast<int64_t>(std::clamp(scaled,
                                           static_cast<double>(signed_min(bit_width)),
                                           static_cast<double>(signed_max(bit_width))));
}

double decode_fixed(int64_t value, double scale) {
    return static_cast<double>(value) / scale;
}

void append_signed_bits(std::vector<uint8_t>& data,
                        int64_t value,
                        uint8_t bit_width,
                        std::size_t& bit_offset) {
    const uint64_t mask = (uint64_t{1} << bit_width) - 1;
    const uint64_t raw = static_cast<uint64_t>(value) & mask;
    for (uint8_t bit = 0; bit < bit_width; ++bit) {
        if (bit_offset % 8 == 0) {
            data.push_back(0);
        }
        if (((raw >> bit) & 0x1u) != 0) {
            data.back() |= static_cast<uint8_t>(1u << (bit_offset % 8));
        }
        ++bit_offset;
    }
}

uint64_t read_unsigned_bits(const std::vector<uint8_t>& data,
                            std::size_t byte_offset,
                            std::size_t bit_offset,
                            uint8_t bit_width) {
    uint64_t raw = 0;
    for (uint8_t bit = 0; bit < bit_width; ++bit) {
        const std::size_t absolute_bit = byte_offset * 8 + bit_offset + bit;
        const std::size_t byte_index = absolute_bit / 8;
        const std::size_t bit_index = absolute_bit % 8;
        if (((data[byte_index] >> bit_index) & 0x1u) != 0) {
            raw |= uint64_t{1} << bit;
        }
    }
    return raw;
}

int64_t read_signed_bits(const std::vector<uint8_t>& data,
                         std::size_t byte_offset,
                         std::size_t& bit_offset,
                         uint8_t bit_width) {
    const uint64_t raw = read_unsigned_bits(data, byte_offset, bit_offset, bit_width);
    bit_offset += bit_width;
    const uint64_t sign_bit = uint64_t{1} << (bit_width - 1);
    if ((raw & sign_bit) == 0) {
        return static_cast<int64_t>(raw);
    }
    return static_cast<int64_t>(raw) - static_cast<int64_t>(uint64_t{1} << bit_width);
}

}  // namespace

namespace mrs_uav_bluetooth::app {

TestNode::TestNode()
    : rclcpp::Node("mrs_uav_bluetooth_test"),
      random_engine_(std::random_device{}()) {
    configure_parameters();
    configure_publishers();
    configure_advertisement_watchers();
    configure_advertisement_odometry_subscription();

    publish_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_hz_),
        [this]() { publish_once(); });
}

void TestNode::configure_parameters() {
    declare_parameter<std::string>("mode", "odometry");
    declare_parameter<double>("rate_hz", 1.0);
    declare_parameter<std::string>("odometry_topic", "");
    declare_parameter<std::string>("advertisement_topic", "");
    declare_parameter<std::string>("advertisement_observe_topic", "");
    declare_parameter<std::string>("peer_topic_prefix", "");
    declare_parameter<std::string>("frame_id", frame_id_);
    declare_parameter<std::string>("child_frame_id", child_frame_id_);
    declare_parameter<double>("odometry_timeout_sec", odometry_timeout_sec_);

    const auto requested_mode = normalize_mode(get_parameter("mode").as_string());
    rate_hz_ = get_parameter("rate_hz").as_double();
    frame_id_ = get_parameter("frame_id").as_string();
    child_frame_id_ = get_parameter("child_frame_id").as_string();
    odometry_timeout_sec_ = get_parameter("odometry_timeout_sec").as_double();

    if (requested_mode == "odometry") {
        mode_ = Mode::kOdometry;
    } else if (requested_mode == "advertisement") {
        mode_ = Mode::kAdvertisement;
    } else {
        throw std::runtime_error("mode must be 'odometry' or 'advertisement'");
    }

    if (!std::isfinite(rate_hz_) || rate_hz_ <= 0.0) {
        throw std::runtime_error("rate_hz must be a finite value greater than 0");
    }
    if (!std::isfinite(odometry_timeout_sec_) || odometry_timeout_sec_ <= 0.0) {
        throw std::runtime_error("odometry_timeout_sec must be a finite value greater than 0");
    }

    const auto raw_hostname = util::system_hostname();
    hostname_ = util::sanitize_topic_suffix(raw_hostname.empty() ? "mrs-uav" : raw_hostname);

    auto configured_odometry_topic = get_parameter("odometry_topic").as_string();
    if (configured_odometry_topic.empty()) {
        configured_odometry_topic = kDefaultOdometryTopic;
    }
    odometry_topic_ = util::normalize_ros_topic(expand_hostname(configured_odometry_topic, hostname_));

    auto configured_advertisement_topic = get_parameter("advertisement_topic").as_string();
    if (configured_advertisement_topic.empty()) {
        configured_advertisement_topic = kDefaultAdvertisementTopic;
    }
    advertisement_topic_ = util::normalize_ros_topic(
        expand_hostname(configured_advertisement_topic, hostname_));

    auto configured_observe_topic = get_parameter("advertisement_observe_topic").as_string();
    if (configured_observe_topic.empty()) {
        configured_observe_topic = kDefaultAdvertisementObserveTopic;
    }
    advertisement_observe_topic_ = util::normalize_ros_topic(
        expand_hostname(configured_observe_topic, hostname_));

    auto configured_peer_topic_prefix = get_parameter("peer_topic_prefix").as_string();
    if (configured_peer_topic_prefix.empty()) {
        configured_peer_topic_prefix = kDefaultPeerTopicPrefix;
    }
    peer_topic_prefix_ = util::normalize_ros_topic(
        expand_hostname(configured_peer_topic_prefix, hostname_));
}

void TestNode::configure_publishers() {
    if (mode_ == Mode::kOdometry) {
        odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, 10);
        RCLCPP_INFO(get_logger(),
                    "Publishing dummy odometry at %.3f Hz on %s",
                    rate_hz_, odometry_topic_.c_str());
        return;
    }

    advertisement_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>(advertisement_topic_, 10);
    RCLCPP_INFO(get_logger(),
                "Publishing packed advertisement odometry at %.3f Hz on %s; subscribing to local odometry on %s; payload=%zu bytes; timestamp resolution=%" PRIu64 " ns; odometry bits=%zu; peer odometry prefix %s",
                rate_hz_, advertisement_topic_.c_str(), odometry_topic_.c_str(),
                kFixedOdometryPayloadSize, kTimestampResolutionNs, kPackedOdometryBits,
                peer_topic_prefix_.c_str());
}

void TestNode::configure_advertisement_watchers() {
    if (mode_ != Mode::kAdvertisement) {
        return;
    }

    const auto callback = [this](const mrs_uav_bluetooth::msg::BleDeviceArray::SharedPtr message) {
        handle_advertisement_scan(message);
    };

    advertisement_scan_sub_ = create_subscription<mrs_uav_bluetooth::msg::BleDeviceArray>(
        advertisement_observe_topic_, rclcpp::QoS(10), callback);
    RCLCPP_INFO(get_logger(), "Monitoring advertisement device topic: %s",
                advertisement_observe_topic_.c_str());
}

void TestNode::configure_advertisement_odometry_subscription() {
    if (mode_ != Mode::kAdvertisement) {
        return;
    }

    advertisement_odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic_,
        rclcpp::QoS(10),
        [this](const nav_msgs::msg::Odometry::SharedPtr message) {
            handle_local_odometry(message);
        });
    RCLCPP_INFO(get_logger(),
                "Advertisement mode will encode local odometry from %s when available",
                odometry_topic_.c_str());
}

void TestNode::publish_once() {
    if (mode_ == Mode::kOdometry) {
        publish_odometry();
        return;
    }

    publish_advertisement_payload();
}

void TestNode::publish_odometry() {
    if (!odometry_pub_) {
        return;
    }

    nav_msgs::msg::Odometry message;
    message.header.stamp = get_clock()->now();
    message.header.frame_id = frame_id_;
    message.child_frame_id = child_frame_id_;

    message.pose.pose.position.x = sample_uniform(-50.0, 50.0);
    message.pose.pose.position.y = sample_uniform(-50.0, 50.0);
    message.pose.pose.position.z = sample_uniform(0.0, 20.0);

    const double yaw = sample_uniform(-kPi, kPi);
    message.pose.pose.orientation.z = std::sin(yaw / 2.0);
    message.pose.pose.orientation.w = std::cos(yaw / 2.0);

    message.twist.twist.linear.x = sample_uniform(-5.0, 5.0);
    message.twist.twist.linear.y = sample_uniform(-5.0, 5.0);
    message.twist.twist.linear.z = sample_uniform(-2.0, 2.0);

    message.twist.twist.angular.x = sample_uniform(-0.5, 0.5);
    message.twist.twist.angular.y = sample_uniform(-0.5, 0.5);
    message.twist.twist.angular.z = sample_uniform(-1.5, 1.5);

    odometry_pub_->publish(message);
}

void TestNode::publish_advertisement_payload() {
    if (!advertisement_pub_) {
        return;
    }

    std_msgs::msg::UInt8MultiArray message;
    const auto odometry = fresh_local_odometry();

    if (odometry.has_value()) {
        const auto timestamp_ns = stamp_to_nanoseconds(odometry->header.stamp);
        const auto rpy = quaternion_to_rpy(odometry->pose.pose.orientation.x,
                                           odometry->pose.pose.orientation.y,
                                           odometry->pose.pose.orientation.z,
                                           odometry->pose.pose.orientation.w);
        message.data.reserve(kFixedOdometryPayloadSize);
        append_microsecond_timestamp(message.data, timestamp_ns);

        std::size_t bit_offset = 0;
        append_signed_bits(message.data,
                           encode_fixed(odometry->pose.pose.position.x, kPositionScale, kPositionXyBits),
                           kPositionXyBits, bit_offset);
        append_signed_bits(message.data,
                           encode_fixed(odometry->pose.pose.position.y, kPositionScale, kPositionXyBits),
                           kPositionXyBits, bit_offset);
        append_signed_bits(message.data,
                           encode_fixed(odometry->pose.pose.position.z, kPositionScale, kPositionZBits),
                           kPositionZBits, bit_offset);
        append_signed_bits(message.data, encode_angle(rpy.roll, kAngleBits), kAngleBits, bit_offset);
        append_signed_bits(message.data, encode_angle(rpy.pitch, kAngleBits), kAngleBits, bit_offset);
        append_signed_bits(message.data, encode_angle(rpy.yaw, kAngleBits), kAngleBits, bit_offset);
        append_signed_bits(message.data,
                           encode_fixed(odometry->twist.twist.linear.x, kLinearVelocityScale, kLinearVelocityBits),
                           kLinearVelocityBits, bit_offset);
        append_signed_bits(message.data,
                           encode_fixed(odometry->twist.twist.linear.y, kLinearVelocityScale, kLinearVelocityBits),
                           kLinearVelocityBits, bit_offset);
        append_signed_bits(message.data,
                           encode_fixed(odometry->twist.twist.linear.z, kLinearVelocityScale, kLinearVelocityBits),
                           kLinearVelocityBits, bit_offset);
        append_signed_bits(message.data,
                           encode_fixed(odometry->twist.twist.angular.x, kAngularVelocityScale, kAngularVelocityBits),
                           kAngularVelocityBits, bit_offset);
        append_signed_bits(message.data,
                           encode_fixed(odometry->twist.twist.angular.y, kAngularVelocityScale, kAngularVelocityBits),
                           kAngularVelocityBits, bit_offset);
        append_signed_bits(message.data,
                           encode_fixed(odometry->twist.twist.angular.z, kAngularVelocityScale, kAngularVelocityBits),
                           kAngularVelocityBits, bit_offset);

        RCLCPP_INFO(get_logger(),
                    "Published advertisement odometry bytes=%zu stamp=%" PRIu64 " pos=(%.3f, %.3f, %.3f)m rpy=(%.3f, %.3f, %.3f)° lin=(%.3f, %.3f, %.3f)m/s ang=(%.3f, %.3f, %.3f)°/s",
                    message.data.size(),
                    timestamp_ns,
                    odometry->pose.pose.position.x,
                    odometry->pose.pose.position.y,
                    odometry->pose.pose.position.z,
                    rpy.roll * 180.0 / kPi,
                    rpy.pitch * 180.0 / kPi,
                    rpy.yaw * 180.0 / kPi,
                    odometry->twist.twist.linear.x,
                    odometry->twist.twist.linear.y,
                    odometry->twist.twist.linear.z,
                    odometry->twist.twist.angular.x * 180.0 / kPi,
                    odometry->twist.twist.angular.y * 180.0 / kPi,
                    odometry->twist.twist.angular.z * 180.0 / kPi);
    } else {
        const auto timestamp_ns = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::system_clock::now().time_since_epoch())
                .count());
        append_microsecond_timestamp(message.data, timestamp_ns);
        RCLCPP_INFO(get_logger(),
                    "Published advertisement timestamp keepalive bytes=%zu stamp=%" PRIu64,
                    message.data.size(),
                    timestamp_ns);
    }

    advertisement_pub_->publish(message);
}

void TestNode::handle_local_odometry(const nav_msgs::msg::Odometry::SharedPtr message) {
    if (!message) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(local_odometry_mutex_);
        latest_local_odometry_ = *message;
        latest_local_odometry_received_ = std::chrono::steady_clock::now();
    }

    /*const auto timestamp_ns = stamp_to_nanoseconds(message->header.stamp);
    RCLCPP_INFO(get_logger(),
                "Received local odometry for advertisement stamp=%" PRIu64 " pos=(%.3f, %.3f, %.3f) q=(%.3f, %.3f, %.3f, %.3f) lin=(%.3f, %.3f, %.3f) ang=(%.3f, %.3f, %.3f)",
                timestamp_ns,
                message->pose.pose.position.x,
                message->pose.pose.position.y,
                message->pose.pose.position.z,
                message->pose.pose.orientation.x,
                message->pose.pose.orientation.y,
                message->pose.pose.orientation.z,
                message->pose.pose.orientation.w,
                message->twist.twist.linear.x,
                message->twist.twist.linear.y,
                message->twist.twist.linear.z,
                message->twist.twist.angular.x,
                message->twist.twist.angular.y,
                message->twist.twist.angular.z);*/
}

std::optional<nav_msgs::msg::Odometry> TestNode::fresh_local_odometry() {
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(local_odometry_mutex_);
    if (!latest_local_odometry_ || !latest_local_odometry_received_) {
        return std::nullopt;
    }

    const auto age_sec =
        std::chrono::duration<double>(now - *latest_local_odometry_received_).count();
    if (age_sec <= odometry_timeout_sec_) {
        return latest_local_odometry_;
    }

    const auto stale_stamp_ns = stamp_to_nanoseconds(latest_local_odometry_->header.stamp);
    RCLCPP_INFO(get_logger(),
                "Local odometry stale for advertisement age=%.3f s timeout=%.3f s last_stamp=%" PRIu64 "; reverting to timestamp keepalive",
                age_sec, odometry_timeout_sec_, stale_stamp_ns);
    latest_local_odometry_.reset();
    latest_local_odometry_received_.reset();
    return std::nullopt;
}

void TestNode::handle_advertisement_scan(const mrs_uav_bluetooth::msg::BleDeviceArray::SharedPtr message) {
    if (!message) {
        return;
    }

    const auto local_time_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());

    bool saw_matching_data = false;
    std::lock_guard<std::mutex> lock(observed_devices_mutex_);
    for (const auto& device : message->devices) {
        if (device.advertising_data.empty()) {
            continue;
        }

        saw_matching_data = true;
        const auto key = device.mac;
        auto last_it = last_logged_advertisements_.find(key);
        if (last_it != last_logged_advertisements_.end() &&
            last_it->second == device.advertising_data) {
            continue;
        }
        last_logged_advertisements_[key] = device.advertising_data;
        log_decoded_advertisement(device, device.advertising_data, local_time_ns);
    }

    if (!saw_matching_data) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                             "No devices with user advertisement payload observed on %s",
                             advertisement_observe_topic_.c_str());
    }
}

void TestNode::log_decoded_advertisement(const mrs_uav_bluetooth::msg::BleDevice& device,
                                         const std::vector<uint8_t>& data,
                                         uint64_t local_time_ns) {
    if (data.size() != kTimestampPayloadSize && data.size() != kFixedOdometryPayloadSize) {
        RCLCPP_INFO(get_logger(),
                    "Advertisement data name='%s' mac=%s bytes=%zu unsupported packed payload size",
                    device.name.c_str(), device.mac.c_str(), data.size());
        return;
    }

    const auto decoded = decode_microsecond_timestamp_ns(data);
    uint64_t odometry_stamp_ns = 0;
    std::vector<float> odometry_values;
    const bool has_odometry = decode_fixed_odometry_payload(data, odometry_stamp_ns, odometry_values);
    std::ostringstream stream;
    stream << "Advertisement data"
           << " name='" << device.name << "'"
           << " mac=" << device.mac
           << " bytes=" << data.size();

    if (!decoded.has_value()) {
        stream << " uint64=-";
        RCLCPP_INFO(get_logger(), "%s", stream.str().c_str());
        return;
    }

    const long double remote_minus_local_ms =
        (static_cast<long double>(*decoded) - static_cast<long double>(local_time_ns)) / 1000000.0L;
    stream << " uint64=" << *decoded
           << " time=" << format_system_time(*decoded)
           << " remote-local-ms=" << std::fixed << std::setprecision(3)
           << static_cast<double>(remote_minus_local_ms);
    if (has_odometry) {
        stream << " odom pos=(" << odometry_values[0] << ", " << odometry_values[1] << ", " << odometry_values[2] << ")m"
               << " rpy=(" << (odometry_values[3] * 180.0 / kPi) << ", " << (odometry_values[4] * 180.0 / kPi) << ", " << (odometry_values[5] * 180.0 / kPi) << ")°"
               << " lin=(" << odometry_values[6] << ", " << odometry_values[7] << ", " << odometry_values[8] << ")m/s"
               << " ang=(" << (odometry_values[9] * 180.0 / kPi) << ", " << (odometry_values[10] * 180.0 / kPi) << ", " << (odometry_values[11] * 180.0 / kPi) << ")°/s";
    } else {
        stream << " keepalive";
    }
    RCLCPP_INFO(get_logger(), "%s", stream.str().c_str());

    if (has_odometry) {
        publish_decoded_peer_odometry(device, odometry_stamp_ns, odometry_values);
    }
}

std::optional<uint64_t> TestNode::decode_microsecond_timestamp_ns(const std::vector<uint8_t>& data) {
    if (data.size() < kTimestampPayloadSize) {
        return std::nullopt;
    }

    uint64_t timestamp_us = 0;
    for (std::size_t index = 0; index < kTimestampPayloadSize; ++index) {
        timestamp_us |= static_cast<uint64_t>(data[index]) << (8 * index);
    }
    return timestamp_us * 1000ULL;
}

bool TestNode::decode_fixed_odometry_payload(const std::vector<uint8_t>& data,
                                             uint64_t& stamp_ns,
                                             std::vector<float>& values) {
    if (data.size() != kFixedOdometryPayloadSize) {
        return false;
    }

    const auto decoded_stamp = decode_microsecond_timestamp_ns(data);
    if (!decoded_stamp.has_value()) {
        return false;
    }

    stamp_ns = *decoded_stamp;
    values.clear();
    values.reserve(kFixedFieldCount);
    std::size_t bit_offset = 0;
    values.push_back(static_cast<float>(
        decode_fixed(read_signed_bits(data, kTimestampPayloadSize, bit_offset, kPositionXyBits), kPositionScale)));
    values.push_back(static_cast<float>(
        decode_fixed(read_signed_bits(data, kTimestampPayloadSize, bit_offset, kPositionXyBits), kPositionScale)));
    values.push_back(static_cast<float>(
        decode_fixed(read_signed_bits(data, kTimestampPayloadSize, bit_offset, kPositionZBits), kPositionScale)));
    values.push_back(static_cast<float>(
        decode_angle(read_signed_bits(data, kTimestampPayloadSize, bit_offset, kAngleBits), kAngleBits)));
    values.push_back(static_cast<float>(
        decode_angle(read_signed_bits(data, kTimestampPayloadSize, bit_offset, kAngleBits), kAngleBits)));
    values.push_back(static_cast<float>(
        decode_angle(read_signed_bits(data, kTimestampPayloadSize, bit_offset, kAngleBits), kAngleBits)));
    values.push_back(static_cast<float>(
        decode_fixed(read_signed_bits(data, kTimestampPayloadSize, bit_offset, kLinearVelocityBits),
                     kLinearVelocityScale)));
    values.push_back(static_cast<float>(
        decode_fixed(read_signed_bits(data, kTimestampPayloadSize, bit_offset, kLinearVelocityBits),
                     kLinearVelocityScale)));
    values.push_back(static_cast<float>(
        decode_fixed(read_signed_bits(data, kTimestampPayloadSize, bit_offset, kLinearVelocityBits),
                     kLinearVelocityScale)));
    values.push_back(static_cast<float>(
        decode_fixed(read_signed_bits(data, kTimestampPayloadSize, bit_offset, kAngularVelocityBits),
                     kAngularVelocityScale)));
    values.push_back(static_cast<float>(
        decode_fixed(read_signed_bits(data, kTimestampPayloadSize, bit_offset, kAngularVelocityBits),
                     kAngularVelocityScale)));
    values.push_back(static_cast<float>(
        decode_fixed(read_signed_bits(data, kTimestampPayloadSize, bit_offset, kAngularVelocityBits),
                     kAngularVelocityScale)));
    return true;
}

void TestNode::append_microsecond_timestamp(std::vector<uint8_t>& data, uint64_t timestamp_ns) {
    const uint64_t timestamp_us = timestamp_ns / 1000ULL;
    for (std::size_t index = 0; index < kTimestampPayloadSize; ++index) {
        data.push_back(static_cast<uint8_t>((timestamp_us >> (8 * index)) & 0xFFu));
    }
}

uint64_t TestNode::stamp_to_nanoseconds(const builtin_interfaces::msg::Time& stamp) {
    return static_cast<uint64_t>(stamp.sec) * 1000000000ULL + static_cast<uint64_t>(stamp.nanosec);
}

builtin_interfaces::msg::Time TestNode::nanoseconds_to_stamp(uint64_t stamp_ns) {
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000ULL);
    stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000ULL);
    return stamp;
}

std::string TestNode::format_system_time(uint64_t timestamp_ns) {
    if (timestamp_ns == 0) {
        return "-";
    }

    const auto seconds = static_cast<std::time_t>(timestamp_ns / 1000000000ULL);
    const auto milliseconds = (timestamp_ns % 1000000000ULL) / 1000000ULL;
    std::tm tm{};
    localtime_r(&seconds, &tm);
    std::ostringstream out;
    out << std::put_time(&tm, "%F %T")
        << '.' << std::setw(3) << std::setfill('0') << milliseconds;
    return out.str();
}

void TestNode::publish_decoded_peer_odometry(const mrs_uav_bluetooth::msg::BleDevice& device,
                                             uint64_t stamp_ns,
                                             const std::vector<float>& values) {
    if (values.size() < kFixedFieldCount) {
        return;
    }

    auto publisher = peer_odometry_publisher(device);

    nav_msgs::msg::Odometry message;
    message.header.stamp = nanoseconds_to_stamp(stamp_ns);
    message.header.frame_id = frame_id_;
    message.child_frame_id = child_frame_id_;
    message.pose.pose.position.x = values[0];
    message.pose.pose.position.y = values[1];
    message.pose.pose.position.z = values[2];
    const auto quaternion = rpy_to_quaternion(values[3], values[4], values[5]);
    message.pose.pose.orientation.x = quaternion.x;
    message.pose.pose.orientation.y = quaternion.y;
    message.pose.pose.orientation.z = quaternion.z;
    message.pose.pose.orientation.w = quaternion.w;
    message.twist.twist.linear.x = values[6];
    message.twist.twist.linear.y = values[7];
    message.twist.twist.linear.z = values[8];
    message.twist.twist.angular.x = values[9];
    message.twist.twist.angular.y = values[10];
    message.twist.twist.angular.z = values[11];

    publisher->publish(message);
    RCLCPP_INFO(get_logger(),
                "Published decoded peer odometry for %s stamp=%" PRIu64 " pos=(%.3f, %.3f, %.3f)m rpy=(%.3f, %.3f, %.3f)° lin=(%.3f, %.3f, %.3f)m/s ang=(%.3f, %.3f, %.3f)°/s",
                device_topic_token(device).c_str(),
                stamp_ns,
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                message.pose.pose.position.z,
                values[3] * 180.0 / kPi,
                values[4] * 180.0 / kPi,
                values[5] * 180.0 / kPi,
                message.twist.twist.linear.x,
                message.twist.twist.linear.y,
                message.twist.twist.linear.z,
                message.twist.twist.angular.x * 180.0 / kPi,
                message.twist.twist.angular.y * 180.0 / kPi,
                message.twist.twist.angular.z * 180.0 / kPi);
}

rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr TestNode::peer_odometry_publisher(
    const mrs_uav_bluetooth::msg::BleDevice& device) {
    const auto token = device_topic_token(device);
    auto existing = peer_odometry_publishers_.find(token);
    if (existing != peer_odometry_publishers_.end()) {
        return existing->second;
    }

    const auto topic = util::normalize_ros_topic(peer_topic_prefix_ + "/" + token + "/odom");
    auto publisher = create_publisher<nav_msgs::msg::Odometry>(topic, 10);
    peer_odometry_publishers_[token] = publisher;
    RCLCPP_INFO(get_logger(),
                "Created peer odometry publisher for name='%s' mac=%s on %s",
                device.name.c_str(),
                device.mac.c_str(),
                topic.c_str());
    return publisher;
}

std::string TestNode::normalize_mode(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
    return value;
}

std::string TestNode::expand_hostname(std::string value, const std::string& hostname) {
    constexpr std::string_view token = "{hostname}";
    std::size_t position = 0;
    while ((position = value.find(token, position)) != std::string::npos) {
        value.replace(position, token.size(), hostname);
        position += hostname.size();
    }
    return value;
}

std::string TestNode::device_topic_token(const mrs_uav_bluetooth::msg::BleDevice& device) {
    std::string token = device.hostname;
    if (token.empty()) {
        token = device.name;
    }
    if (token.empty()) {
        token = device.alias;
    }
    if (token.empty()) {
        token = "peer_" + device.mac;
    }
    return util::sanitize_topic_suffix(token);
}

double TestNode::sample_uniform(double min, double max) {
    std::uniform_real_distribution<double> distribution(min, max);
    return distribution(random_engine_);
}

}  // namespace mrs_uav_bluetooth::app
