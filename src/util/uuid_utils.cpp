// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/util/uuid_utils.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <regex>
#include <sstream>
#include <string>

namespace mrs_uav_bluetooth::util {

namespace {

// Minimal MD5 implementation (RFC 1321).
struct MD5 {
    uint32_t state[4]{0x67452301, 0xefcdab89, 0x98badcfe, 0x10325476};
    uint64_t count{0};
    uint8_t buffer[64]{};

    static constexpr uint32_t S[64] = {
         7, 12, 17, 22,  7, 12, 17, 22,  7, 12, 17, 22,  7, 12, 17, 22,
         5,  9, 14, 20,  5,  9, 14, 20,  5,  9, 14, 20,  5,  9, 14, 20,
         4, 11, 16, 23,  4, 11, 16, 23,  4, 11, 16, 23,  4, 11, 16, 23,
         6, 10, 15, 21,  6, 10, 15, 21,  6, 10, 15, 21,  6, 10, 15, 21,
    };
    static constexpr uint32_t K[64] = {
        0xd76aa478, 0xe8c7b756, 0x242070db, 0xc1bdceee,
        0xf57c0faf, 0x4787c62a, 0xa8304613, 0xfd469501,
        0x698098d8, 0x8b44f7af, 0xffff5bb1, 0x895cd7be,
        0x6b901122, 0xfd987193, 0xa679438e, 0x49b40821,
        0xf61e2562, 0xc040b340, 0x265e5a51, 0xe9b6c7aa,
        0xd62f105d, 0x02441453, 0xd8a1e681, 0xe7d3fbc8,
        0x21e1cde6, 0xc33707d6, 0xf4d50d87, 0x455a14ed,
        0xa9e3e905, 0xfcefa3f8, 0x676f02d9, 0x8d2a4c8a,
        0xfffa3942, 0x8771f681, 0x6d9d6122, 0xfde5380c,
        0xa4beea44, 0x4bdecfa9, 0xf6bb4b60, 0xbebfbc70,
        0x289b7ec6, 0xeaa127fa, 0xd4ef3085, 0x04881d05,
        0xd9d4d039, 0xe6db99e5, 0x1fa27cf8, 0xc4ac5665,
        0xf4292244, 0x432aff97, 0xab9423a7, 0xfc93a039,
        0x655b59c3, 0x8f0ccc92, 0xffeff47d, 0x85845dd1,
        0x6fa87e4f, 0xfe2ce6e0, 0xa3014314, 0x4e0811a1,
        0xf7537e82, 0xbd3af235, 0x2ad7d2bb, 0xeb86d391,
    };

    static uint32_t left_rotate(uint32_t x, uint32_t c) {
        return (x << c) | (x >> (32 - c));
    }

    void transform(const uint8_t block[64]) {
        uint32_t M[16];
        for (int i = 0; i < 16; ++i) {
            std::memcpy(&M[i], block + i * 4, 4);
        }
        uint32_t a = state[0], b = state[1], c = state[2], d = state[3];
        for (int i = 0; i < 64; ++i) {
            uint32_t f, g;
            if (i < 16) {
                f = (b & c) | (~b & d);
                g = static_cast<uint32_t>(i);
            } else if (i < 32) {
                f = (d & b) | (~d & c);
                g = (5u * i + 1u) % 16u;
            } else if (i < 48) {
                f = b ^ c ^ d;
                g = (3u * i + 5u) % 16u;
            } else {
                f = c ^ (b | ~d);
                g = (7u * i) % 16u;
            }
            f = f + a + K[i] + M[g];
            a = d;
            d = c;
            c = b;
            b = b + left_rotate(f, S[i]);
        }
        state[0] += a;
        state[1] += b;
        state[2] += c;
        state[3] += d;
    }

    void update(const uint8_t* data, size_t len) {
        size_t index = static_cast<size_t>(count % 64);
        count += len;
        size_t i = 0;
        if (index) {
            size_t space = 64 - index;
            size_t part = std::min(len, space);
            std::memcpy(buffer + index, data, part);
            i += part;
            if (index + part < 64) return;
            transform(buffer);
        }
        for (; i + 64 <= len; i += 64) {
            transform(data + i);
        }
        if (i < len) {
            std::memcpy(buffer, data + i, len - i);
        }
    }

    std::array<uint8_t, 16> finalize() {
        uint8_t pad[64]{};
        size_t index = static_cast<size_t>(count % 64);
        pad[0] = 0x80;
        size_t pad_len = (index < 56) ? (56 - index) : (120 - index);
        update(pad, pad_len);
        uint64_t bits = count * 8 - pad_len * 8;
        // Actually we need original count.  We already added pad_len, so:
        // Recompute.
        // Easier approach: just encode bit-count before padding.
        // Let's redo with a cleaner pattern.
        (void)bits;
        // We already consumed the padding; the state is updated.
        // The last 8 bytes should have been the bit count.
        // This simplified MD5 works for our purpose but let's use the
        // standard finalization approach.
        std::array<uint8_t, 16> digest{};
        for (int i = 0; i < 4; ++i) {
            std::memcpy(digest.data() + i * 4, &state[i], 4);
        }
        return digest;
    }
};

}  // namespace

// Use OpenSSL-style or a well-known small MD5.
// For correctness and simplicity, use a clean implementation:

namespace {

struct MD5Context {
    uint32_t state[4];
    uint64_t count;
    uint8_t buffer[64];
};

void md5_init(MD5Context& ctx) {
    ctx.state[0] = 0x67452301;
    ctx.state[1] = 0xefcdab89;
    ctx.state[2] = 0x98badcfe;
    ctx.state[3] = 0x10325476;
    ctx.count = 0;
    std::memset(ctx.buffer, 0, 64);
}

constexpr uint32_t md5_F(uint32_t x, uint32_t y, uint32_t z) { return (x & y) | (~x & z); }
constexpr uint32_t md5_G(uint32_t x, uint32_t y, uint32_t z) { return (x & z) | (y & ~z); }
constexpr uint32_t md5_H(uint32_t x, uint32_t y, uint32_t z) { return x ^ y ^ z; }
constexpr uint32_t md5_I(uint32_t x, uint32_t y, uint32_t z) { return y ^ (x | ~z); }
constexpr uint32_t md5_rotl(uint32_t x, uint32_t n) { return (x << n) | (x >> (32 - n)); }

void md5_transform(uint32_t state[4], const uint8_t block[64]) {
    static constexpr uint32_t S[64] = {
         7,12,17,22, 7,12,17,22, 7,12,17,22, 7,12,17,22,
         5, 9,14,20, 5, 9,14,20, 5, 9,14,20, 5, 9,14,20,
         4,11,16,23, 4,11,16,23, 4,11,16,23, 4,11,16,23,
         6,10,15,21, 6,10,15,21, 6,10,15,21, 6,10,15,21,
    };
    static constexpr uint32_t K[64] = {
        0xd76aa478,0xe8c7b756,0x242070db,0xc1bdceee,
        0xf57c0faf,0x4787c62a,0xa8304613,0xfd469501,
        0x698098d8,0x8b44f7af,0xffff5bb1,0x895cd7be,
        0x6b901122,0xfd987193,0xa679438e,0x49b40821,
        0xf61e2562,0xc040b340,0x265e5a51,0xe9b6c7aa,
        0xd62f105d,0x02441453,0xd8a1e681,0xe7d3fbc8,
        0x21e1cde6,0xc33707d6,0xf4d50d87,0x455a14ed,
        0xa9e3e905,0xfcefa3f8,0x676f02d9,0x8d2a4c8a,
        0xfffa3942,0x8771f681,0x6d9d6122,0xfde5380c,
        0xa4beea44,0x4bdecfa9,0xf6bb4b60,0xbebfbc70,
        0x289b7ec6,0xeaa127fa,0xd4ef3085,0x04881d05,
        0xd9d4d039,0xe6db99e5,0x1fa27cf8,0xc4ac5665,
        0xf4292244,0x432aff97,0xab9423a7,0xfc93a039,
        0x655b59c3,0x8f0ccc92,0xffeff47d,0x85845dd1,
        0x6fa87e4f,0xfe2ce6e0,0xa3014314,0x4e0811a1,
        0xf7537e82,0xbd3af235,0x2ad7d2bb,0xeb86d391,
    };

    uint32_t M[16];
    for (int i = 0; i < 16; ++i) {
        std::memcpy(&M[i], block + i * 4, 4);
    }

    uint32_t a = state[0], b = state[1], c = state[2], d = state[3];
    for (int i = 0; i < 64; ++i) {
        uint32_t f, g;
        if (i < 16) {
            f = md5_F(b, c, d);
            g = static_cast<uint32_t>(i);
        } else if (i < 32) {
            f = md5_G(b, c, d);
            g = (5u * i + 1u) % 16u;
        } else if (i < 48) {
            f = md5_H(b, c, d);
            g = (3u * i + 5u) % 16u;
        } else {
            f = md5_I(b, c, d);
            g = (7u * i) % 16u;
        }
        uint32_t temp = d;
        d = c;
        c = b;
        b = b + md5_rotl(a + f + K[i] + M[g], S[i]);
        a = temp;
    }
    state[0] += a;
    state[1] += b;
    state[2] += c;
    state[3] += d;
}

void md5_update(MD5Context& ctx, const uint8_t* data, size_t len) {
    size_t index = static_cast<size_t>(ctx.count % 64);
    ctx.count += len;

    size_t i = 0;
    if (index) {
        size_t space = 64 - index;
        size_t part = std::min(len, space);
        std::memcpy(ctx.buffer + index, data, part);
        i = part;
        if (index + part < 64) return;
        md5_transform(ctx.state, ctx.buffer);
    }
    for (; i + 64 <= len; i += 64) {
        md5_transform(ctx.state, data + i);
    }
    if (i < len) {
        std::memcpy(ctx.buffer, data + i, len - i);
    }
}

std::array<uint8_t, 16> md5_finalize(MD5Context& ctx) {
    uint64_t bit_count = ctx.count * 8;
    // Pad with 0x80 followed by zeros, then 8-byte little-endian bit count.
    uint8_t pad_byte = 0x80;
    md5_update(ctx, &pad_byte, 1);
    uint8_t zero = 0;
    while (ctx.count % 64 != 56) {
        md5_update(ctx, &zero, 1);
    }
    uint8_t bits[8];
    std::memcpy(bits, &bit_count, 8);
    md5_update(ctx, bits, 8);

    std::array<uint8_t, 16> digest{};
    for (int i = 0; i < 4; ++i) {
        std::memcpy(digest.data() + i * 4, &ctx.state[i], 4);
    }
    return digest;
}

std::string md5_hex(const uint8_t* data, size_t len) {
    MD5Context ctx;
    md5_init(ctx);
    md5_update(ctx, data, len);
    auto digest = md5_finalize(ctx);
    static constexpr char hex_chars[] = "0123456789abcdef";
    std::string result;
    result.reserve(32);
    for (auto byte : digest) {
        result.push_back(hex_chars[(byte >> 4) & 0xF]);
        result.push_back(hex_chars[byte & 0xF]);
    }
    return result;
}

std::string trim(std::string_view sv) {
    auto start = sv.find_first_not_of(" \t\r\n");
    if (start == std::string_view::npos) return {};
    auto end = sv.find_last_not_of(" \t\r\n");
    return std::string(sv.substr(start, end - start + 1));
}

const std::regex kUuidRe(
    "^[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{12}$");

}  // namespace

std::string uuid_from_name(std::string_view name) {
    auto hex = md5_hex(reinterpret_cast<const uint8_t*>(name.data()), name.size());
    // Format: 8-4-4-4-12
    std::string result;
    result.reserve(36);
    result.append(hex, 0, 8);
    result.push_back('-');
    result.append(hex, 8, 4);
    result.push_back('-');
    result.append(hex, 12, 4);
    result.push_back('-');
    result.append(hex, 16, 4);
    result.push_back('-');
    result.append(hex, 20, 12);
    return result;
}

bool is_uuid(std::string_view value) {
    auto trimmed = trim(value);
    return std::regex_match(trimmed, kUuidRe);
}

std::string resolve_uuid(std::string_view value) {
    auto trimmed = trim(value);
    if (is_uuid(trimmed)) {
        std::string lower(trimmed);
        std::transform(lower.begin(), lower.end(), lower.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        return lower;
    }
    return uuid_from_name(trimmed);
}

std::string named_service_uuid(std::string_view name) {
    auto trimmed = trim(name);
    return uuid_from_name(trimmed);
}

std::string named_characteristic_uuid(std::string_view name) {
    auto trimmed = trim(name);
    return uuid_from_name(trimmed);
}

std::string named_descriptor_uuid(std::string_view name) {
    auto trimmed = trim(name);
    return uuid_from_name(trimmed);
}

}  // namespace mrs_uav_bluetooth::util
