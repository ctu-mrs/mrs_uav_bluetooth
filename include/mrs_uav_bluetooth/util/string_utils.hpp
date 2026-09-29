// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/util/string_utils.hpp
/// \brief Declares the string utils component of the shared utility layer.

#pragma once

#include <string>

namespace mrs_uav_bluetooth::util {

/// \brief Return text without leading or trailing ASCII whitespace.
/// \param value Text to trim.
/// \return Copy with surrounding ASCII whitespace removed.
std::string trim_ascii_copy(std::string value);

/// \brief Trim surrounding whitespace and lowercase a copy for policy comparisons.
/// \param value Text to trim.
/// \return Trimmed lowercase copy.
std::string lower_trim_copy(std::string value);

}  // namespace mrs_uav_bluetooth::util
