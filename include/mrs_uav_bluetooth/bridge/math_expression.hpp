// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/bridge/math_expression.hpp
/// \brief Declares the math expression component of the transport-independent ROS message bridge.

#pragma once

#include <functional>
#include <optional>
#include <string>

namespace mrs_uav_bluetooth::bridge {

/// Evaluate a small arithmetic relation from an overlay declaration.
/// Identifiers are resolved by the caller, so the same evaluator works on
/// source ROS fields during encoding and on named wire values during decoding.
/// Supported operators are +, -, *, and /. Supported functions include
/// sin, cos, asin, atan2, sqrt, abs, floor, round, min, max, and clamp.
/// \param expression arithmetic expression evaluated for a bridge member.
/// \param resolve_identifier callback that supplies a value for each expression identifier.
/// \return Finite numeric expression result.
double evaluate_math_expression(
    const std::string& expression,
    const std::function<double(const std::string&)>& resolve_identifier);

using ExactInteger = __int128_t;

/// Evaluate a relation exactly when all literals and resolved fields are
/// integers. The floating-point evaluator handles decimal literals,
/// floating-point fields, pi, and functions. Integer division truncates
/// toward zero and % gives the remainder. Overflow and division by zero fail.
/// \param expression arithmetic expression evaluated for a bridge member.
/// \param resolve_identifier callback that supplies a value for each expression identifier.
/// \return Exact integer result when no floating operation was required; otherwise std::nullopt.
std::optional<ExactInteger> try_evaluate_integer_expression(
    const std::string& expression,
    const std::function<std::optional<ExactInteger>(const std::string&)>& resolve_identifier);

}  // namespace mrs_uav_bluetooth::bridge
