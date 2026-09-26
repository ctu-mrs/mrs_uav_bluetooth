// SPDX-License-Identifier: BSD-3-Clause
#include "mrs_uav_bluetooth/bridge/math_expression.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <numbers>
#include <stdexcept>
#include <vector>

namespace mrs_uav_bluetooth::bridge {

namespace {

/// A deliberately small expression parser. It contains no assignment,
/// filesystem access, or external interpreter dependency. Only the numeric
/// relations needed to map ROS fields to fixed-width wire scalars are legal.
class Parser {
public:
    Parser(const std::string& input,
           const std::function<double(const std::string&)>& resolver)
        : input_(input), resolver_(resolver) {}

    double evaluate() {
        const double result = expression();
        whitespace();
        if (position_ != input_.size()) fail("Unexpected trailing text");
        if (!std::isfinite(result)) fail("Expression produced a non-finite value");
        return result;
    }

private:
    void whitespace() {
        while (position_ < input_.size() &&
               std::isspace(static_cast<unsigned char>(input_[position_]))) {
            ++position_;
        }
    }

    bool consume(char token) {
        whitespace();
        if (position_ < input_.size() && input_[position_] == token) {
            ++position_;
            return true;
        }
        return false;
    }

    [[noreturn]] void fail(const std::string& reason) const {
        throw std::runtime_error(reason + " at column " +
            std::to_string(position_ + 1) + " in expression: " + input_);
    }

    double expression() {
        double result = product();
        for (;;) {
            if (consume('+')) result += product();
            else if (consume('-')) result -= product();
            else return result;
        }
    }

    double product() {
        double result = unary();
        for (;;) {
            if (consume('*')) result *= unary();
            else if (consume('/')) {
                const double denominator = unary();
                if (denominator == 0.0) fail("Division by zero");
                result /= denominator;
            } else return result;
        }
    }

    double unary() {
        if (consume('+')) return unary();
        if (consume('-')) return -unary();
        return primary();
    }

    double call(const std::string& name, const std::vector<double>& args) {
        const auto require = [&](size_t count) {
            if (args.size() != count) fail(name + " expects " + std::to_string(count) + " arguments");
        };
        if (name == "sin") { require(1); return std::sin(args[0]); }
        if (name == "cos") { require(1); return std::cos(args[0]); }
        if (name == "asin") { require(1); return std::asin(args[0]); }
        if (name == "atan2") { require(2); return std::atan2(args[0], args[1]); }
        if (name == "sqrt") { require(1); return std::sqrt(args[0]); }
        if (name == "abs") { require(1); return std::abs(args[0]); }
        if (name == "floor") { require(1); return std::floor(args[0]); }
        if (name == "round") { require(1); return std::round(args[0]); }
        if (name == "min") { require(2); return std::min(args[0], args[1]); }
        if (name == "max") { require(2); return std::max(args[0], args[1]); }
        if (name == "clamp") { require(3); return std::clamp(args[0], args[1], args[2]); }
        fail("Unknown function " + name);
    }

    double primary() {
        whitespace();
        if (consume('(')) {
            const double result = expression();
            if (!consume(')')) fail("Missing closing parenthesis");
            return result;
        }
        if (position_ >= input_.size()) fail("Expected number or identifier");

        const auto current = static_cast<unsigned char>(input_[position_]);
        if (std::isdigit(current) || current == '.') {
            char* end = nullptr;
            const double result = std::strtod(input_.c_str() + position_, &end);
            if (end == input_.c_str() + position_) fail("Invalid number");
            position_ = static_cast<size_t>(end - input_.c_str());
            return result;
        }
        if (!std::isalpha(current) && current != '_') fail("Expected identifier");
        const size_t begin = position_++;
        while (position_ < input_.size()) {
            const auto next = static_cast<unsigned char>(input_[position_]);
            if (!std::isalnum(next) && next != '_' && next != '.' &&
                next != '[' && next != ']') break;
            ++position_;
        }
        const std::string name = input_.substr(begin, position_ - begin);
        if (consume('(')) {
            std::vector<double> args;
            if (!consume(')')) {
                do { args.push_back(expression()); } while (consume(','));
                if (!consume(')')) fail("Missing function closing parenthesis");
            }
            return call(name, args);
        }
        if (name == "pi") return std::numbers::pi;
        return resolver_(name);
    }

    const std::string& input_;
    const std::function<double(const std::string&)>& resolver_;
    size_t position_{0};
};

struct FloatingExpression {};

/// Keep integer-only relations exact beyond the 53-bit mantissa of double.
/// This matters for current Unix nanoseconds, which already exceed that range.
class IntegerParser {
public:
    IntegerParser(const std::string& input,
                  const std::function<std::optional<ExactInteger>(const std::string&)>& resolver)
        : input_(input), resolver_(resolver) {}

    ExactInteger evaluate() {
        const auto result = expression();
        whitespace();
        if (position_ != input_.size()) fail("Unexpected trailing text");
        return result;
    }

private:
    static constexpr auto kMaximum = static_cast<ExactInteger>(
        static_cast<unsigned __int128>(-1) >> 1);
    static constexpr auto kMinimum = -kMaximum - 1;

    void whitespace() {
        while (position_ < input_.size() &&
               std::isspace(static_cast<unsigned char>(input_[position_]))) ++position_;
    }

    bool consume(char token) {
        whitespace();
        if (position_ < input_.size() && input_[position_] == token) {
            ++position_;
            return true;
        }
        return false;
    }

    [[noreturn]] void fail(const std::string& reason) const {
        throw std::runtime_error(reason + " at column " +
            std::to_string(position_ + 1) + " in expression: " + input_);
    }

    ExactInteger expression() {
        auto result = product();
        for (;;) {
            if (consume('+')) {
                const auto rhs = product();
                if (__builtin_add_overflow(result, rhs, &result)) fail("Integer overflow");
            } else if (consume('-')) {
                const auto rhs = product();
                if (__builtin_sub_overflow(result, rhs, &result)) fail("Integer overflow");
            } else return result;
        }
    }

    ExactInteger product() {
        auto result = unary();
        for (;;) {
            if (consume('*')) {
                const auto rhs = unary();
                if (__builtin_mul_overflow(result, rhs, &result)) fail("Integer overflow");
            } else if (consume('/') || consume('%')) {
                const auto op = input_[position_ - 1];
                const auto rhs = unary();
                if (rhs == 0) fail("Division by zero");
                if (result == kMinimum && rhs == -1) fail("Integer overflow");
                result = op == '/' ? result / rhs : result % rhs;
            } else return result;
        }
    }

    ExactInteger unary() {
        if (consume('+')) return unary();
        if (consume('-')) {
            const auto value = unary();
            if (value == kMinimum) fail("Integer overflow");
            return -value;
        }
        return primary();
    }

    ExactInteger primary() {
        whitespace();
        if (consume('(')) {
            const auto result = expression();
            if (!consume(')')) fail("Missing closing parenthesis");
            return result;
        }
        if (position_ >= input_.size()) fail("Expected number or identifier");
        const auto current = static_cast<unsigned char>(input_[position_]);
        if (current == '.') throw FloatingExpression{};
        if (std::isdigit(current)) {
            ExactInteger value = 0;
            do {
                const auto digit = static_cast<ExactInteger>(input_[position_] - '0');
                if (__builtin_mul_overflow(value, static_cast<ExactInteger>(10), &value) ||
                    __builtin_add_overflow(value, digit, &value)) fail("Integer literal overflow");
                ++position_;
            } while (position_ < input_.size() &&
                     std::isdigit(static_cast<unsigned char>(input_[position_])));
            if (position_ < input_.size() &&
                (input_[position_] == '.' || input_[position_] == 'e' ||
                 input_[position_] == 'E')) throw FloatingExpression{};
            return value;
        }
        if (!std::isalpha(current) && current != '_') fail("Expected identifier");
        const size_t begin = position_++;
        while (position_ < input_.size()) {
            const auto next = static_cast<unsigned char>(input_[position_]);
            if (!std::isalnum(next) && next != '_' && next != '.' &&
                next != '[' && next != ']') break;
            ++position_;
        }
        const auto name = input_.substr(begin, position_ - begin);
        whitespace();
        if (position_ < input_.size() && input_[position_] == '(') {
            throw FloatingExpression{};
        }
        if (name == "pi") throw FloatingExpression{};
        const auto value = resolver_(name);
        if (!value) throw FloatingExpression{};
        return *value;
    }

    const std::string& input_;
    const std::function<std::optional<ExactInteger>(const std::string&)>& resolver_;
    size_t position_{0};
};

}  // namespace

double evaluate_math_expression(
    const std::string& expression,
    const std::function<double(const std::string&)>& resolve_identifier) {
    return Parser(expression, resolve_identifier).evaluate();
}

std::optional<ExactInteger> try_evaluate_integer_expression(
    const std::string& expression,
    const std::function<std::optional<ExactInteger>(const std::string&)>& resolve_identifier) {
    try {
        return IntegerParser(expression, resolve_identifier).evaluate();
    } catch (const FloatingExpression&) {
        return std::nullopt;
    }
}

}  // namespace mrs_uav_bluetooth::bridge
