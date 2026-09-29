// SPDX-License-Identifier: BSD-3-Clause
/// \file src/bridge/math_expression.cpp
/// \brief Implements the math expression component of the transport-independent ROS message bridge.

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
    /// \brief Bind the expression text to the resolver used for each identifier.
    /// \param input Floating-point expression text to parse.
    /// \param resolver Callback that resolves each field name to a numeric value.
    Parser(const std::string& input,
           const std::function<double(const std::string&)>& resolver)
        : input_(input), resolver_(resolver) {
            // Bind the expression text to the resolver used for each identifier.
        }

    /// \brief Evaluate the complete floating-point expression and reject trailing text.
    /// \return Fully evaluated expression value after requiring complete input consumption.
    double evaluate() {
        // A successful result must consume every token and remain finite.
        const double result = expression();
        whitespace();
        if (position_ != input_.size()) fail("Unexpected trailing text");
        if (!std::isfinite(result)) fail("Expression produced a non-finite value");
        return result;
    }

private:
    /// \brief Advance the parser cursor past ASCII whitespace.
    void whitespace() {
        // Advance the parser cursor past ASCII whitespace.
        while (position_ < input_.size() &&
               std::isspace(static_cast<unsigned char>(input_[position_]))) {
            ++position_;
        }
    }

    /// \brief Consume the requested token only when it begins at the current parser position.
    /// \param token Expected operator consumed only when present at the parser cursor.
    /// \return True when consume; otherwise false.
    bool consume(char token) {
        // Consume the requested token only when it begins at the current parser position.
        whitespace();
        if (position_ < input_.size() && input_[position_] == token) {
            ++position_;
            return true;
        }
        return false;
    }

    /// \brief Raise a parse error annotated with the current expression offset.
    /// \param reason Parser error appended with the current expression position.
    [[noreturn]] void fail(const std::string& reason) const {
        // Raise a parse error annotated with the current expression offset.
        throw std::runtime_error(reason + " at column " +
            std::to_string(position_ + 1) + " in expression: " + input_);
    }

    /// \brief Evaluate addition and subtraction over the parsed product terms.
    /// \return Sum or difference parsed at the current cursor.
    double expression() {
        // Evaluate addition and subtraction over the parsed product terms.
        double result = product();
        for (;;) {
            if (consume('+')) result += product();
            else if (consume('-')) result -= product();
            else return result;
        }
    }

    /// \brief Evaluate multiplication and division with higher precedence than addition.
    /// \return Product or quotient parsed at the current cursor.
    double product() {
        // Evaluate multiplication and division with higher precedence than addition.
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

    /// \brief Parse leading plus or minus operators before the next primary expression.
    /// \return Signed operand parsed at the current cursor.
    double unary() {
        // Parse leading plus or minus operators before the next primary expression.
        if (consume('+')) return unary();
        if (consume('-')) return -unary();
        return primary();
    }

    /// \brief Evaluate one supported mathematical function after arity validation.
    /// \param name Supported expression function name.
    /// \param args function arguments parsed or validated by the expression evaluator.
    /// \return Result of the validated built-in function call.
    double call(const std::string& name, const std::vector<double>& args) {
        // Enforce each built-in function’s fixed argument count before indexing.
        const auto require = [&](size_t count) {
            // Reject a function call whose argument count differs from its mathematical signature.
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

    /// \brief Parse a number, identifier, function call, or parenthesized expression.
    /// \return Literal parenthesized expression or variable parsed at the current cursor.
    double primary() {
        // Parse a number, identifier, function call, or parenthesized expression.
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
    /// \brief Bind the integer expression to a resolver that can decline nonintegral identifiers.
    /// \param input Exact-integer expression text to parse.
    /// \param resolver Callback that resolves integral fields without rounding.
    IntegerParser(const std::string& input,
                  const std::function<std::optional<ExactInteger>(const std::string&)>& resolver)
        : input_(input), resolver_(resolver) {
            // Bind the integer expression to a resolver that can decline nonintegral identifiers.
        }

    /// \brief Evaluate the complete exact-integer expression and reject trailing text.
    /// \return Fully evaluated expression value after requiring complete input consumption.
    ExactInteger evaluate() {
        // Exact evaluation succeeds only when every input token is consumed.
        const auto result = expression();
        whitespace();
        if (position_ != input_.size()) fail("Unexpected trailing text");
        return result;
    }

private:
    /// Largest magnitude representable by the signed exact-integer accumulator.
    static constexpr auto kMaximum = static_cast<ExactInteger>(
        static_cast<unsigned __int128>(-1) >> 1);
    static constexpr auto kMinimum = -kMaximum - 1;

    /// \brief Advance the parser cursor past ASCII whitespace.
    void whitespace() {
        // Advance the parser cursor past ASCII whitespace.
        while (position_ < input_.size() &&
               std::isspace(static_cast<unsigned char>(input_[position_]))) ++position_;
    }

    /// \brief Consume the requested token only when it begins at the current parser position.
    /// \param token Expected operator consumed only when present at the parser cursor.
    /// \return True when consume; otherwise false.
    bool consume(char token) {
        // Consume the requested token only when it begins at the current parser position.
        whitespace();
        if (position_ < input_.size() && input_[position_] == token) {
            ++position_;
            return true;
        }
        return false;
    }

    /// \brief Raise a parse error annotated with the current expression offset.
    /// \param reason Parser error appended with the current expression position.
    [[noreturn]] void fail(const std::string& reason) const {
        // Raise a parse error annotated with the current expression offset.
        throw std::runtime_error(reason + " at column " +
            std::to_string(position_ + 1) + " in expression: " + input_);
    }

    /// \brief Evaluate addition and subtraction over the parsed product terms.
    /// \return Sum or difference parsed at the current cursor.
    ExactInteger expression() {
        // Evaluate addition and subtraction over the parsed product terms.
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

    /// \brief Evaluate multiplication and division with higher precedence than addition.
    /// \return Product or quotient parsed at the current cursor.
    ExactInteger product() {
        // Evaluate multiplication and division with higher precedence than addition.
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

    /// \brief Parse leading plus or minus operators before the next primary expression.
    /// \return Signed operand parsed at the current cursor.
    ExactInteger unary() {
        // Parse leading plus or minus operators before the next primary expression.
        if (consume('+')) return unary();
        if (consume('-')) {
            const auto value = unary();
            if (value == kMinimum) fail("Integer overflow");
            return -value;
        }
        return primary();
    }

    /// \brief Parse a number, identifier, function call, or parenthesized expression.
    /// \return Literal parenthesized expression or variable parsed at the current cursor.
    ExactInteger primary() {
        // Parse a number, identifier, function call, or parenthesized expression.
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

/// \brief Parse and evaluate a finite arithmetic expression with named ROS fields.
/// \param expression arithmetic expression evaluated for a bridge member.
/// \param resolve_identifier callback that supplies a value for each expression identifier.
/// \return Finite numeric expression result.
double evaluate_math_expression(
    const std::string& expression,
    const std::function<double(const std::string&)>& resolve_identifier) {
    // Use the finite floating parser for expressions that may include trigonometry.
    return Parser(expression, resolve_identifier).evaluate();
}

/// \brief Evaluate an expression exactly only when every operand and operation stays integral.
/// \param expression arithmetic expression evaluated for a bridge member.
/// \param resolve_identifier callback that supplies a value for each expression identifier.
/// \return Exact integer result when no floating operation was required; otherwise std::nullopt.
std::optional<ExactInteger> try_evaluate_integer_expression(
    const std::string& expression,
    const std::function<std::optional<ExactInteger>(const std::string&)>& resolve_identifier) {
    // Evaluate an expression exactly only when every operand and operation stays integral.
    try {
        return IntegerParser(expression, resolve_identifier).evaluate();
    } catch (const FloatingExpression&) {
        return std::nullopt;
    }
}

}  // namespace mrs_uav_bluetooth::bridge
