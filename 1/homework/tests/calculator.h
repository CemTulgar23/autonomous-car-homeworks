#ifndef CALCULATOR_H
#define CALCULATOR_H

#include <string>
#include <optional>

template <typename T>
class Calculator {
public:
    static T addition (T num1, T num2);
    static T subtraction (T num1, T num2);
    static T multiplication (T num1, T num2);
    static std::optional<T> division (T num1, T num2);
    static T square (T num);
    static T exponentiation (T num, int exponent);
    static T modulus (T nmu1, T num2);
};

#endif
