#include <iostream>
#include <string>
#include <optional>
#include <cmath>
#include "calculator.h"

template <typename T>
T Calculator<T>::addition(T num1, T num2) {
    return num1 + num2;
}

template <typename T>
T Calculator<T>::subtraction(T num1, T num2) {
    return num1 - num2;
}

template <typename T>
T Calculator<T>::multiplication(T num1, T num2) {
    return num1 * num2;
}

template <typename T>
std::optional<T> Calculator<T>::division(T num1, T num2) {
    return num2 == 0 ? std::nullopt : std::optional<T>(num1 / num2);
}

template <typename T>
T Calculator<T>::square(T num) {
    return num * num;
}

template <typename T>
T Calculator<T>::exponentiation(T num, int exponent) {
    return pow(num, exponent);
}

template <typename T>
T Calculator<T>::modulus(T nmu1, T num2) {
    return fmod(nmu1, num2);   
}

template class Calculator<int>;
template class Calculator<double>;
template class Calculator<float>;


