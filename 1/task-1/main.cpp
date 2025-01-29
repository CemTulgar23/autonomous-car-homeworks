#include <iostream>
#include <string>
#include <limits>
#include <algorithm>
#include "tests/calculator.h"

template <typename T>
T number;

int main()
{
    bool status = true;
    while (status == true)
    {
        std::cout << "WELCOME TO CALCULATOR" << std::endl;
        std::cout << "for addition press +" << std::endl;
        std::cout << "for subtraction press -" << std::endl;
        std::cout << "for multiplication press *" << std::endl;
        std::cout << "for division press /" << std::endl;
        std::cout << "for square press sq" << std::endl;
        std::cout << "for exponentiation press ex" << std::endl;
        std::cout << "for modulus press md" << std::endl;

        std::string oprt;

        std::cout << "Please enter an operator: " << std::endl;
        std::cin >> oprt;

        while (oprt != "+" && oprt != "-" && oprt != "*" && oprt != "/" && oprt != "sq" && oprt != "ex" && oprt != "md")
        {
            std::cout << "Please enter an operator that is given above" << std::endl;
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cin >> oprt;
        }

        if (oprt == "+")
        {
            std::string input1, input2;
            std::cout << "Please enter first number: " << std::endl;
            std::cin >> input1;
            std::replace(input1.begin(), input1.end(), ',', '.');
            std::cout << "Please enter second number: " << std::endl;
            std::cin >> input2;
            std::replace(input2.begin(), input2.end(), ',', '.');

            while (true)
            {
                try
                {
                    double num1 = std::stod(input1);
                    double num2 = std::stod(input2);
                    number<double> = Calculator<double>::addition(num1, num2);
                    std::cout << "Result: " << number<double> << std::endl;
                    break;
                }
                catch (const std::invalid_argument &e)
                {
                    std::cout << "Invalid input. Please enter a valid number." << std::endl;
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    std::cout << "Please enter first number: " << std::endl;
                    std::cin >> input1;
                    std::replace(input1.begin(), input1.end(), ',', '.');
                    std::cout << "Please enter second number: " << std::endl;
                    std::cin >> input2;
                    std::replace(input2.begin(), input2.end(), ',', '.');
                }
                catch (const std::out_of_range &e)
                {
                    std::cout << "The entered number is out of range. Try again." << std::endl;
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    std::cout << "Please enter first number: " << std::endl;
                    std::cin >> input1;
                    std::replace(input1.begin(), input1.end(), ',', '.');
                    std::cout << "Please enter second number: " << std::endl;
                    std::cin >> input2;
                    std::replace(input2.begin(), input2.end(), ',', '.');
                }
            }
        }

        if (oprt == "-")
        {
            std::string input1, input2;
            std::cout << "Please enter first number: " << std::endl;
            std::cin >> input1;
            std::replace(input1.begin(), input1.end(), ',', '.');
            std::cout << "Please enter second number: " << std::endl;
            std::cin >> input2;
            std::replace(input2.begin(), input2.end(), ',', '.');

            while (true)
            {
                try
                {
                    double num1 = std::stod(input1);
                    double num2 = std::stod(input2);
                    number<double> = Calculator<double>::subtraction(num1, num2);
                    std::cout << "Result: " << number<double> << std::endl;
                    break;
                }
                catch (const std::invalid_argument &e)
                {
                    std::cout << "Invalid input. Please enter a valid number." << std::endl;
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    std::cout << "Please enter first number: " << std::endl;
                    std::cin >> input1;
                    std::replace(input1.begin(), input1.end(), ',', '.');
                    std::cout << "Please enter second number: " << std::endl;
                    std::cin >> input2;
                    std::replace(input2.begin(), input2.end(), ',', '.');
                }
                catch (const std::out_of_range &e)
                {
                    std::cout << "The entered number is out of range. Try again." << std::endl;
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    std::cout << "Please enter first number: " << std::endl;
                    std::cin >> input1;
                    std::replace(input1.begin(), input1.end(), ',', '.');
                    std::cout << "Please enter second number: " << std::endl;
                    std::cin >> input2;
                    std::replace(input2.begin(), input2.end(), ',', '.');
                }
            }
        }

        if (oprt == "*")
        {
            std::string input1, input2;
            std::cout << "Please enter first number: " << std::endl;
            std::cin >> input1;
            std::replace(input1.begin(), input1.end(), ',', '.');
            std::cout << "Please enter second number: " << std::endl;
            std::cin >> input2;
            std::replace(input2.begin(), input2.end(), ',', '.');

            while (true)
            {
                try
                {
                    double num1 = std::stod(input1);
                    double num2 = std::stod(input2);
                    number<double> = Calculator<double>::multiplication(num1, num2);
                    std::cout << "Result: " << number<double> << std::endl;
                    break;
                }
                catch (const std::invalid_argument &e)
                {
                    std::cout << "Invalid input. Please enter a valid number." << std::endl;
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    std::cout << "Please enter first number: " << std::endl;
                    std::cin >> input1;
                    std::replace(input1.begin(), input1.end(), ',', '.');
                    std::cout << "Please enter second number: " << std::endl;
                    std::cin >> input2;
                    std::replace(input2.begin(), input2.end(), ',', '.');
                }
                catch (const std::out_of_range &e)
                {
                    std::cout << "The entered number is out of range. Try again." << std::endl;
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    std::cout << "Please enter first number: " << std::endl;
                    std::cin >> input1;
                    std::replace(input1.begin(), input1.end(), ',', '.');
                    std::cout << "Please enter second number: " << std::endl;
                    std::cin >> input2;
                    std::replace(input2.begin(), input2.end(), ',', '.');
                }
            }
        }

        if (oprt == "/")
        {
            std::string input1, input2;
            std::cout << "Please enter first number: " << std::endl;
            std::cin >> input1;
            std::replace(input1.begin(), input1.end(), ',', '.');
            std::cout << "Please enter second number: " << std::endl;
            std::cin >> input2;
            std::replace(input2.begin(), input2.end(), ',', '.');

            while (true)
            {
                try
                {
                    double num1 = std::stod(input1);
                    double num2 = std::stod(input2);

                    number<std::optional<double>> = Calculator<double>::division(num1, num2);
                    if (!number<std::optional<double>>.has_value())
                    {
                        std::cout << "A number can't be divided zero" << std::endl;
                    }
                    else
                    {
                        std::cout << "Result: " << number<std::optional<double>>.value() << std::endl;
                    }
                    break;
                }
                catch (const std::invalid_argument &e)
                {
                    std::cout << "Invalid input. Please enter a valid number." << std::endl;
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    std::cout << "Please enter first number: " << std::endl;
                    std::cin >> input1;
                    std::replace(input1.begin(), input1.end(), ',', '.');
                    std::cout << "Please enter second number: " << std::endl;
                    std::cin >> input2;
                    std::replace(input2.begin(), input2.end(), ',', '.');
                }
                catch (const std::out_of_range &e)
                {
                    std::cout << "The entered number is out of range. Try again." << std::endl;
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    std::cout << "Please enter first number: " << std::endl;
                    std::cin >> input1;
                    std::replace(input1.begin(), input1.end(), ',', '.');
                    std::cout << "Please enter second number: " << std::endl;
                    std::cin >> input2;
                    std::replace(input2.begin(), input2.end(), ',', '.');
                }
            }
        }

        if (oprt == "sq")
        {
            std::string input;
            std::cout << "Please enter a number: " << std::endl;
            std::cin >> input;
            std::replace(input.begin(), input.end(), ',', '.');

            while (true)
            {
                try
                {
                    double num = std::stod(input);
                    number<double> = Calculator<double>::square(num);
                    std::cout << "Result: " << number<double> << std::endl;
                    break;
                }
                catch (const std::invalid_argument &e)
                {
                    std::cout << "Invalid input. Please enter a valid number." << std::endl;
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    std::cout << "Please enter a number: " << std::endl;
                    std::cin >> input;
                    std::replace(input.begin(), input.end(), ',', '.');
                }
                catch (const std::out_of_range &e)
                {
                    std::cout << "The entered number is out of range. Try again." << std::endl;
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    std::cout << "Please enter a number: " << std::endl;
                    std::cin >> input;
                }
            }
        }

        if (oprt == "ex")
        {
            std::string input1, input2;
            std::cout << "Please enter base number: " << std::endl;
            std::cin >> input1;
            std::replace(input1.begin(), input1.end(), ',', '.');
            std::cout << "Please enter exponent number: " << std::endl;
            std::cin >> input2;
            std::replace(input2.begin(), input2.end(), ',', '.');

            while (true)
            {
                try
                {
                    double num = std::stod(input1);
                    double exponent = std::stod(input2);
                    number<double> = Calculator<double>::exponentiation(num, exponent);
                    std::cout << "Result: " << number<double> << std::endl;
                    break;
                }
                catch (const std::invalid_argument &e)
                {
                    std::cout << "Invalid input. Please enter a valid number." << std::endl;
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    std::cout << "Please enter base number: " << std::endl;
                    std::cin >> input1;
                    std::replace(input1.begin(), input1.end(), ',', '.');
                    std::cout << "Please enter exponent number: " << std::endl;
                    std::cin >> input2;
                    std::replace(input2.begin(), input2.end(), ',', '.');
                }
                catch (const std::out_of_range &e)
                {
                    std::cout << "The entered number is out of range. Try again." << std::endl;
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    std::cout << "Please enter base number: " << std::endl;
                    std::cin >> input1;
                    std::replace(input1.begin(), input1.end(), ',', '.');
                    std::cout << "Please enter exponent number: " << std::endl;
                    std::cin >> input2;
                    std::replace(input2.begin(), input2.end(), ',', '.');
                }
            }
        }

        if (oprt == "md")
        {
            std::string input1, input2;
            std::cout << "Please enter first number: " << std::endl;
            std::cin >> input1;
            std::replace(input1.begin(), input1.end(), ',', '.');
            std::cout << "Please enter second number: " << std::endl;
            std::cin >> input2;
            std::replace(input2.begin(), input2.end(), ',', '.');

            while (true)
            {
                try
                {
                    double num1 = std::stod(input1);
                    double num2 = std::stod(input2);
                    number<double> = Calculator<double>::modulus(num1, num2);
                    std::cout << "Result: " << number<double> << std::endl;
                    break;
                }
                catch (const std::invalid_argument &e)
                {
                    std::cout << "Invalid input. Please enter a valid number." << std::endl;
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    std::cout << "Please enter first number: " << std::endl;
                    std::cin >> input1;
                    std::replace(input1.begin(), input1.end(), ',', '.');
                    std::cout << "Please enter second number: " << std::endl;
                    std::cin >> input2;
                    std::replace(input2.begin(), input2.end(), ',', '.');
                }
                catch (const std::out_of_range &e)
                {
                    std::cout << "The entered number is out of range. Try again." << std::endl;
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    std::cout << "Please enter first number: " << std::endl;
                    std::cin >> input1;
                    std::replace(input1.begin(), input1.end(), ',', '.');
                    std::cout << "Please enter second number: " << std::endl;
                    std::cin >> input2;
                    std::replace(input2.begin(), input2.end(), ',', '.');
                }
            }
        }

        std::string iscalculate;
        std::cout << "Do you want to continue calculate? (y/n)" << std::endl;
        std::cin >> iscalculate;
        while (iscalculate != "y" && iscalculate != "n")
        {
            std::cout << "Wrong Input!" << std::endl;
            std::cout << "Do you want to continue calculate? (y/n)" << std::endl;
            std::cin >> iscalculate;
        }
        if (iscalculate == "n")
        {
            status = false;
        }
    }
}